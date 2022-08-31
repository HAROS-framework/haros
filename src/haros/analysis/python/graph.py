# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Iterable, List, NewType, Optional, Set, Tuple, Union

from attrs import define, field, frozen

from haros.analysis.python.logic import to_condition
from haros.metamodel.logic import TRUE, LogicValue
from haros.parsing.python.ast import (
    PythonAst,
    PythonBooleanLiteral,
    PythonConditionalBlock,
    PythonForStatement,
    PythonModule,
    PythonStatement,
    PythonWhileStatement,
    PythonTryStatement,
)

###############################################################################
# Data Structures
###############################################################################


@define
class Scope:
    pass


ControlNodeId = NewType('ControlNodeId', int)


@define
class ControlNode:
    id: ControlNodeId
    body: List[PythonStatement] = field(factory=list)
    condition: LogicValue = field(factory=LogicValue.tautology)
    incoming: Set[ControlNodeId] = field(factory=set)
    outgoing: Set[ControlNodeId] = field(factory=set)

    @property
    def is_empty(self) -> bool:
        return not self.body

    @property
    def is_terminal(self) -> bool:
        return not self.outgoing

    @property
    def is_unreachable(self) -> bool:
        return not self.incoming

    @property
    def is_conditional(self) -> bool:
        return not self.condition.is_true and not self.condition.is_false

    def append(self, statement: PythonStatement):
        self.body.append(statement)

    def jump_to(self, other: 'ControlNode'):
        self.outgoing.add(other.id)
        other.incoming.add(self.id)


@define
class ControlFlowGraph:
    nodes: List[ControlNode] = field(factory=list)
    is_asynchronous: bool = False

    @property
    def first_node(self) -> ControlNode:
        return self.nodes[0]

    @property
    def last_node(self) -> ControlNode:
        return self.nodes[-1]


###############################################################################
# Graph Builder
###############################################################################


@define
class ControlFlowGraphBuilder:
    cfg: ControlFlowGraph = field(factory=ControlFlowGraph)

    def start(self):
        return

    def add_statement(
        self,
        statement: PythonStatement,
        loop_node: Optional[ControlNode] = None,
        post_loop_node: Optional[ControlNode] = None,
    ):
        this_node = self.current_node
        this_node.append(statement)

        if statement.is_break:
            assert loop_node is not None, 'break outside of loop'
            assert post_loop_node is not None, 'break outside of loop'
            this_node.jump_to(post_loop_node)
            self._new_node()  # dead code

        elif statement.is_continue:
            assert loop_node is not None, 'continue outside of loop'
            assert post_loop_node is not None, 'continue outside of loop'
            this_node.jump_to(loop_node)
            self._new_node()  # dead code

        elif statement.is_return or statement.is_raise:
            self._new_node()  # dead code

        elif statement.is_yield:
            self.cfg.is_asynchronous = True
            node = self._new_node(origin=this_node)

        elif statement.is_assert:
            phi = to_condition(statement.condition)
            # terminal node
            self._new_node(condition=phi.negate(), origin=this_node)
            # node that continues the flow
            self._new_node(condition=phi, origin=this_node)

        elif statement.is_if:
            self._build_if(statement, loop_node, post_loop_node)

        elif statement.is_while:
            builder = WhileFlowBuilder(statement)

        elif statement.is_for:
            return False
        elif statement.is_try:
            return False
        elif statement.is_match:
            return False
        elif statement.is_with:
            return False
        elif statement.is_function_def:
            return False
        elif statement.is_class_def:
            return
        return True

    def build(self) -> ControlFlowGraph:
        cfg = self.cfg
        self.cfg = _cfg_factory()
        return cfg

    @property
    def current_node(self) -> ControlNode:
        return self.cfg.nodes[-1]

    def _new_node(
        self,
        condition: LogicValue = TRUE,
        origin: Optional[ControlNode] = None,
    ) -> ControlNode:
        uid = ControlNodeId(len(self.cfg.nodes))
        condition = self.current_node.condition.join(condition)
        node = ControlNode(uid, condition=condition)
        self.cfg.nodes.append(node)
        if origin:
            origin.jump_to(node)
        return node

    def _build_if(
        self,
        statement: PythonStatement,
        loop_node: Optional[ControlNode],
        post_loop_node: Optional[ControlNode],
    ):
        ns = []
        self._build_branch(this_node, ns, statement.then_branch, loop_node, post_loop_node)
        for branch in statement.elif_branches:
            self._build_branch(this_node, ns, branch, loop_node, post_loop_node)
        if statement.has_else_branch:
            self._build_branch(this_node, ns, statement.else_branch, loop_node, post_loop_node)
        self._new_node(condition=this_node.condition)
        for node in ns:


        # Create a block for the code after the if-else.
        afterif_block = self.new_block()

        # New block for the body of the else if there is an else clause.
        if len(node.orelse) != 0:
            else_block = self.new_block()
            self.add_exit(self.current_block, else_block, invert(node.test))
            self.current_block = else_block
            # Visit the children in the body of the else to populate the block.
            for child in node.orelse:
                self.visit(child)
            # If encountered a break, exit will have already been added
            if not self.current_block.exits:
                self.add_exit(self.current_block, afterif_block)
        else:
            self.add_exit(self.current_block, afterif_block, invert(node.test))

        # Visit children to populate the if block.
        self.current_block = if_block
        for child in node.body:
            self.visit(child)
        if not self.current_block.exits:
            self.add_exit(self.current_block, afterif_block)

        # Continue building the CFG in the after-if block.
        self.current_block = afterif_block

    def _build_branch(
        self,
        node_buffer: List[ControlNode],
        branch: PythonConditionalBlock,
        loop_node: Optional[ControlNode],
        post_loop_node: Optional[ControlNode],
    ):
        origin = self.current_node
        i = len(self.cfg.nodes) - 1
        phi = to_condition(branch.condition)
        node = self._new_node(condition=phi, origin=origin)
        node_buffer.append(node)
        for statement in branch.body:
            self.add_statement(statement, loop_node, post_loop_node)
        del self.cfg.nodes[i]
        self.cfg.nodes.append(origin)

    def _build_while(self, statement: PythonWhileStatement):
        loop_node = self.current_node
        self._new_node(statement.loop.condition)
        for stmt in statement.loop.body:
            self.add_statement(stmt)
        pass


###############################################################################
# Interface
###############################################################################


def from_ast(ast: PythonAst) -> Scope:
    if not ast.is_statement:
        raise TypeError(f'expected a statement, got {ast!r}')
    if ast.is_module:
        return from_module(ast)
    raise TypeError(f'unexpected tree node: {ast!r}')


def from_module(module: PythonModule) -> Scope:
    return
