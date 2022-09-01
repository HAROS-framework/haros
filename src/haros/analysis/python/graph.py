# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Dict, Iterable, List, NewType, Optional, Set, Tuple, Union

from attrs import define, field, frozen

from haros.analysis.python.logic import to_condition
from haros.metamodel.logic import FALSE, TRUE, LogicValue
from haros.parsing.python.ast import (
    PythonAst,
    PythonBooleanLiteral,
    PythonExpression,
    PythonForStatement,
    PythonModule,
    PythonStatement,
    PythonWhileStatement,
    PythonTryStatement,
)

###############################################################################
# Data Structures
###############################################################################

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


class MalformedProgramError(Exception):
    @classmethod
    def break_without_loop(cls) -> 'MalformedProgramError':
        return cls('found a break statement outside of a loop')

    @classmethod
    def continue_without_loop(cls) -> 'MalformedProgramError':
        return cls('found a continue statement outside of a loop')


@define
class ProgramContext:
    current_node: ControlNode

    def must_break(self):
        raise MalformedProgramError.break_without_loop()

    def must_continue(self):
        raise MalformedProgramError.continue_without_loop()


@define
class LoopContext(ProgramContext):
    begin_loop_node: ControlNode
    post_loop_node: ControlNode

    def must_break(self):
        self.current_node.jump_to(self.post_loop_node)

    def must_continue(self):
        self.current_node.jump_to(self.begin_loop_node)


@define
class BranchingContext(ProgramContext):
    post_branch_node: ControlNode
    branch_nodes: List[ControlNode] = field(factory=list)

    def add_branch(self, node: ControlNode) -> int:
        n = len(self.branches)
        self.branches.append(node)
        self.current_node.jump_to(node)
        return n


# ControlFlowGraphBuilder is ready to `add_statement` after initialisation.
# It keeps a stack of `ProgramContext` upon which it builds new nodes.


@define
class ControlFlowGraphBuilder:
    _nodes: Dict[ControlNodeId, ControlNode] = field(factory=dict)
    _context_stack: List[ProgramContext] = field(init=False, factory=list)
    _start: ControlNodeId = ControlNodeId(0)

    def __attrs_post_init__(self):
        node = self._new_node(condition=TRUE)
        self._start = node.id
        context = ProgramContext(node)
        self._context_stack.append(context)

    @property
    def context(self) -> ProgramContext:
        return self._context_stack[-1]

    @property
    def current_node(self) -> ControlNode:
        self._context_stack[-1].current_node

    def add_statement(self, statement: PythonStatement):
        context = self.context
        this_node = context.current_node
        this_node.append(statement)

        if statement.is_break:
            context.must_break()
            self._start_dead_code()

        elif statement.is_continue:
            context.must_continue()
            self._start_dead_code()

        elif statement.is_return or statement.is_raise:
            self._start_dead_code()

        elif statement.is_yield:
            self.cfg.is_asynchronous = True
            node = self._new_node(origin=this_node)

        elif statement.is_assert:
            phi = to_condition(statement.condition)
            # terminal node
            self._new_node(condition=phi.negate(), origin=this_node)
            # node that continues the flow
            context.current_node = self._new_node(condition=phi, origin=this_node)

        elif statement.is_if:
            self._build_if(statement)

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

    def _new_node(
        self,
        condition: Optional[LogicValue] = None,
        origin: Optional[ControlNode] = None,
    ) -> ControlNode:
        uid = ControlNodeId(len(self._nodes))
        if condition is None:
            condition = self.current_node.condition
        node = ControlNode(uid, condition=condition)
        self._nodes[uid] = node
        if origin is not None:
            origin.jump_to(node)
        return node

    def _start_dead_code(self):
        # push a new node that propagates the current condition,
        # but has no incoming links from other nodes
        self.context.current_node = self._new_node()

    def _push_context(self, node: ControlNode) -> ProgramContext:
        new_context = ProgramContext(node)
        self._context_stack.append(new_context)
        return new_context

    def _go_back_to(self, context: ProgramContext):
        while self._context_stack:
            other = self._context_stack.pop()
            if other is context:
                return
        else:
            raise AssertionError(f'missing context: {context!r}')

    def _build_if(self, statement: PythonStatement):
        context = self.context
        this_node = context.current_node

        phi = to_condition(statement.then_branch.condition)
        psi = this_node.condition.join(phi)
        branch_node = self._new_node(condition=psi, origin=this_node)
        self._push_context(branch_node)
        for stmt in statement.then_branch.body:
            self.add_statement(stmt)
        self._go_back_to(context)

        rho = statement.then_branch.condition.negate()  # accumulator
        for branch in statement.elif_branches:
            phi = rho.join(branch.condition)
            psi = this_node.condition.join(phi)
            branch_node = self._new_node(condition=psi, origin=this_node)
            self._push_context(branch_node)
            for stmt in branch.body:
                self.add_statement(stmt)
            self._go_back_to(context)
            rho = rho.join(branch.condition.negate())

        if statement.has_else_branch:
            self._build_branch(this_node, ns, statement.else_branch, loop_node, post_loop_node)
        self._new_node(condition=this_node.condition)

    def _build_branch(self, condition: LogicValue, statements: Iterable[PythonStatement]):
        context = self.context
        this_node = context.current_node

        phi = to_condition(test)
        psi = this_node.condition.join(phi)
        branch_node = self._new_node(condition=psi, origin=this_node)
        self._push_context(branch_node)
        for stmt in statement.then_branch.body:
            self.add_statement(stmt)
        self._go_back_to(context)

        rho = statement.then_branch.condition.negate()  # accumulator
        for branch in statement.elif_branches:
            phi = rho.join(branch.condition)
            psi = this_node.condition.join(phi)
            branch_node = self._new_node(condition=psi, origin=this_node)
            self._push_context(branch_node)
            for stmt in branch.body:
                self.add_statement(stmt)
            self._go_back_to(context)
            rho = rho.join(branch.condition.negate())


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
