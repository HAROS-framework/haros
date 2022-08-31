# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Iterable, List, NewType, Optional, Set, Tuple, Union

from attrs import define, field, frozen

from haros.parsing.python.ast import (
    PythonAst,
    PythonBooleanLiteral,
    PythonForStatement,
    PythonModule,
    PythonStatement,
    PythonWhileStatement,
    PythonTryStatement,
    negate,
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
    condition: PythonExpression = field(factory=PythonBooleanLiteral.const_true)
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


def _cfg_factory() -> ControlFlowGraph:
    uid = ControlNodeId(0)
    node = ControlNode(uid)
    return ControlFlowGraph(nodes=[node])


@define
class ControlFlowGraphBuilder:
    cfg: ControlFlowGraph = field(factory=_cfg_factory)

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
            self._new_node(origin=this_node)

        elif statement.is_assert:
            # terminal node
            self._new_node(negate(statement.condition), origin=this_node)
            # node that continues the flow
            self._new_node(statement.condition, origin=this_node)

        elif statement.is_if:
            return False

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
        condition: Optional[PythonExpression] = None,
        *,
        origin: Optional[ControlNode] = None,
    ) -> ControlNode:
        uid = ControlNodeId(len(self.cfg.nodes))
        if condition is None:
            condition = PythonBooleanLiteral.const_true()
        node = ControlNode(uid, condition=condition)
        self.cfg.nodes.append(node)
        if origin is not None:
            origin.jump_to(node)
        return node

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
