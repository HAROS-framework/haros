# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Iterable, List, NewType, Optional, Tuple, Union

from attrs import define, field, frozen

from haros.parsing.python.ast import (
    PythonAst,
    PythonBooleanLiteral,
    PythonModule,
    PythonStatement,
)

###############################################################################
# Data Structures
###############################################################################


@define
class Scope:
    pass


ControlNodeId = NewType('ControlNodeId', Union[int, str])


@define
class ControlNode:
    id: ControlNodeId
    body: List[PythonStatement] = field(factory=list)
    condition: PythonExpression = field(factory=PythonBooleanLiteral.const_true)
    jumps: List[ControlNodeId] = field(factory=list)

    @property
    def is_empty(self) -> bool:
        return not self.body

    @property
    def is_leaf(self) -> bool:
        return not self.jumps


@define
class ControlFlowGraph:
    nodes: Dict[ControlNodeId, ControlNode]
    start: ControlNodeId

    @property
    def first_node(self) -> ControlNode:
        return self.nodes[self.start]

    @classmethod
    def new_graph(cls) -> 'ControlFlowGraph':
        start = ControlNodeId(0)
        node = ControlNode(start)
        return cls({start: node}, start)


###############################################################################
# Graph Builder
###############################################################################


@define
class ControlFlowGraphBuilder:
    cfg: ControlFlowGraph = field(factory=ControlFlowGraph.new_graph)

    def build(self, statements: Iterable[PythonStatement]) -> ControlFlowGraph:
        for statement in statements:
            self.cfg.add_statement(statement)
        return self.cfg

    def _add_node(self, condition: Optional[PythonExpression]):
        if condition is None:
            condition = PythonBooleanLiteral.const_true()
        node = ControlNodeBuilder(id=len(self.nodes), condition=condition)
        self.nodes.append(node)

    def _add_statement(self, statement: PythonStatement):
        if statement.is_expression_statement:
            self.current_node.body.append(statement)
        elif statement.is_assignment:
            self.current_node.body.append(statement)
        elif statement.is_delete:
            self.current_node.body.append(statement)
        elif statement.is_pass:
            self.current_node.body.append(statement)
        elif statement.is_break:
            return False
        elif statement.is_continue:
            return False
        elif statement.is_return:
            self.current_node.body.append(statement)
            self.
        elif statement.is_raise:
            return False
        elif statement.is_yield:
            return
        elif statement.is_import:
            return False
        elif statement.is_global:
            return False
        elif statement.is_nonlocal:
            return False
        elif statement.is_assert:
            return False
        elif statement.is_if:
            return False
        elif statement.is_while:
            return False
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
        else:
            raise TypeError(f'unknown statement type: {statement!r}')


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
