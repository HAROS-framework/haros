# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Dict, Final, Iterable, List, NewType, Optional, Set, Tuple, Union

from attrs import define, field, frozen

from haros.analysis.python.dataflow import DataScope
from haros.analysis.python.logic import to_condition
from haros.metamodel.logic import FALSE, TRUE, LogicValue
from haros.parsing.python.ast import (
    PythonAst,
    PythonBinaryOperator,
    PythonBooleanLiteral,
    PythonExpression,
    PythonForStatement,
    PythonModule,
    PythonStatement,
    PythonWhileStatement,
    PythonTryStatement,
    PythonTupleLiteral,
)

###############################################################################
# Constants
###############################################################################

ControlNodeId = NewType('ControlNodeId', int)

ROOT_ID: Final[ControlNodeId] = ControlNodeId(0)

MAIN: Final[str] = '__main__'

###############################################################################
# Data Structures
###############################################################################


@define
class ControlNode:
    id: ControlNodeId
    body: List[PythonStatement] = field(factory=list)
    condition: LogicValue = field(default=TRUE)
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

    def pretty(self) -> str:
        lines = [
            f'# Node {self.id}',
            f'condition:\n  {self.condition}',
            f'incoming:\n  {list(self.incoming)}',
            f'outgoing:\n  {list(self.outgoing)}',
        ]
        if self.body:
            lines.append('body:')
            for statement in self.body:
                lines.append(statement.pretty(indent=1, step=2))
        return '\n'.join(lines)


@define
class ControlFlowGraph:
    name: str
    root_id: ControlNodeId = field()
    nodes: Dict[ControlNodeId, ControlNode] = field(factory=dict)
    asynchronous: bool = False
    nested_graphs: Dict[str, 'ControlFlowGraph'] = field(factory=dict)

    @root_id.validator
    def _check_root_id(self, attribute, value: ControlNodeId):
        if value not in self.nodes:
            raise ValueError(f'unknown node id: {value!r}')

    @property
    def root_node(self) -> ControlNode:
        return self.nodes[self.root_id]

    @classmethod
    def singleton(cls, name: str = MAIN, asynchronous: bool = False) -> 'ControlFlowGraph':
        nodes = {ROOT_ID: ControlNode(ROOT_ID)}
        return cls(name, ROOT_ID, nodes=nodes, asynchronous=asynchronous)

    def pretty(self) -> str:
        header = f'# async {self.name}' if self.asynchronous else f'# {self.name}'
        root = f'root: {self.root_id}'
        lines = [header, root, 'graph:']
        lines.append(f'  {self.root_id} -> {list(self.root_node.outgoing)}')
        unreachable = []
        for uid, node in self.nodes.items():
            if uid == self.root_id:
                continue
            if node.incoming:
                lines.append(f'  {uid} -> {list(node.outgoing)}')
            else:
                unreachable.append(f'  {uid} -> {list(node.outgoing)}')
        if unreachable:
            lines.append('dead code:')
            lines.extend(unreachable)
        return '\n'.join(lines)


###############################################################################
# Graph Builder
###############################################################################


class MalformedProgramError(Exception):
    @classmethod
    def not_looping(cls) -> 'MalformedProgramError':
        return cls('found a loop jump statement outside of a loop')

    @classmethod
    def not_branching(cls) -> 'MalformedProgramError':
        return cls('found a branching statement outside of a conditional')

    @classmethod
    def if_after_else(cls) -> 'MalformedProgramError':
        return cls('found a branching statement after an else statement')


@define
class LoopingContext:
    guard_node: ControlNode
    future_node: ControlNode
    break_condition: LogicValue = field(init=False, default=FALSE)

    def break_from(self, node: ControlNode):
        node.jump_to(self.future_node)
        phi = node.condition.assume(self.guard_node.condition)
        self.break_condition = self.break_condition.disjoin(phi)

    def continue_from(self, node: ControlNode):
        node.jump_to(self.guard_node)


@define
class BranchingContext:
    guard_node: ControlNode
    future_node: ControlNode
    condition: LogicValue = field(init=False, default=FALSE)
    previous: LogicValue = field(init=False, default=TRUE)

    def add_branch(self, test: PythonExpression) -> LogicValue:
        return self._add_branch(to_condition(test))

    def add_else_branch(self) -> LogicValue:
        return self._add_branch(TRUE)

    def _add_branch(self, phi: LogicValue) -> LogicValue:
        if self.condition.is_true:
            raise MalformedProgramError.if_after_else()
        # negate the condition of the current (now previous) branch
        psi = self.condition.negate()
        # join with conditions from past branches
        self.previous = self.previous.join(psi)
        # condition evaluated at the new branch
        self.condition = phi
        # return full condition
        return self.previous.join(self.condition)


# ProgramGraphBuilder is ready to `add_statement` after initialisation.
# It keeps a stack of `LoopingContext` and a stack of `BranchingContext`
# which it uses to build and link nodes.


@define
class ProgramGraphBuilder:
    graph: ControlFlowGraph = field(factory=ControlFlowGraph.singleton)
    current_id: ControlNodeId = ROOT_ID
    _loop_stack: List[LoopingContext] = field(factory=list)
    _branch_stack: List[BranchingContext] = field(factory=list)

    @classmethod
    def from_scratch(cls, name: str = MAIN):
        graph = ControlFlowGraph.singleton(name=name)
        return cls(graph=graph, current_id=graph.root_id)

    @property
    def current_node(self) -> ControlNode:
        return self.graph.nodes[self.current_id]

    @current_node.setter
    def current_node(self, node: ControlNode):
        self.current_id = node.id

    @property
    def loop_context(self) -> LoopingContext:
        if not self._loop_stack:
            raise MalformedProgramError.not_looping()
        return self._loop_stack[-1]

    @property
    def branch_context(self) -> BranchingContext:
        if not self._branch_stack:
            raise MalformedProgramError.not_branching()
        return self._branch_stack[-1]

    def add_statement(self, statement: PythonStatement):
        this_node = self.current_node

        if statement.is_while or statement.is_for:
            if statement.is_while:
                phi = statement.condition
            else:
                it = statement.iterator
                variables = PythonTupleLiteral(it.variables)
                phi = PythonBinaryOperator('in', variables, it.iterable)
            guard_node = self._follow_up_node(switch=True)
            self._start_looping(guard_node)
            self._build_loop(phi, statement.body)
            self._build_loop_else(statement.else_branch)
            self._stop_looping()

        else:
            this_node.append(statement)

            if statement.is_break:
                self.loop_context.break_from(this_node)
                self.start_dead_code()

            elif statement.is_continue:
                self.loop_context.continue_from(this_node)
                self.start_dead_code()

            elif statement.is_return or statement.is_raise:
                self.start_dead_code()

            elif statement.is_yield:
                self.graph.asynchronous = True
                self._follow_up_node(switch=True)

            elif statement.is_assert:
                phi = to_condition(statement.condition)
                self._follow_up_node(phi.negate())  # terminal node
                self._follow_up_node(phi, switch=True)

            elif statement.is_if:
                self._start_branching(this_node)
                self._build_branch(statement.condition, statement.body)
                for branch in statement.elif_branches:
                    self._build_branch(branch.condition, branch.body)
                self._build_branch(None, statement.else_branch)
                self._stop_branching()

            elif statement.is_match:
                pass  # TODO

            elif statement.is_with:
                # create and move to a new node
                future_node = self._new_node(this_node.condition)
                self._follow_up_node(switch=True)
                # recursively process the statements
                for stmt in statement.body:
                    self.add_statement(stmt)
                # link to the node that comes after
                self.current_node.jump_to(future_node)
                self.current_id = future_node.id

            elif statement.is_try:
                # create and move to a new node
                future_node = self._new_node(this_node.condition)
                self._follow_up_node(switch=True)
                # recursively process the statements
                for stmt in statement.body:
                    self.add_statement(stmt)
                # FIXME add links to the except clauses
                for clause in statement.except_clauses:
                    node = self._new_node(this_node.condition)
                    self.current_id = node.id
                    for stmt in clause.body:
                        self.add_statement(stmt)
                self.current_id = this_node.id
                # move on to the else branch
                if statement.has_else_branch:
                    self._follow_up_node(switch=True)
                    for stmt in statement.else_branch:
                        self.add_statement(stmt)
                # move on to the finally block
                if statement.has_finally_block:
                    self._follow_up_node(switch=True)
                    for stmt in statement.finally_block:
                        self.add_statement(stmt)
                # link to the node that comes after
                self.current_node.jump_to(future_node)
                self.current_id = future_node.id

            elif statement.is_function_def or statement.is_class_def:
                builder = ProgramGraphBuilder.from_scratch(name=statement.name)
                if statement.is_function_def:
                    builder.graph.asynchronous = statement.asynchronous
                for stmt in statement.body:
                    builder.add_statement(stmt)
                builder.clean_up()
                self.graph.nested_graphs[statement.name] = builder.graph

    def start_dead_code(self):
        # push a new node that propagates the current condition,
        # but has no incoming links from other nodes
        self._new_node(phi=self.current_node.condition, switch=True)

    def clean_up(self):
        # removes useless nodes from the graph
        for node in list(self.graph.nodes.values()):
            if node.id == self.graph.root_id or len(node.body) > 0:
                continue
            if node.is_unreachable:
                del self.graph.nodes[node.id]

    def _new_node(
        self,
        phi: LogicValue,
        *,
        origin: Optional[ControlNode] = None,
        switch: bool = False,
    ) -> ControlNode:
        uid = ControlNodeId(len(self.graph.nodes))
        node = ControlNode(uid, condition=phi)
        self.graph.nodes[uid] = node
        if origin is not None:
            origin.jump_to(node)
        if switch:
            self.current_id = node.id
        return node

    def _follow_up_node(self, phi: LogicValue = TRUE, *, switch: bool = False) -> ControlNode:
        this_node = self.current_node
        phi = this_node.condition.join(phi)
        return self._new_node(phi, origin=this_node, switch=switch)

    def _start_branching(self, guard_node: ControlNode) -> BranchingContext:
        future_node = self._new_node(guard_node.condition)
        context = BranchingContext(guard_node, future_node)
        self._branch_stack.append(context)
        return context

    def _stop_branching(self):
        if not self._branch_stack:
            raise MalformedProgramError.not_branching()
        context = self._branch_stack.pop()
        self.current_id = context.future_node.id

    def _build_branch(
        self,
        test: Optional[PythonExpression],
        statements: Iterable[PythonStatement],
    ):
        # create and move to a new branch
        conditional = self.branch_context
        self.current_id = conditional.guard_node.id
        if test is None:
            phi = conditional.add_else_branch()
        else:
            phi = conditional.add_branch(test)
        self._follow_up_node(phi, switch=True)

        # recursively process the statements
        for statement in statements:
            self.add_statement(statement)

        # link to the node that comes after
        self.current_node.jump_to(conditional.future_node)

    def _start_looping(self, guard_node: ControlNode) -> LoopingContext:
        future_node = self._new_node(guard_node.condition)
        context = LoopingContext(guard_node, future_node)
        self._loop_stack.append(context)
        return context

    def _stop_looping(self):
        if not self._loop_stack:
            raise MalformedProgramError.not_looping()
        context = self._loop_stack.pop()
        self.current_id = context.future_node.id

    def _build_loop(self, test: PythonExpression, statements: Iterable[PythonStatement]):
        # very similar to `_build_branch`
        # create and move to a new branch
        phi = to_condition(test)
        self._follow_up_node(phi, switch=True)

        # recursively process the statements
        for statement in statements:
            self.add_statement(statement)

        # link back to the loop guard node
        self.current_node.jump_to(self.loop_context.guard_node)

    def _build_loop_else(self, statements: Iterable[PythonStatement]):
        loop = self.loop_context
        self.current_id = loop.guard_node.id

        # create and move to a new branch
        phi = loop.break_condition.negate()
        self._follow_up_node(phi, switch=True)

        # recursively process the statements
        for statement in statements:
            self.add_statement(statement)

        # link to the node that comes after
        self.current_node.jump_to(loop.future_node)


###############################################################################
# Interface
###############################################################################


def from_ast(ast: PythonAst) -> ControlFlowGraph:
    if ast.is_module:
        return from_module(ast)
    if not ast.is_statement:
        raise TypeError(f'expected a statement, got {ast!r}')
    raise TypeError(f'unexpected tree node: {ast!r}')


def from_module(module: PythonModule) -> ControlFlowGraph:
    builder = ProgramGraphBuilder.from_scratch(name=module.name)
    for statement in module.statements:
        builder.add_statement(statement)
    builder.clean_up()
    return builder.graph


def find_qualified_name(graph: ControlFlowGraph, full_name: str) -> List[PythonAst]:
    data = DataScope.with_builtins()
    return find_name_in_graph(graph, full_name, data)


def find_name_in_graph(
    graph: ControlFlowGraph,
    full_name: str,
    data: DataScope,
) -> List[PythonExpression]:
    references = []
    branch_queue = [graph.root_node]
    while branch_queue:
        node = branch_queue.pop(0)
        while node is not None:
            # set the scope's condition
            data.condition = node.condition
            # process the node's statements
            for statement in node.body:
                if statement.is_import:
                    data.add_import(statement)
                elif statement.is_function_def:
                    data.add_function_def(statement)
                elif statement.is_class_def:
                    data.add_class_def(statement)
                elif statement.is_assignment:
                    data.add_assignment(statement)
                elif statement.is_return:
                    references.extend(find_name_in_expression(statement.value, full_name, data))
                # FIXME TODO
            # get the next node
            if not node.outgoing:
                node = None
            else:
                # follow the first node, queue up the rest
                outgoing = list(node.outgoing)
                node_id = outgoing[0]
                node = graph.nodes[node_id]
                for node_id in outgoing[1:]:
                    branch_queue.append(graph.nodes[node_id])
    for g in graph.nested_graphs.values():
        references.extend(find_name_in_graph(g, full_name, data.duplicate()))
    return references


def find_name_in_expression(
    expression: PythonExpression,
    full_name: str,
    data: DataScope,
) -> List[PythonExpression]:
    references = []
    if expression.is_literal:
        pass  # FIXME TODO
    elif expression.is_reference:
        if expression.object is not None:
            return find_name_in_expression(expression.object, full_name, data)
        name = expression.name
        var = data.get(name)
        for option in var.possible_values():
            definition = option.value
            if compare_qualified_names(full_name, definition.import_base, name):
                references.append(expression)
                break  # one is enough, unless we attach a condition
    return references


def compare_qualified_names(full_name: str, import_base: str, local_name: str) -> bool:
    name = f'{import_base}.{local_name}' if import_base else local_name
    return full_name == name
