# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, NewType, Optional, Set, Tuple, Union

from attrs import define, field, frozen

from haros.analysis.python.cfg import BasicControlFlowGraphBuilder
from haros.analysis.python.dataflow import DataScope, PythonType
from haros.analysis.python.logic import to_condition
from haros.metamodel.common import VariantData
from haros.metamodel.logic import FALSE, TRUE, LogicValue
from haros.parsing.python.ast import (
    PythonAst,
    PythonBinaryOperator,
    PythonBooleanLiteral,
    PythonExceptClause,
    PythonExpression,
    PythonForStatement,
    PythonFunctionCall,
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
ProgramNodeId = NewType('ProgramNodeId', int)
ExpressionNodeId = NewType('ExpressionNodeId', int)

ROOT_ID: Final[ControlNodeId] = ControlNodeId(0)

MAIN: Final[str] = '__main__'

###############################################################################
# Data Structures
###############################################################################


@frozen
class ControlJump:
    node: ProgramNodeId
    condition: LogicValue


@frozen
class ProgramNode:
    id: ProgramNodeId
    ast: PythonStatement
    condition: LogicValue = field(default=TRUE)
    control_in: Set[ControlJump] = field(factory=set)
    control_out: Set[ControlJump] = field(factory=set)


@frozen
class AnalysisExpression:
    value: Any
    type: PythonType

    @property
    def is_unknown(self) -> bool:
        return False

    @property
    def is_literal(self) -> bool:
        return False

    @property
    def is_operator(self) -> bool:
        return False

    @property
    def is_function_call(self) -> bool:
        return False

    @property
    def is_generator(self) -> bool:
        return False

    @property
    def is_lambda(self) -> bool:
        return False

    @property
    def is_star_expression(self) -> bool:
        return False

    @property
    def is_yield(self) -> bool:
        return False

    @property
    def is_await(self) -> bool:
        return False


@frozen
class ExpressionNode:
    id: ExpressionNodeId
    ast: PythonExpression
    values: VariantData[AnalysisExpression] = field(factory=VariantData)


@frozen
class FunctionCallNode(ExpressionNode):
    function: VariantData[str] = field(factory=VariantData)
    arguments: Tuple[ExpressionNodeId] = field(factory=tuple)


def eval_expression(expr: PythonExpression, data: DataScope) -> VariantData[AnalysisExpression]:
    # function that evaluates all possible values an expression can take
    # within a given context.
    variants = VariantData()
    return variants


def make_expression_node(expr: PythonExpression, data: DataScope) -> ExpressionNode:
    uid = ExpressionNodeId(0)
    variants = eval_expression(expr, data)
    return ExpressionNode(uid, expr, variants)




def program_to_nodes(module):
    statements = {}
    expressions = {}
    for statement in module.statements:
        uid = ProgramNodeId(len(statements))
        node = statement_to_node(statement, uid)
        statements[uid] = node
        # traverse expressions and add to dict
    return statements, expressions


def statement_to_node(statement: PythonStatement, uid: ProgramNodeId) -> ProgramNode:
    condition = TRUE
    control_in = set()
    control_out = set()
    return ProgramNode(
        uid,
        statement,
        condition=condition,
        control_in=control_in,
        control_out=control_out,
    )













###############################################################################
# Graph Builder
###############################################################################


@define
class ProgramGraphBuilder:
    #graph: ControlFlowGraph = field(factory=ControlFlowGraph.singleton)
    #current_id: ControlNodeId = ROOT_ID
    cfg: BasicControlFlowGraphBuilder = field(factory=BasicControlFlowGraphBuilder.from_scratch)
    data: DataScope = field(factory=DataScope.with_builtins)

    @classmethod
    def from_scratch(cls, name: str = MAIN, asynchronous: bool = False):
        cfg = BasicControlFlowGraphBuilder.from_scratch(name=name, asynchronous=asynchronous)
        return cls(cfg=cfg)

    def add_statement(self, statement: PythonStatement):
        if statement.is_while or statement.is_for:  # FIXME
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
            self.cfg.add_statement(statement)

            if statement.is_break:  # FIXME
                self.loop_context.break_from(this_node)
                self.cfg.start_dead_code()

            elif statement.is_continue:  # FIXME
                self.loop_context.continue_from(this_node)
                self.cfg.start_dead_code()

            elif statement.is_return or statement.is_raise:
                self.cfg.start_dead_code()

            elif statement.is_yield:
                self.cfg.asynchronous = True
                self.cfg.jump_to_new_node()

            elif statement.is_assert:
                phi = to_condition(statement.condition)  # FIXME
                # self.jump_to_new_node(phi.negate())  # terminal node
                self.cfg.jump_to_new_node(phi)

            elif statement.is_if:
                self.cfg.start_branching()
                self._build_branch(statement.condition, statement.body)
                for branch in statement.elif_branches:
                    self._build_branch(branch.condition, branch.body)
                self._build_branch(None, statement.else_branch)
                self.cfg.stop_branching()

            elif statement.is_match:
                pass  # TODO

            elif statement.is_with:
                # create and move to a new node
                self.cfg.jump_to_new_node()
                # recursively process the statements
                for stmt in statement.body:
                    self.add_statement(stmt)
                # move out of the with context
                self.cfg.jump_to_new_node()

            elif statement.is_try:
                # create and move to a new node
                future_node = self._new_node(this_node.condition)
                node = self.cfg.jump_to_new_node()
                # recursively process the statements
                for stmt in statement.body:
                    self.add_statement(stmt)
                    # keep track of the last node before dead code
                    if not self.cfg.current_node.is_unreachable:
                        node = self.cfg.current_node
                # FIXME add links to the except clauses
                for clause in statement.except_clauses:
                    self._build_except_clause(clause)
                # move back to last reachable node of try block
                self.cfg.switch_to(node)
                # move on to the else branch (if there is no dead code in try)
                if statement.has_else_branch:  # FIXME
                    self.cfg.jump_to_new_node()
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

    def _build_branch(
        self,
        test: Optional[PythonExpression],
        statements: Iterable[PythonStatement],
    ):
        if test is None:
            # create and move to a new branch
            psi = self.cfg.add_else_branch()
        else:
            # data flow analysis on the condition
            phi = self.logic_solver.to_condition(test)  # FIXME
            # create and move to a new branch
            psi = self.cfg.add_branch(phi)

        # recursively process the statements
        for statement in statements:
            self.add_statement(statement)

        # link to the node that comes after
        self.cfg.close_branch()

    def _build_except_clause(self, clause: PythonExceptClause):  # FIXME
        #node = self._new_node(this_node.condition)
        #self.current_id = node.id
        #for statement in clause.body:
        #    self.add_statement(statement)
        pass

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
    matches = find_name_in_graph(graph, full_name, data)
    return list(dict.fromkeys(matches))


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
    elif expression.is_function_call:
        references.extend(find_name_in_expression(expression.function, full_name, data))
        for arg in expression.arguments:
            references.extend(find_name_in_expression(arg.value, full_name, data))
    return references


def compare_qualified_names(full_name: str, import_base: str, local_name: str) -> bool:
    name = f'{import_base}.{local_name}' if import_base else local_name
    return full_name == name



###############################################################################
# Duplicate Code
###############################################################################


# If the target function has only one part (e.g. 'function'), it is a local function,
# and we have to look for variables whose definitions are a `def` AST statement.
# If the target function has multiple name parts (e.g. 'pkg.module.function'),
# it is an imported function, and we have to look for references to a variable
# with the proper function name (e.g. 'function'), and whose definitions point
# to an UnknownValue whose origin is the given module (e.g. 'pkg.module').


def find_qualified_function_call(graph: ControlFlowGraph, full_name: str) -> List[PythonFunctionCall]:
    data = DataScope.with_builtins()
    matches = find_function_call_in_graph(graph, full_name, data)
    return list(dict.fromkeys(matches))


def find_function_call_in_graph(
    graph: ControlFlowGraph,
    full_name: str,
    starting_data: DataScope,
) -> List[PythonFunctionCall]:
    calls = []
    branch_queue = [(graph.root_node, starting_data)]
    while branch_queue:
        node, data = branch_queue.pop(0)
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
                    calls.extend(find_function_call_in_expression(statement.value, full_name, data))
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
                    branch_queue.append((graph.nodes[node_id], data.duplicate()))
    for g in graph.nested_graphs.values():
        calls.extend(find_name_in_graph(g, full_name, data.duplicate()))
    return calls


def find_function_call_in_expression(
    expression: PythonExpression,
    full_name: str,
    data: DataScope,
) -> List[PythonFunctionCall]:
    calls = []
    if expression.is_function_call:
        if find_name_in_expression(expression.function, full_name, data):
            calls.append(expression)
        for arg in expression.arguments:
            calls.extend(find_function_call_in_expression(arg.value, full_name, data))
    # FIXME TODO operators etc.
    return calls
