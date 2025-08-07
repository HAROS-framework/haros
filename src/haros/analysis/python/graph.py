# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import (
    Any,
    Final,
    NewType,
    Optional,
)

from collections.abc import Callable, Iterable, Mapping, Sequence
import logging
from types import SimpleNamespace

from attrs import define, field, frozen

from haros.analysis.python.cfg import BasicControlFlowGraphBuilder, ControlFlowGraph, ControlNodeId
from haros.analysis.python.dataflow import DataScope, tracked
from haros.analysis.python.logic import to_condition
from haros.errors import AnalysisError
from haros.metamodel.common import VariantData
from haros.metamodel.logic import TRUE, LogicValue
from haros.metamodel.result import Result, TypeToken
from haros.parsing.python.ast import (
    PythonAst,
    PythonBinaryOperator,
    PythonExceptClause,
    PythonExpression,
    PythonFunctionCall,
    PythonFunctionDefStatement,
    PythonModule,
    PythonStatement,
    PythonTupleLiteral,
)
from haros.parsing.python.ast.expressions import PythonAssignmentExpression
from haros.parsing.python.ast.statements import PythonExpressionStatement, PythonWithStatement

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

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

    def pretty(self) -> str:
        return str(self)

    def __str__(self) -> str:
        return f'{self.node} ({self.condition})'


@frozen
class ProgramNode:
    id: ProgramNodeId
    ast: PythonStatement
    condition: LogicValue = field(default=TRUE)
    incoming: Mapping[ProgramNodeId, LogicValue] = field(factory=dict, eq=False, hash=False)
    outgoing: Mapping[ProgramNodeId, LogicValue] = field(factory=dict, eq=False, hash=False)


@frozen
class AnalysisExpression:
    value: Any
    type: TypeToken

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
    arguments: Sequence[ExpressionNodeId] = field(factory=tuple)


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


@frozen
class ProgramGraph:
    name: str
    root_id: ProgramNodeId = field()
    nodes: Mapping[ProgramNodeId, ProgramNode] = field(factory=dict)
    asynchronous: bool = False

    @root_id.validator
    def _check_root_id(self, attribute, value: ProgramNodeId):
        if value not in self.nodes:
            raise ValueError(f'unknown node id: {value!r}')

    @property
    def root_node(self) -> ProgramNode:
        return self.nodes[self.root_id]


###############################################################################
# Graph Builder
###############################################################################


@define
class ProgramGraphBuilder:
    # graph: ControlFlowGraph = field(factory=ControlFlowGraph.singleton)
    # current_id: ControlNodeId = ROOT_ID
    name: str = '__main__'
    cfg: BasicControlFlowGraphBuilder = field(factory=BasicControlFlowGraphBuilder.from_scratch)
    data: DataScope = field(factory=DataScope.with_builtins)
    nested_graphs: Mapping[str, PythonStatement] = field(factory=dict)
    _pid: int = 0

    @classmethod
    def from_scratch(cls, name: str = MAIN, asynchronous: bool = False):
        cfg = BasicControlFlowGraphBuilder.from_scratch(name=name, asynchronous=asynchronous)
        return cls(name=name, cfg=cfg)

    def add_imported_symbol(self, name: str, module: str, value: Any):
        self.data.add_imported_symbol(name, module, value)

    def add_statement(self, statement: PythonStatement):
        pid = ProgramNodeId(self._pid)
        self._pid += 1
        node = ProgramNode(pid, statement)
        if statement.is_while or statement.is_for:  # FIXME
            if statement.is_while:
                test = statement.condition
            else:
                it = statement.iterator
                variables = PythonTupleLiteral(it.variables)
                test = PythonBinaryOperator('in', variables, it.iterable)
            self.cfg.start_looping()
            # add statement to the loop's guard node
            self.cfg.add_statement(statement)
            self._build_loop(test, statement.body)
            self._build_loop_else(statement.else_branch)
            self.cfg.stop_looping()

        else:
            self.cfg.add_statement(statement)

            if statement.is_import:
                self.data.add_import(statement)

            elif statement.is_assignment:
                self.data.add_assignment(statement)

            elif statement.is_break:  # FIXME
                # FIXME # self.loop_context.break_from(this_node)
                self.cfg.start_dead_code()

            elif statement.is_continue:  # FIXME
                # FIXME # self.loop_context.continue_from(this_node)
                self.cfg.start_dead_code()

            elif statement.is_raise:  # FIXME
                self.cfg.start_dead_code()

            elif statement.is_return:
                self.data.add_return_value(statement.value)
                self.cfg.start_dead_code()

            elif statement.is_yield:
                self.cfg.asynchronous = True
                self.cfg.jump_to_new_node()

            elif statement.is_assert:
                # data flow analysis on the condition
                phi = self.data.evaluate_condition(statement.condition)
                if phi.is_false:
                    self.cfg.start_dead_code()
                else:
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
                assert isinstance(statement, PythonWithStatement)
                # create and move to a new node
                self.cfg.jump_to_new_node()
                # define new variables from context managers
                for context in statement.managers:
                    if context.alias:
                        value = self.data.value_from_expression(context.manager)
                        self.data.set(context.alias, value, ast=statement)
                # recursively process the statements
                for stmt in statement.body:
                    self.add_statement(stmt)
                # move out of the with context
                self.cfg.jump_to_new_node()

            elif statement.is_try:
                # create and move to a new node
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
                # FIXME jump also to finally block from except clauses
                if statement.has_finally_block:  # FIXME
                    self.cfg.jump_to_new_node()
                    for stmt in statement.finally_block:
                        self.add_statement(stmt)
                # link to the node that comes after
                self.cfg.jump_to_new_node()

            elif statement.is_function_def or statement.is_class_def:
                if statement.is_class_def:
                    # asynchronous = False
                    self.data.add_class_def(statement)
                else:
                    # asynchronous = statement.asynchronous
                    cb = self._function_interpreter(statement)
                    self.data.add_function_def(statement, fun=cb)
                self.nested_graphs[statement.name] = statement

            elif statement.is_expression_statement:
                assert isinstance(statement, PythonExpressionStatement)
                # handle method calls and side effects
                value = self.data.value_from_expression(statement.expression)
                if statement.expression.is_assignment:
                    assert isinstance(statement.expression, PythonAssignmentExpression)
                    self.data.set(statement.expression.name, value, ast=statement)

    def clean_up(self):
        self.cfg.clean_up()

    def build(self):  # FIXME
        cfg = self.cfg.build()
        # cid = 0
        # for control_node in cfg.nodes.values():
        #     incoming = dict
        #     for statement in control_node.body:
        #         # id: ProgramNodeId
        #         # ast: PythonStatement
        #         # condition: LogicValue = field(default=TRUE)
        #         # incoming: dict[ProgramNodeId, LogicValue] = field(
        #         #     factory=dict, eq=False, hash=False)
        #         # outgoing: dict[ProgramNodeId, LogicValue] = field(
        #         #     factory=dict, eq=False, hash=False)
        #         node = ProgramNode(
        #             ProgramNodeId(cid),
        #             statement,
        #             condition=control_node.condition,
        #             incoming
        #             outgoing
        #         )
        #         cid += 1
        return cfg, self.data
        # return ProgramGraph(g.name)

    def subgraph_builder(self, name: str) -> 'ProgramGraphBuilder':
        # FIXME make this an external function to operate on 'built' graphs
        try:
            statement = self.nested_graphs[name]
        except KeyError:
            raise AnalysisError(f'subgraph "{name}" does not exist')
        assert statement.is_function_def or statement.is_class_def, repr(statement)
        asynchronous = False if not statement.is_function_def else statement.asynchronous
        builder = ProgramGraphBuilder.from_scratch(name=name, asynchronous=asynchronous)
        builder.data = self.data.duplicate()
        try:
            for stmt in statement.body:
                builder.add_statement(stmt)
        except Exception:
            logger.exception(f'error processing statement of "{name}"')
            logger.error(f'add_statement({stmt!r})')
        builder.clean_up()
        return builder

    def _build_branch(
        self,
        test: Optional[PythonExpression],
        body: Iterable[PythonStatement],
    ):
        if test is None:
            # create and move to a new branch
            psi = self.cfg.add_else_branch().condition
        else:
            # data flow analysis on the condition
            phi = self.data.evaluate_condition(test)
            # create and move to a new branch
            psi = self.cfg.add_branch(phi).condition

        # update the logic condition for data flow analysis
        self.data.push_condition(psi)

        # recursively process the statements
        for statement in body:
            self.add_statement(statement)

        # link to the node that comes after
        self.cfg.close_branch()
        self.data.pop_condition()

    def _build_except_clause(self, clause: PythonExceptClause):  # FIXME
        # node = self._new_node(this_node.condition)
        # self.current_id = node.id
        # for statement in clause.body:
        #    self.add_statement(statement)
        pass

    def _build_loop(self, test: PythonExpression, body: Iterable[PythonStatement]):
        # very similar to `_build_branch`
        # create and move to a new branch
        phi = to_condition(test)  # FIXME
        self.cfg.jump_to_new_node(phi=phi)

        # recursively process the statements
        for statement in body:
            self.add_statement(statement)

        # link back to the loop guard node
        self.cfg.close_loop()

    def _build_loop_else(self, body: Iterable[PythonStatement]):
        if not body:
            return

        # create and move to a new branch
        self.cfg.add_loop_else_branch()

        # recursively process the statements
        for statement in body:
            self.add_statement(statement)

    def _function_interpreter(self, function: PythonFunctionDefStatement) -> Callable:
        # returns a wrapper that, given the proper arguments, interprets the function
        # retains the current data scope (where the def appears) to act as globals
        global_vars = self.data

        # FIXME what to do if `len(function.decorators) > 0`?
        def cb(*args: Result[Any], **kwargs: Result[Any]) -> Result[Any]:
            # args and kwargs should be Result
            builder = ProgramGraphBuilder.from_scratch(
                name=function.name,
                asynchronous=function.asynchronous,
            )
            builder.data = global_vars.duplicate()
            # assign arguments to parameters
            stack = list(reversed(args))
            mapping = dict(kwargs)
            if function.parameters:
                for param in function.parameters:
                    # FIXME check parameter order (positional, standard, etc)?
                    if param.is_positional:
                        # FIXME check for errors here (num of args, etc)
                        builder.data.set(param.name, stack.pop())
                    elif param.is_standard:
                        if stack:
                            builder.data.set(param.name, stack.pop())
                        else:
                            value = mapping.get(param.name)
                            if value is None:
                                if param.default_value is None:
                                    raise ValueError(f'expected argument for {param.name}')
                                value = global_vars.value_from_expression(param.default_value)
                            else:
                                del mapping[param.name]
                            builder.data.set(param.name, value)
                    elif param.is_variadic_list:
                        builder.data.set(param.name, Result.of_list(stack))  # FIXME
                    elif param.is_keyword:
                        value = mapping.get(param.name)
                        if value is None:
                            value = global_vars.value_from_expression(param.default_value)
                        else:
                            del mapping[param.name]
                        builder.data.set(param.name, value)
                    elif param.is_variadic_keywords:
                        builder.data.set(param.name, Result.of_dict(mapping))  # FIXME
            for statement in function.body:
                builder.add_statement(statement)
            # builder.clean_up()
            # builder.build()
            result = builder.data.return_values
            if result.has_values and result.is_deterministic:
                return result.get()
            return Result.unknown_value(source=tracked(function))

        # cb.name = function.name
        # cb.module = '__main__'
        return cb


###############################################################################
# Interface
###############################################################################


def from_ast(ast: PythonAst, symbols: Optional[Mapping[str, Any]] = None) -> Any:
    if ast.is_module:
        return from_module(ast, symbols=symbols)
    if not ast.is_statement:
        raise TypeError(f'expected a statement, got {ast!r}')
    raise TypeError(f'unexpected tree node: {ast!r}')


def from_module(module: PythonModule, symbols: Optional[Mapping[str, Any]] = None) -> Any:
    builder = ProgramGraphBuilder.from_scratch(name=module.name)
    if symbols:
        for key, value in symbols.items():
            name, import_base = _split_names(key)
            assert bool(import_base), f'expected non-empty import base: {key}'
            if isinstance(value, SimpleNamespace):
                # full module
                if not name:
                    name = import_base
                    import_base = ''
            else:
                assert bool(name), f'expected non-empty name: {key}'
            builder.add_imported_symbol(name, import_base, value)
    for statement in module.statements:
        builder.add_statement(statement)
    builder.clean_up()
    # return builder.build()
    return builder


def _split_names(full_name: str) -> tuple[str, str]:
    initial = full_name
    prefix = ''
    if full_name.startswith('..'):
        prefix = '..'
        initial = full_name[2:]
    elif full_name.startswith('.'):
        prefix = '.'
        initial = full_name[1:]
    parts = initial.rsplit('.', maxsplit=1)
    if len(parts) > 1:
        name = parts[-1]
        module = f'{prefix}{parts[0]}'
    elif initial.startswith('__') and initial.endswith('__'):
        # magic variables
        name = initial
        module = ''
    else:
        name = ''
        module = f'{prefix}{initial}'
    return name, module


def find_qualified_name(graph: ControlFlowGraph, full_name: str) -> list[PythonAst]:
    data = DataScope.with_builtins()
    matches = find_name_in_graph(graph, full_name, data)
    return list(dict.fromkeys(matches))


def find_name_in_graph(
    graph: ControlFlowGraph,
    full_name: str,
    data: DataScope,
) -> list[PythonExpression]:
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
) -> list[PythonExpression]:
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


def find_qualified_function_call(
    graph: ControlFlowGraph, full_name: str
) -> list[PythonFunctionCall]:
    data = DataScope.with_builtins()
    matches = find_function_call_in_graph(graph, full_name, data)
    return list(dict.fromkeys(matches))


def find_function_call_in_graph(
    graph: ControlFlowGraph,
    full_name: str,
    starting_data: DataScope,
) -> list[PythonFunctionCall]:
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
) -> list[PythonFunctionCall]:
    calls = []
    if expression.is_function_call:
        if find_name_in_expression(expression.function, full_name, data):
            calls.append(expression)
        for arg in expression.arguments:
            calls.extend(find_function_call_in_expression(arg.value, full_name, data))
    # FIXME TODO operators etc.
    return calls
