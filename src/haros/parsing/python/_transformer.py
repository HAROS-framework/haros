# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Iterable, Optional, Tuple, Union

import re

from attrs import frozen
from lark import Token, Transformer, v_args

from haros.parsing.python.ast import (
    PythonArgument,
    PythonAssertStatement,
    PythonAssignmentExpression,
    PythonAssignmentStatement,
    PythonAst,
    PythonAstNodeId,
    PythonAstNodeMetadata,
    PythonAwaitExpression,
    PythonBinaryOperator,
    PythonBooleanLiteral,
    PythonBreakStatement,
    PythonCasePattern,
    PythonCaseStatement,
    PythonClassCasePattern,
    PythonClassDefStatement,
    PythonConditionalBlock,
    PythonConditionalExpression,
    PythonContextManager,
    PythonContinueStatement,
    PythonDecorator,
    PythonDeleteStatement,
    PythonDictComprehension,
    PythonDictEntry,
    PythonDictLiteral,
    PythonExceptClause,
    PythonExpression,
    PythonExpressionStatement,
    PythonForStatement,
    PythonFunctionCall,
    PythonFunctionDefStatement,
    PythonFunctionParameter,
    PythonGenerator,
    PythonIfStatement,
    PythonImportBase,
    PythonImportStatement,
    PythonImportedName,
    PythonItemAccess,
    PythonIterator,
    PythonKeyAccess,
    PythonKeyCasePattern,
    PythonKeyValuePair,
    PythonLambdaExpression,
    PythonListComprehension,
    PythonListLiteral,
    PythonMappingCasePattern,
    PythonMatchStatement,
    PythonModule,
    PythonNamedCasePattern,
    PythonNoneLiteral,
    PythonNumberLiteral,
    PythonOrCasePattern,
    PythonPassStatement,
    PythonRaiseStatement,
    PythonReference,
    PythonReturnStatement,
    PythonScopeStatement,
    PythonSequenceCasePattern,
    PythonSetComprehension,
    PythonSetLiteral,
    PythonSimpleArgument,
    PythonSimpleCasePattern,
    PythonSlice,
    PythonSpecialArgument,
    PythonStarExpression,
    PythonStatement,
    PythonStringLiteral,
    PythonSubscript,
    PythonTryStatement,
    PythonTupleComprehension,
    PythonTupleLiteral,
    PythonUnaryOperator,
    PythonWhileStatement,
    PythonWildcardCasePattern,
    PythonWithStatement,
    PythonYieldExpression,
)

###############################################################################
# Transformer
###############################################################################

PythonDefinition = Union[PythonFunctionDefStatement, PythonClassDefStatement]

MaybeParams = Optional[Union[PythonFunctionParameter, Tuple[PythonFunctionParameter]]]

SomeExpressions = Union[PythonExpression, Tuple[PythonExpression]]
MaybeExpressions = Optional[Tuple[PythonExpression]]

SomeStatements = Union[PythonStatement, Tuple[PythonStatement]]
MaybeStatements = Optional[Tuple[PythonStatement]]

OperatorSequence = Iterable[Union[str, PythonExpression]]


@frozen
class PythonAliasName:
    name: str
    alias: Optional[str]
    # meta
    line: int = 0
    column: int = 0


class ToAst(Transformer):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node_id_counter = 0

    def _node_metadata(self, line: int = 0, column: int = 0) -> PythonAstNodeMetadata:
        uid = PythonAstNodeId(self.node_id_counter)
        self.node_id_counter += 1
        return PythonAstNodeMetadata(uid, line=line, column=column)

    def _new_node(self, cls, *args, line: int = 0, column: int = 0, **kwargs) -> PythonAst:
        meta = self._node_metadata(line=line, column=column)
        return cls(meta, *args, **kwargs)

    # Top Level Rules ######################################

    def file_input(self, children: Iterable[SomeStatements]) -> PythonModule:
        statements = self._flatten_statements(children)
        return self._new_node(PythonModule, statements)

    def suite(self, children: Iterable[SomeStatements]) -> Tuple[PythonStatement]:
        return self._flatten_statements(children)

    def _flatten_statements(self, items: Iterable[SomeStatements]) -> Tuple[PythonStatement]:
        assert len(items) >= 1, f'_flatten_statements: {items}'
        statements = []
        for statement in items:
            if isinstance(statement, tuple):
                statements.extend(statement)
            else:
                assert isinstance(statement, PythonStatement), f'_flatten_statements: {children}'
                statements.append(statement)
        return tuple(statements)

    def _tuple_to_expr(self, value: SomeExpressions, singles: bool = True) -> PythonExpression:
        if isinstance(value, tuple):
            if not singles and len(value) == 1:
                assert isinstance(value[0], PythonExpression), f'_tuple_to_expr: {value!r}'
                return value[0]
            return self._new_node(
                PythonTupleLiteral,
                value,
                line=value[0].line,
                column=value[0].column,
            )
        assert isinstance(value, PythonExpression), f'_tuple_to_expr: {value!r}'
        return value

    # Helper Nodes #########################################

    @v_args(inline=True)
    def key_value(self, key: PythonExpression, value: PythonExpression) -> PythonKeyValuePair:
        return self._new_node(PythonKeyValuePair, key, value)

    @v_args(inline=True)
    def comprehension(
        self,
        result: PythonAst,
        iterators: Tuple[PythonIterator],
        test: Optional[PythonExpression],
    ) -> PythonGenerator:
        return self._new_node(PythonGenerator, result, iterators, test=test)

    def comp_fors(self, children: Iterable[PythonIterator]) -> Tuple[PythonIterator]:
        return tuple(children)

    @v_args(inline=True)
    def comp_for(
        self,
        maybe_async: Optional[Token],
        variables: Tuple[PythonExpression],
        iterable: PythonExpression,
    ) -> PythonIterator:
        asynchronous = maybe_async is not None
        return self._new_node(PythonIterator, variables, iterable, asynchronous=asynchronous)

    @v_args(inline=True)
    def decorated(
        self,
        decorators: Tuple[PythonDecorator],
        definition: PythonDefinition,
    ) -> PythonDefinition:
        object.__setattr__(definition, 'decorators', decorators)
        return definition

    def decorators(self, children: Iterable[PythonDecorator]) -> Tuple[PythonDecorator]:
        return tuple(children)

    def decorator(self, children: Iterable[Any]) -> PythonDecorator:
        assert len(children) >= 1 and len(children) <= 2, str(children)
        names = children[0]
        assert isinstance(names, tuple), f'expected dotted name: {children}'
        arguments = () if len(children) < 2 else children[1]
        assert isinstance(args, tuple), f'expected arg tuple: {children}'
        return self._new_node(PythonDecorator, names, arguments)

    @v_args(inline=True)
    def async_stmt(self, statement: PythonStatement) -> PythonStatement:
        if statement.is_for:
            object.__setattr__(statement.iterator, 'asynchronous', True)
        elif statement.is_function_def:
            object.__setattr__(statement, 'asynchronous', True)
        elif statement.is_with:
            object.__setattr__(statement, 'asynchronous', True)
        else:
            raise AssertionError(f'async_stmt: {statement!r}')
        return statement

    # Simple Statements ####################################

    def simple_stmt(
        self,
        children: Iterable[Union[PythonStatement, Tuple[PythonStatement]]],
    ) -> Tuple[PythonStatement]:
        return self._flatten_statements(children)

    @v_args(inline=True)
    def expr_stmt(self, expression: PythonExpression):
        return self._new_node(
            PythonExpression,
            expression,
            line=expression.line,
            column=expression.column,
        )

    @v_args(inline=True)
    def pass_stmt(self, token: Token) -> PythonPassStatement:
        return self._new_node(PythonPassStatement, line=token.line, column=token.column)

    @v_args(inline=True)
    def break_stmt(self, token: Token) -> PythonBreakStatement:
        return self._new_node(PythonBreakStatement, line=token.line, column=token.column)

    @v_args(inline=True)
    def continue_stmt(self, token: Token) -> PythonContinueStatement:
        return self._new_node(PythonContinueStatement, line=token.line, column=token.column)

    @v_args(inline=True)
    def del_stmt(self, expressions: Tuple[PythonExpression]) -> PythonDeleteStatement:
        assert len(expressions) > 0, str(children)
        line = getattr(expressions[0], 'line', 0)
        column = getattr(expressions[0], 'column', 0)
        return self._new_node(PythonDeleteStatement, expressions, line=line, column=column)

    @v_args(inline=True)
    def return_stmt(self, maybe_value: MaybeExpressions) -> PythonReturnStatement:
        # return_stmt: "return" [testlist]
        value = None
        line = 0
        column = 0
        if maybe_value is not None:
            value = self._tuple_to_expr(maybe_value, singles=False)
            line = value.line
            column = value.column
        return self._new_node(PythonReturnStatement, value, line=line, column=column)

    @v_args(inline=True)
    def raise_stmt(
        self,
        exception: Optional[PythonExpression],
        cause: Optional[PythonExpression],
    ) -> PythonRaiseStatement:
        line = 0
        column = 0
        if exception is not None:
            line = exception.line
            column = exception.column
        return self._new_node(PythonRaiseStatement, exception, cause, line=line, column=column)

    @v_args(inline=True)
    def yield_stmt(self, expr: PythonYieldExpression) -> PythonExpressionStatement:
        return self._new_node(PythonExpressionStatement, expr)

    def global_stmt(self, names: Iterable[Token]) -> PythonScopeStatement:
        return self._new_node(PythonScopeStatement, tuple(names), global_scope=True)

    def nonlocal_stmt(self, names: Iterable[Token]) -> PythonScopeStatement:
        return self._new_node(PythonScopeStatement, tuple(names), global_scope=False)

    @v_args(inline=True)
    def assert_stmt(
        self,
        test: PythonExpression,
        msg: Optional[PythonExpression],
    ) -> PythonAssertStatement:
        return self._new_node(PythonAssertStatement, test, msg, line=test.line, column=test.column)

    # Assignment Statements ################################

    @v_args(inline=True)
    def assign_stmt(
        self,
        assignments: Tuple[PythonAssignmentStatement],
    ) -> Tuple[PythonAssignmentStatement]:
        return assignments

    @v_args(inline=True)
    def annassign(
        self,
        variable: PythonExpression,
        type_hint: PythonExpression,
        value: Optional[PythonExpression],
    ) -> Tuple[PythonAssignmentStatement]:
        if value is None:
            return ()
        return (
            self._new_node(
                PythonAssignmentStatement,
                variable,
                value,
                type_hint=type_hint,
                line=variable.line,
                column=variable.column,
            ),
        )

    def assign(self, children: Iterable[PythonExpression]) -> Tuple[PythonAssignmentStatement]:
        assert len(children) >= 2, f'assign: {children}'
        statements = []
        for i in range(len(children) - 2, -1, -1):
            variable = children[i]
            value = children[i + 1]
            line = variable.line
            column = variable.column
            statement = self._new_node(
                PythonAssignmentStatement,
                variable,
                value,
                line=line,
                column=column,
            )
            statements.append(statement)
        return tuple(statements)

    @v_args(inline=True)
    def augassign(
        self,
        variable: PythonExpression,
        operator: Token,
        value: SomeExpressions,
    ) -> Tuple[PythonAssignmentStatement]:
        value = self._tuple_to_expr(value)
        return (
            self._new_node(
                PythonAssignmentStatement,
                variable,
                value,
                operator=operator,
                line=variable.line,
                column=variable.column,
            ),
        )

    # Import Statements ####################################

    def dotted_name(self, names: Iterable[Token]) -> Tuple[Token]:
        return tuple(names)

    @v_args(inline=True)
    def dotted_as_name(
        self,
        dotted_name: Tuple[Token],
        alias: Optional[Token] = None,
    ) -> PythonImportedName:
        name = dotted_name[-1]
        base = self._new_node(
            PythonImportBase,
            dotted_name[:-1],
            line=dotted_name[0].line,
            column=dotted_name[0].column,
        )
        return self._new_node(PythonImportedName, base, name, alias=alias)

    def dotted_as_names(self, names: Iterable[PythonImportedName]) -> Tuple[PythonImportedName]:
        return tuple(names)

    @v_args(inline=True)
    def import_name(self, dotted_as_names: Tuple[PythonImportedName]) -> PythonImportStatement:
        assert len(dotted_as_names) > 0, f'import_name: {dotted_as_names}'
        return self._new_node(
            PythonImportStatement,
            dotted_as_names,
            line=dotted_as_names[0].line,
            column=dotted_as_names[0].column,
        )

    @v_args(inline=True)
    def import_as_name(self, name: Token, alias: Optional[Token] = None) -> PythonAliasName:
        return self._new_node(PythonAliasName, name, alias, line=name.line, column=name.column)

    def import_as_names(self, names: Iterable[PythonAliasName]) -> Tuple[PythonAliasName]:
        return tuple(names)

    def import_from(self, children: Iterable) -> PythonImportStatement:
        assert len(children) > 0, f'import_from: {children}'
        i = 0
        dots = 0
        line = 0
        column = 0
        if isinstance(children[0], Token):
            # e.g.: from ..
            i += 1
            dots = len(children[0])
            line=children[0].line
            column=children[0].column
        base = self._new_node(PythonImportBase, (), dots=dots, line=line, column=column)

        items = children[i]
        assert isinstance(items, tuple), f'import_from: {children}'
        if isinstance(items[0], Token):
            # e.g.: from dotted_name
            # e.g.: from .dotted_name
            i += 1
            line = line or items[0].line
            column = column or items[0].column
            base = self._new_node(PythonImportBase, items, dots=dots, line=line, column=column)
        # else: from ..

        assert i > 0, f'import_from: missing "from" part: {children}'
        if i == len(children):
            # e.g.: from .. import *
            # e.g.: from .dotted_name import *
            # FIXME: line and column are wrong, there's no Token for *
            names = (self._new_node(PythonImportedName, base, '*', alias=None),)
            return self._new_node(PythonImportStatement, names, line=line, column=column)

        # e.g.: from . import name as alias
        # e.g.: from .dotted_name import name as alias, name as alias
        items = children[i]
        assert isinstance(items, tuple), f'import_from: {children}'
        assert isinstance(items[0], PythonAliasName), f'import_from: {children}'
        names = tuple(
            self._new_node(PythonImportedName, base, name.name, alias=name.alias)
            for name in items
        )
        return self._new_node(PythonImportStatement, names, line=line, column=column)

    @v_args(inline=True)
    def import_stmt(self, statement: PythonImportStatement) -> PythonImportStatement:
        return statement

    # Function Definition ##################################

    @v_args(inline=True)
    def funcdef(
        self,
        name: Token,
        parameters: Optional[Tuple[PythonFunctionParameter]],
        type_hint: Optional[PythonExpression],
        body: Tuple[PythonStatement],
    ) -> PythonFunctionDefStatement:
        parameters = parameters or ()
        # type_hint = None if type_hint is None else str(type_hint)
        return self._new_node(
            PythonFunctionDefStatement,
            name,
            parameters or (),
            body,
            type_hint=type_hint,
            asynchronous=False,
            line=name.line,
            column=name.column,
        )

    @v_args(inline=True)
    def async_funcdef(self, funcdef: PythonFunctionDefStatement) -> PythonFunctionDefStatement:
        object.__setattr__(funcdef, 'asynchronous', True)
        return funcdef

    def parameters(self, children: Iterable[Any]) -> Tuple[PythonFunctionParameter]:
        params = []
        assert len(children) > 1, f'parameters: {children}'
        for i in range(len(children)):
            child = children[i]
            if child is None:
                continue  # because of square brackets in the grammar
            if child == '/':
                # positional only -> standard
                for j in range(i):
                    object.__setattr__(children[j], 'modifier', '/')
            elif isinstance(child, tuple):
                # starparams
                params.extend(child)
            else:
                # paramvalue | kwparams
                assert isinstance(child, PythonFunctionParameter), f'parameters: {children}'
                params.append(child)
        return tuple(params)

    @v_args(inline=True)
    def starparams(
        self,
        starparam: Union[PythonFunctionParameter, Token],
        poststarparams: Iterable[PythonFunctionParameter],
    ) -> Tuple[PythonFunctionParameter]:
        if starparam == '*':
            return tuple(poststarparams)
        return (starparam,) + tuple(poststarparams)

    @v_args(inline=True)
    def starparam(
        self,
        param: Union[PythonFunctionParameter, Token],
    ) -> PythonFunctionParameter:
        if not isinstance(param, PythonFunctionParameter):
            param = self._new_node(PythonFunctionParameter, param, line=param.line, column=param.column)
        assert param.default_value is None, f'starparam: {param}'
        object.__setattr__(param, 'modifier', '*')
        return param

    def poststarparams(
        self,
        children: Iterable[Union[PythonFunctionParameter, Token]],
    ) -> Tuple[PythonFunctionParameter]:
        params = []
        for param in children:
            if param is None:
                # kwargs could be None if not present
                continue
            if isinstance(param, PythonFunctionParameter):
                # this should be keyword-only
                assert not param.modifier, f'modifier: {param.modifier}'
                object.__setattr__(param, 'modifier', '=')
            else:
                param = self._new_node(
                    PythonFunctionParameter,
                    param,
                    modifier='=',
                    line=param.line,
                    column=param.column,
                )
            params.append(param)
        return tuple(params)

    @v_args(inline=True)
    def kwparams(
        self,
        param: Union[PythonFunctionParameter, Token],
    ) -> PythonFunctionParameter:
        if not isinstance(param, PythonFunctionParameter):
            param = self._new_node(
                PythonFunctionParameter,
                param,
                line=param.line,
                column=param.column,
            )
        assert param.default_value is None, f'kwparams: {param}'
        object.__setattr__(param, 'modifier', '**')
        return param

    @v_args(inline=True)
    def paramvalue(
        self,
        param: Union[PythonFunctionParameter, Token],
        default_value: Optional[PythonExpression] = None,
    ) -> PythonFunctionParameter:
        if isinstance(param, PythonFunctionParameter):
            object.__setattr__(param, 'default_value', default_value)
        else:
            param = self._new_node(
                PythonFunctionParameter,
                param,
                default_value=default_value,
                line=param.line,
                column=param.column,
            )
        return param

    @v_args(inline=True)
    def typedparam(
        self,
        name: Token,
        type_hint: Optional[PythonExpression] = None,
    ) -> PythonFunctionParameter:
        return self._new_node(
            PythonFunctionParameter,
            name,
            type_hint=str(type_hint),  # FIXME
            line=name.line,
            column=name.column,
        )

    # Class Definition #####################################

    @v_args(inline=True)
    def classdef(
        self,
        name: Token,
        arguments: Optional[Tuple[PythonArgument]],
        body: Tuple[PythonStatement],
    ) -> PythonClassDefStatement:
        arguments = arguments or ()
        return self._new_node(
            PythonClassDefStatement,
            name,
            body,
            arguments=arguments,
            line=name.line,
            column=name.column,
        )

    # If Statements ########################################

    @v_args(inline=True)
    def if_stmt(
        self,
        test: PythonExpression,
        body: Tuple[PythonStatement],
        elif_branches: Tuple[PythonConditionalBlock],
        else_branch: MaybeStatements,
    ) -> PythonIfStatement:
        then_branch = self._new_node(
            PythonConditionalBlock,
            test,
            body,
            line=test.line,
            column=test.column,
        )
        else_branch = else_branch or ()  # avoid None
        return self._new_node(PythonIfStatement, then_branch, elif_branches, else_branch)

    def elifs(self, children: Iterable[PythonConditionalBlock]) -> Tuple[PythonConditionalBlock]:
        return tuple(children)

    @v_args(inline=True)
    def elif_(
        self,
        condition: PythonExpression,
        body: Tuple[PythonStatement]
    ) -> PythonConditionalBlock:
        return self._new_node(PythonConditionalBlock, condition, body)

    # Loop Statements ######################################

    @v_args(inline=True)
    def while_stmt(
        self,
        test: PythonExpression,
        body: Tuple[PythonStatement],
        else_branch: MaybeStatements,
    ) -> PythonWhileStatement:
        loop = self._new_node(
            PythonConditionalBlock,
            test,
            body,
            line=test.line,
            column=test.column,
        )
        else_branch = else_branch or ()  # avoid None
        return self._new_node(PythonWhileStatement, loop, else_branch)

    def for_stmt(
        self,
        variables: Tuple[PythonExpression],
        iterables: Tuple[PythonExpression],
        body: Tuple[PythonStatement],
        else_branch: MaybeStatements,
    ) -> PythonForStatement:
        iterable = self._tuple_to_expr(iterables)
        iterator = self._new_node(PythonIterator, variables, iterable, asynchronous=False)
        else_branch = else_branch or ()  # avoid None
        return self._new_node(PythonForStatement, iterator, body, else_branch)

    # Try Statement ########################################

    @v_args(inline=True)
    def try_stmt(
        self,
        body: Tuple[PythonStatement],
        except_clauses: Tuple[PythonExceptClause],
        else_branch: MaybeStatements,
        finally_block: MaybeStatements,
    ) -> PythonTryStatement:
        else_branch = else_branch or ()
        finally_block = finally_block or ()
        return self._new_node(
            PythonTryStatement,
            body,
            except_clauses=except_clauses,
            else_branch=else_branch,
            finally_block=finally_block,
        )

    @v_args(inline=True)
    def try_finally(
        self,
        body: Tuple[PythonStatement],
        finally_block: Tuple[PythonStatement],
    ) -> PythonTryStatement:
        return self._new_node(PythonTryStatement, body, finally_block=finally_block)

    @v_args(inline=True)
    def finally_block(self, body: Tuple[PythonStatement]) -> Tuple[PythonStatement]:
        return body

    def except_clauses(self, clauses: Iterable[PythonExceptClause]) -> Tuple[PythonExceptClause]:
        assert len(clauses) >= 1, f'except_clauses: {clauses}'
        return tuple(clauses)

    @v_args(inline=True)
    def except_clause(
        self,
        exception: Optional[PythonExpression],
        alias: Optional[Token],
        body: Tuple[PythonStatement],
    ) -> PythonExceptClause:
        assert len(body) > 0, f'except_clause: {exception}, {alias}, {body}'
        line = body[0].line
        column = body[0].column
        if exception is not None:
            line = exception.line
            column = exception.column
        return self._new_node(
            PythonExceptClause,
            body,
            exception=exception,
            alias=alias,
            line=line,
            column=column,
        )

    # Lambdas ##############################################

    @v_args(inline=True)
    def lambdef(
        self,
        parameters: Optional[Tuple[PythonFunctionParameter]],
        expression: PythonExpression,
    ) -> PythonLambdaExpression:
        return self._lambdef_common(parameters, expression)

    @v_args(inline=True)
    def lambdef_nocond(
        self,
        parameters: Optional[Tuple[PythonFunctionParameter]],
        expression: PythonExpression,
    ) -> PythonLambdaExpression:
        return self._lambdef_common(parameters, expression)

    def _lambdef_common(
        self,
        parameters: Optional[Tuple[PythonFunctionParameter]],
        expression: PythonExpression,
    ) -> PythonLambdaExpression:
        parameters = parameters or ()  # handle None
        line = expression.line
        column = expression.column
        if parameters:
            line = parameters[0].line
            column = parameters[0].column
        return self._new_node(
            PythonLambdaExpression,
            parameters,
            expression,
            line=line,
            column=column,
        )

    def lambda_params(self, children: Iterable[MaybeParams]) -> Tuple[PythonFunctionParameter]:
        return self.parameters(children)

    @v_args(inline=True)
    def lambda_paramvalue(
        self,
        name: Token,
        default_value: Optional[PythonExpression] = None,
    ) -> PythonFunctionParameter:
        return self._new_node(
            PythonFunctionParameter,
            name,
            default_value=default_value,
            line=name.line,
            column=name.column,
        )

    @v_args(inline=True)
    def lambda_starparams(
        self,
        name: Optional[Token],
        params: Tuple[PythonFunctionParameter],
    ) -> Tuple[PythonFunctionParameter]:
        if name is not None:
            # square brackets places a None if no matches occurred
            assert isinstance(name, Token), f'lambda_starparams: {children}'
            param = self._new_node(
                PythonFunctionParameter,
                name,
                modifier='*',
                line=name.line,
                column=name.column,
            )
            params = (param,) + params
        return params

    def lambda_poststarparams(
        self,
        params: Iterable[Optional[PythonFunctionParameter]],
    ) -> Tuple[PythonFunctionParameter]:
        return self.poststarparams(params)

    @v_args(inline=True)
    def lambda_kwparams(self, name: Token) -> PythonFunctionParameter:
        return self._new_node(
            PythonFunctionParameter,
            name,
            modifier='**',
            line=name.line,
            column=name.column,
        )

    # Match Statement ######################################

    def match_stmt(
        self,
        children: Iterable[Union[PythonExpression, PythonCaseStatement]],
    ) -> PythonMatchStatement:
        assert len(children) >= 2, f'match_stmt: {children!r}'
        value = children[0]
        assert value.is_expression, f'match_stmt: {children!r}'
        cases = tuple(children[1:])
        line = value.line  # FIXME
        column = value.column  # FIXME
        return self._new_node(PythonMatchStatement, value, cases, line=line, column=column)

    @v_args(inline=True)
    def case_stmt(
        self,
        pattern: PythonCasePattern,
        guard: Optional[PythonExpression],
        body: Tuple[PythonStatement],
    ) -> PythonCaseStatement:
        line = pattern.line  # FIXME
        column = pattern.column  # FIXME
        return self._new_node(
            PythonCaseStatement,
            pattern,
            body,
            condition=guard,
            line=line,
            column=column,
        )

    @v_args(inline=True)
    def as_pattern(self, pattern: PythonCasePattern, name: Token) -> PythonCasePattern:
        assert hasattr(pattern, 'alias'), f'as_pattern: {pattern!r}'
        object.__setattr__(pattern, 'alias', name)
        return pattern

    def or_pattern(self, patterns: Iterable[PythonCasePattern]) -> PythonOrCasePattern:
        # inlined if there is only one
        return self._new_node(PythonOrCasePattern, tuple(patterns))

    @v_args(inline=True)
    def capture_pattern(self, name: Token) -> PythonSimpleCasePattern:
        ref = self._token_to_ref(name)
        return self._new_node(PythonSimpleCasePattern, ref)

    @v_args(inline=True)
    def any_pattern(self, token: Token) -> PythonWildcardCasePattern:
        return self._new_node(PythonWildcardCasePattern, line=token.line, column=token.column)

    def sequence_pattern(self, items: Iterable[PythonCasePattern]) -> PythonSequenceCasePattern:
        line = 0
        column = 0
        if len(items) >= 1:
            line = items[0].line
            column = items[0].column
        return self._new_node(PythonSequenceCasePattern, tuple(items), line=line, column=column)

    def mapping_pattern(self, items: Iterable[PythonKeyCasePattern]) -> PythonMappingCasePattern:
        line = 0
        column = 0
        if len(items) >= 1:
            line = items[0].line
            column = items[0].column
        return self._new_node(PythonMappingCasePattern, tuple(items), line=line, column=column)

    def mapping_star_pattern(
        self,
        children: Iterable[Union[PythonKeyCasePattern, Token]],
    ) -> PythonMappingCasePattern:
        assert len(children) >= 2, f'mapping_star_pattern: {children!r}'
        patterns = list(children)
        name = patterns[-1]
        assert isinstance(name, Token), f'mapping_star_pattern: {children!r}'
        patterns[-1] = self._new_node(
            PythonWildcardCasePattern,
            name=name,
            is_star_pattern=True,
            line=name.line,
            column=name.column,
        )
        line = patterns[0].line
        column = patterns[0].column
        return self._new_node(
            PythonMappingCasePattern,
            tuple(patterns),
            line=line,
            column=column,
        )

    @v_args(inline=True)
    def literal_pattern(self, expr: PythonExpression) -> PythonSimpleCasePattern:
        return self._new_node(PythonSimpleCasePattern, expr)

    @v_args(inline=True)
    def mapping_item_pattern(
        self,
        key: PythonSimpleCasePattern,
        pattern: PythonCasePattern,
    ) -> PythonKeyCasePattern:
        return self._new_node(PythonKeyCasePattern, key.expression, pattern)

    @v_args(inline=True)
    def star_pattern(self, name: Token) -> PythonWildcardCasePattern:
        return self._new_node(
            PythonWildcardCasePattern,
            name=(name if name != '_' else None),
            is_star_pattern=True,
            line=name.line,
            column=name.column,
        )

    @v_args(inline=True)
    def class_pattern(
        self,
        pattern: PythonSimpleCasePattern,
        arguments: Optional[Tuple[PythonCasePattern]],
    ) -> PythonClassCasePattern:
        arguments = arguments or ()
        assert pattern.expression.is_reference, f'class_pattern: {pattern}'
        type_reference = pattern.expression
        return self._new_node(PythonClassCasePattern, type_reference, arguments)

    def value(self, children: Iterable[Token]) -> PythonSimpleCasePattern:
        assert len(children) >= 1, f'value: {children}'
        ref = self._token_to_ref(children[0])
        for i in range(1, len(children)):
            ref = self._token_to_ref(children[i], base=ref)
        return self._new_node(PythonSimpleCasePattern, ref)

    @v_args(inline=True)
    def arguments_pattern(
        self,
        positional: Tuple[PythonCasePattern],
        keyword: Optional[Tuple[PythonNamedCasePattern]],
    ) -> Tuple[PythonCasePattern]:
        keyword = keyword or ()
        return positional + keyword

    @v_args(inline=True)
    def no_pos_arguments(
        self,
        kwargs: Tuple[PythonNamedCasePattern],
    ) -> Tuple[PythonNamedCasePattern]:
        return kwargs

    def pos_arg_pattern(self, ps: Iterable[PythonCasePattern]) -> Tuple[PythonCasePattern]:
        return tuple(ps)

    def keyws_arg_pattern(
        self,
        patterns: Iterable[PythonNamedCasePattern],
    ) -> Tuple[PythonNamedCasePattern]:
        return tuple(patterns)

    @v_args(inline=True)
    def keyw_arg_pattern(self, key: Token, pattern: PythonCasePattern) -> PythonNamedCasePattern:
        return self._new_node(
            PythonNamedCasePattern,
            key,
            pattern,
            line=key.line,
            column=key.column,
        )

    # Other Compound Statements ############################

    @v_args(inline=True)
    def with_stmt(
        self,
        managers: Tuple[PythonContextManager],
        body: Tuple[PythonStatement],
    ) -> PythonWithStatement:
        assert len(managers) >= 1, f'with_stmt: {managers}'
        assert len(body) >= 1, f'with_stmt: {body}'
        line = managers[0].line
        column = managers[0].column
        return self._new_node(PythonWithStatement, managers, body, line=line, column=column)

    def with_items(self, managers: Iterable[PythonContextManager]) -> Tuple[PythonContextManager]:
        assert len(managers) >= 1, f'with_items: {managers}'
        return tuple(managers)

    @v_args(inline=True)
    def with_item(self, manager: PythonExpression, alias: Optional[Token]) -> PythonContextManager:
        return self._new_node(PythonContextManager, manager, alias=alias)

    # Other Expressions ####################################

    @v_args(inline=True)
    def await_expr(self, kw: Token, expression: PythonExpression) -> PythonAwaitExpression:
        return self._new_node(PythonAwaitExpression, expression, line=kw.line, column=kw.column)

    @v_args(inline=True)
    def yield_expr(self, expressions: MaybeExpressions) -> PythonYieldExpression:
        line = 0
        column = 0
        if expressions is None:
            expressions = ()
        else:
            assert len(expressions) >= 1, f'yield_expr: {expressions}'
            line = expressions[0].line
            column = expressions[0].column
        return self._new_node(
            PythonYieldExpression,
            expressions,
            is_from=False,
            line=line,
            column=column,
        )

    @v_args(inline=True)
    def yield_from(self, expression: PythonExpression) -> PythonYieldExpression:
        return self._new_node(
            PythonYieldExpression,
            (expression,),
            is_from=True,
            line=expression.line,
            column=expression.column,
        )

    @v_args(inline=True)
    def assign_expr(self, name: Token, expression: PythonExpression) -> PythonAssignmentExpression:
        return self._new_node(
            PythonAssignmentExpression,
            name,
            expression,
            line=name.line,
            column=name.column,
        )

    @v_args(inline=True)
    def star_expr(self, expression: PythonExpression) -> PythonStarExpression:
        return self._new_node(
            PythonStarExpression,
            expression,
            line=expression.line,
            column=expression.column,
        )

    @v_args(inline=True)
    def testlist(self, test_or_tuple: SomeExpressions) -> Tuple[PythonExpression]:
        if isinstance(test_or_tuple, tuple):
            return test_or_tuple
        return (test_or_tuple,)

    def testlist_tuple(self, children: Iterable[PythonExpression]) -> Tuple[PythonExpression]:
        return tuple(children)

    @v_args(inline=True)
    def test(
        self,
        then_expression: PythonExpression,
        condition: PythonExpression,
        else_expression: PythonExpression,
    ) -> PythonConditionalExpression:
        # all other cases have only one child, so the rule is inlined
        return self._new_node(
            PythonConditionalExpression,
            condition,
            then_expression,
            else_expression,
            line=then_expression.line,
            column=then_expression.column,
        )

    # Operators ############################################

    def or_test(self, children: OperatorSequence) -> PythonBinaryOperator:
        return self._op_sequence(children)

    def and_test(self, children: OperatorSequence) -> PythonBinaryOperator:
        return self._op_sequence(children)

    @v_args(inline=True)
    def not_test(self, operator: Token, operand: PythonExpression) -> PythonUnaryOperator:
        # this rule is inlined if the unary operator is not present
        line = operator.line
        column= operator.column
        return self._new_node(PythonUnaryOperator, operator, operand, line=line, column=column)

    def comparison(self, children: OperatorSequence) -> PythonBinaryOperator:
        return self._op_sequence(children)

    def or_expr(self, children: OperatorSequence) -> PythonBinaryOperator:
        return self._op_sequence(children)

    def xor_expr(self, children: OperatorSequence) -> PythonBinaryOperator:
        return self._op_sequence(children)

    def and_expr(self, children: OperatorSequence) -> PythonBinaryOperator:
        return self._op_sequence(children)

    def shift_expr(self, children: OperatorSequence) -> PythonBinaryOperator:
        return self._op_sequence(children)

    def arith_expr(self, children: OperatorSequence) -> PythonBinaryOperator:
        return self._op_sequence(children)

    def term(self, children: OperatorSequence) -> PythonBinaryOperator:
        return self._op_sequence(children)

    def _op_sequence(self, children: OperatorSequence) -> PythonBinaryOperator:
        # this rule is inlined if the operator and second operand are not present
        assert len(children) >= 3 and len(children) % 2 == 1, f'_op_sequence: {children!r}'
        operand1 = children[0]
        for i in range(1, len(children), 2):
            operator = children[i]
            assert isinstance(operator, Token), f'_op_sequence: {children!r}'
            operand2 = children[i + 1]
            assert isinstance(operand2, PythonExpression), f'_op_sequence: {children!r}'
            line = operand1.line
            column= operand1.column
            operand1 = self._new_node(
                PythonBinaryOperator,
                operator,
                operand1,
                operand2,
                line=line,
                column=column,
            )
        return operand1

    @v_args(inline=True)
    def factor(self, operator: Token, operand: PythonExpression) -> PythonUnaryOperator:
        # this rule is inlined if the unary operator is not present
        line = operator.line
        column= operator.column
        return self._new_node(PythonUnaryOperator, operator, operand, line=line, column=column)

    @v_args(inline=True)
    def power(
        self,
        operand1: PythonExpression,
        operand2: PythonExpression,
    ) -> PythonBinaryOperator:
        # this rule is inlined if the operator and second operand are not present
        operator = '**'
        line = operand1.line
        column= operand1.column
        return self._new_node(
            PythonBinaryOperator,
            operator,
            operand1,
            operand2,
            line=line,
            column=column,
        )

    @v_args(inline=True)
    def comp_op(self, operator: Token, other_token: Optional[Token] = None) -> Token:
        if other_token is not None:
            return operator.update(f'{operator.value} {other_token.value}')
        return operator

    # Complex Literals #####################################

    def tuple(self, items: Iterable[PythonExpression]) -> PythonTupleLiteral:
        # missing: line and column; requires modified grammar
        return self._new_node(PythonTupleLiteral, tuple(items))

    @v_args(inline=True)
    def tuple_comprehension(self, gen: PythonGenerator) -> PythonTupleComprehension:
        # missing: line and column; requires modified grammar
        assert gen.result.is_expression, f'tuple_comprehension: {gen}'
        return self._new_node(PythonTupleComprehension, gen.result, gen.iterators, test=gen.test)

    def list(self, items: Iterable[PythonExpression]) -> PythonListLiteral:
        # missing: line and column; requires modified grammar
        return self._new_node(PythonListLiteral, tuple(items))

    @v_args(inline=True)
    def list_comprehension(self, gen: PythonGenerator) -> PythonListComprehension:
        # missing: line and column; requires modified grammar
        assert gen.result.is_expression, f'list_comprehension: {gen}'
        return self._new_node(PythonListComprehension, gen.result, gen.iterators, test=gen.test)

    def dict(self, entries: Iterable[PythonDictEntry]) -> PythonDictLiteral:
        # missing: line and column; requires modified grammar
        return self._new_node(PythonDictLiteral, tuple(entries))

    @v_args(inline=True)
    def dict_comprehension(self, gen: PythonGenerator) -> PythonDictComprehension:
        # missing: line and column; requires modified grammar
        assert gen.result.is_helper, f'dict_comprehension: {gen}'
        assert gen.result.is_key_value, f'dict_comprehension: {gen}'
        return self._new_node(PythonDictComprehension, gen.result, gen.iterators, test=gen.test)

    def set(self, items: Iterable[PythonExpression]) -> PythonSetLiteral:
        # missing: line and column; requires modified grammar
        return self._new_node(PythonSetLiteral, tuple(items))

    @v_args(inline=True)
    def set_comprehension(self, gen: PythonGenerator) -> PythonSetComprehension:
        # missing: line and column; requires modified grammar
        assert gen.result.is_expression, f'set_comprehension: {gen}'
        return self._new_node(PythonSetComprehension, gen.result, gen.iterators, test=gen.test)

    # Simple Expressions ###################################

    @v_args(inline=True)
    def var(self, name: Token) -> PythonReference:
        return self._token_to_ref(name)

    @v_args(inline=True)
    def getattr(self, expr: PythonExpression, name: Token) -> PythonReference:
        return self._token_to_ref(name, base=expr)

    def _token_to_ref(
        self,
        name: Token,
        base: Optional[PythonExpression] = None,
    ) -> PythonReference:
        line = name.line
        column = name.column
        if base is not None:
            try:
                line = base.line
                column = base.column
            except AttributeError:
                line = name.line
                column = name.column
        return self._new_node(PythonReference, name, object=base, line=line, column=column)

    @v_args(inline=True)
    def getitem(self, expr: PythonExpression, subscript: PythonSubscript) -> PythonItemAccess:
        return self._new_node(PythonItemAccess, expr, subscript)

    def subscript_tuple(self, items: Iterable[PythonExpression]) -> PythonTupleLiteral:
        assert len(items) > 1, f'subscript_tuple: {items!r}'
        return self._new_node(
            PythonTupleLiteral,
            tuple(items),
            line=items[0].line,
            column=items[0].column,
        )

    @v_args(inline=True)
    def slice(
        self,
        start: Optional[PythonExpression],
        colon: Token,
        end: Optional[PythonExpression],
        step: Optional[PythonExpression],
    ) -> PythonSlice:
        line = colon.line if start is None else start.line
        column = colon.column if start is None else start.column
        return self._new_node(
            PythonSlice,
            start=start,
            end=end,
            step=step,
            line=line,
            column=column,
        )

    def funccall(self, children: Iterable[Any]) -> PythonFunctionCall:
        function = children[0]
        arguments = () if len(children) == 1 else children[1]
        return self._new_node(
            PythonFunctionCall,
            function,
            arguments,
            line=function.line,
            column=function.column,
        )

    def arguments(self, children: Iterable[PythonAst]) -> Tuple[PythonArgument]:
        args = []
        for arg in children:
            if arg is None:
                continue  # [ starargs | kwargs ]
            if isinstance(arg, tuple):
                args.extend(arg)
            elif arg.is_expression:
                args.append(
                    self._new_node(PythonSimpleArgument, arg, line=arg.line, column=arg.column)
                )
            elif arg.is_helper and arg.is_argument:
                args.append(arg)
            else:
                assert False, f'unexpected argument: {arg}'
        return tuple(args)

    def starargs(self, children: Iterable[PythonAst]) -> Tuple[PythonArgument]:
        args = []
        for arg in children:
            if isinstance(arg, tuple):
                args.extend(arg)
            elif arg.is_expression:
                args.append(
                    self._new_node(PythonSimpleArgument, arg, line=arg.line, column=arg.column)
                )
            elif arg.is_helper and arg.is_argument:
                args.append(arg)
            else:
                assert False, f'unexpected argument: {arg}'
        return tuple(args)

    @v_args(inline=True)
    def stararg(self, argument: PythonExpression) -> PythonSpecialArgument:
        return self._new_node(
            PythonSpecialArgument,
            argument,
            is_double_star=False,
            line=argument.line,
            column=argument.column,
        )

    def kwargs(self, children: Iterable[PythonAst]) -> Tuple[PythonArgument]:
        assert len(children) >= 1
        args = [self._new_node(
            PythonSpecialArgument,
            children[0],
            is_double_star=True,
            line=children[0].line,
            column=children[0].column,
        )]
        for i in range(1, len(children)):
            arg = children[1]
            args.append(
                self._new_node(PythonSimpleArgument, arg, line=arg.line, column=arg.column)
                if arg.is_expression
                else arg
            )
        return tuple(args)

    def argvalue(self, children: Iterable[PythonExpression]) -> PythonSimpleArgument:
        assert len(children) >= 1 and len(children) <= 2, f'argvalue: {children}'
        value = children[-1]
        name = None
        if len(children) == 2:
            assert children[0].is_expression, f'expected reference: {children[0]}'
            assert children[0].is_reference, f'expected reference: {children[0]}'
            assert children[0].object is None, f'expected name: {children[0]}'
            name = children[0].name
        # else:
        #     print('>> argvalue with len(children) == 1')
        # return PythonSimpleArgument(value, name=name, line=value.line, column=value.column)
        return self._new_node(
            PythonSimpleArgument,
            value,
            name=name,
            line=value.line,
            column=value.column,
        )

    def exprlist(self, children: Iterable[PythonExpression]) -> Tuple[PythonExpression]:
        return tuple(children)

    # Atomic Literals ######################################

    def const_none(self, children) -> PythonNoneLiteral:
        # missing: line and column; requires modified grammar
        assert not children
        return self._new_node(PythonNoneLiteral)

    def const_true(self, children) -> PythonBooleanLiteral:
        # missing: line and column; requires modified grammar
        assert not children
        return self._new_node(PythonBooleanLiteral, True)

    def const_false(self, children) -> PythonBooleanLiteral:
        # missing: line and column; requires modified grammar
        assert not children
        # return PythonBooleanLiteral(False)
        return self._new_node(PythonBooleanLiteral, False)

    @v_args(inline=True)
    def string(self, s: PythonStringLiteral) -> PythonStringLiteral:
        return s

    STRING_PREFIX = re.compile(r'([ubf]?r?|r[ubf])("|\')', re.I)

    def STRING(self, s: Token) -> PythonStringLiteral:
        match = self.STRING_PREFIX.match(s)
        assert match is not None, f'expected a match: {s}'
        prefix = match.group(1)
        is_raw = 'r' in prefix
        is_unicode = not 'b' in prefix
        is_format = 'f' in prefix
        value = s[match.end():-1]
        return self._new_node(
            PythonStringLiteral,
            value,
            is_raw=is_raw,
            is_unicode=is_unicode,
            is_format=is_format,
            is_long=False,
            line=s.line,
            column=s.column,
        )

    def LONG_STRING(self, s: Token) -> PythonStringLiteral:
        match = self.STRING_PREFIX.match(s)
        assert match is not None, f'expected a match: {s}'
        prefix = match.group(1)
        is_raw = 'r' in prefix
        is_unicode = not 'b' in prefix
        is_format = 'f' in prefix
        value = s[match.end()+2:-3]
        return self._new_node(
            PythonStringLiteral,
            value,
            is_raw=is_raw,
            is_unicode=is_unicode,
            is_format=is_format,
            is_long=True,
            line=s.line,
            column=s.column,
        )

    @v_args(inline=True)
    def number(self, n: PythonNumberLiteral) -> PythonNumberLiteral:
        return n

    def DEC_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = int(n)
        # return PythonNumberLiteral(v, line=n.line, column=n.column)
        return self._new_node(PythonNumberLiteral, v, line=n.line, column=n.column)

    def HEX_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = int(n, 16)
        # return PythonNumberLiteral(v, line=n.line, column=n.column)
        return self._new_node(PythonNumberLiteral, v, line=n.line, column=n.column)

    def OCT_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = int(n, 8)
        # return PythonNumberLiteral(v, line=n.line, column=n.column)
        return self._new_node(PythonNumberLiteral, v, line=n.line, column=n.column)

    def BIN_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = int(n, 2)
        # return PythonNumberLiteral(v, line=n.line, column=n.column)
        return self._new_node(PythonNumberLiteral, v, line=n.line, column=n.column)

    def DECIMAL(self, n: Token) -> PythonNumberLiteral:
        v = float(n)
        # return PythonNumberLiteral(v, line=n.line, column=n.column)
        return self._new_node(PythonNumberLiteral, v, line=n.line, column=n.column)

    def FLOAT_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = float(n)
        # return PythonNumberLiteral(v, line=n.line, column=n.column)
        return self._new_node(PythonNumberLiteral, v, line=n.line, column=n.column)

    def IMAG_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = complex(0, float(n[:-1]))
        # return PythonNumberLiteral(v, line=n.line, column=n.column)
        return self._new_node(PythonNumberLiteral, v, line=n.line, column=n.column)

    # Keywords and Utilities ###############################

    @v_args(inline=True)
    def name(self, token: Token) -> Token:
        return token
