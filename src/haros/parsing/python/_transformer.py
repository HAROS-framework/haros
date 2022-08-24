# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Iterable, Optional, Tuple, Union

import re

from lark import Token, Transformer, v_args

from haros.parsing.python.ast import (
    PythonAliasName,
    PythonArgument,
    PythonAssertStatement,
    PythonAssignmentExpression,
    PythonAssignmentStatement,
    PythonAst,
    PythonBinaryOperator,
    PythonBooleanLiteral,
    PythonBreakStatement,
    PythonClassDefStatement,
    PythonConditionalBlock,
    PythonConditionalExpression,
    PythonContinueStatement,
    PythonDecorator,
    PythonDeleteStatement,
    PythonDictComprehension,
    PythonDictEntry,
    PythonDictLiteral,
    PythonExpression,
    PythonExpressionStatement,
    PythonFunctionCall,
    PythonFunctionDefStatement,
    PythonFunctionParameter,
    PythonGenerator,
    PythonIfStatement,
    PythonImportBase,
    PythonImportStatement,
    PythonImportedName,
    PythonIterator,
    PythonKeyValuePair,
    PythonLambdaExpression,
    PythonListComprehension,
    PythonListLiteral,
    PythonNoneLiteral,
    PythonNumberLiteral,
    PythonPassStatement,
    PythonRaiseStatement,
    PythonReference,
    PythonReturnStatement,
    PythonScopeStatement,
    PythonSetComprehension,
    PythonSetLiteral,
    PythonSimpleArgument,
    PythonSpecialArgument,
    PythonStarExpression,
    PythonStatement,
    PythonStringLiteral,
    PythonTupleComprehension,
    PythonTupleLiteral,
    PythonWhileStatement,
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


class ToAst(Transformer):
    # Top Level Rules ######################################

    def file_input(self, children: Iterable[SomeStatements]) -> Tuple[PythonStatement]:
        return self._flatten_statements(children)

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

    def _tuple_to_expr(self, value: SomeExpressions) -> PythonExpression:
        if isinstance(value, tuple):
            return PythonTupleLiteral(value, line=value[0].line, column=value[0].column)
        assert isinstance(value, PythonExpression), f'_tuple_to_expr: {value!r}'
        return value

    # Helper Nodes #########################################

    @v_args(inline=True)
    def key_value(self, key: PythonExpression, value: PythonExpression) -> PythonKeyValuePair:
        return PythonKeyValuePair(key, value)

    @v_args(inline=True)
    def comprehension(
        self,
        result: PythonAst,
        iterators: Tuple[PythonIterator],
        test: Optional[PythonExpression],
    ) -> PythonGenerator:
        return PythonGenerator(result, iterators, test=test)

    def comp_fors(self, children: Iterable[PythonIterator]) -> Tuple[PythonIterator]:
        return tuple(children)

    @v_args(inline=True)
    def comp_for(
        self,
        asynchronous: Optional[Token],
        variables: Tuple[PythonExpression],
        iterable: PythonExpression,
    ) -> PythonIterator:
        is_async = asynchronous is not None
        return PythonIterator(variables, iterable, asynchronous=is_async)

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
        return PythonDecorator(names, arguments)

    # Simple Statements ####################################

    def simple_stmt(
        self,
        children: Iterable[Union[PythonStatement, Tuple[PythonStatement]]],
    ) -> Tuple[PythonStatement]:
        return self._flatten_statements(children)

    @v_args(inline=True)
    def expr_stmt(self, expression: PythonExpression):
        return PythonExpression(expression, line=expression.line, column=expression.column)

    @v_args(inline=True)
    def pass_stmt(self, token: Token) -> PythonPassStatement:
        return PythonPassStatement(line=token.line, column=token.column)

    @v_args(inline=True)
    def break_stmt(self, token: Token) -> PythonBreakStatement:
        return PythonBreakStatement(line=token.line, column=token.column)

    @v_args(inline=True)
    def continue_stmt(self, token: Token) -> PythonContinueStatement:
        return PythonContinueStatement(line=token.line, column=token.column)

    @v_args(inline=True)
    def del_stmt(self, expressions: Tuple[PythonExpression]) -> PythonDeleteStatement:
        assert len(expressions) > 0, str(children)
        line = getattr(expressions[0], 'line', 0)
        column = getattr(expressions[0], 'column', 0)
        return PythonDeleteStatement(expressions, line=line, column=column)

    @v_args(inline=True)
    def return_stmt(self, maybe_value: MaybeExpressions) -> PythonReturnStatement:
        # return_stmt: "return" [testlist]
        value = None
        line = 0
        column = 0
        if maybe_value is not None:
            value = self._tuple_to_expr(maybe_value)
            line = value.line
            column = value.column
        return PythonReturnStatement(expressions, line=line, column=column)

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
        return PythonRaiseStatement(exception, cause, line=line, column=column)

    @v_args(inline=True)
    def yield_stmt(self, expr: PythonYieldExpression) -> PythonExpressionStatement:
        return PythonExpressionStatement(expr)

    def global_stmt(self, names: Iterable[Token]) -> PythonScopeStatement:
        return PythonScopeStatement(tuple(names), global_scope=True)

    def nonlocal_stmt(self, names: Iterable[Token]) -> PythonScopeStatement:
        return PythonScopeStatement(tuple(names), global_scope=False)

    @v_args(inline=True)
    def assert_stmt(
        self,
        test: PythonExpression,
        msg: Optional[PythonExpression],
    ) -> PythonAssertStatement:
        return PythonAssertStatement(test, msg, line=test.line, column=test.column)

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
            PythonAssignmentStatement(
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
            statement = PythonAssignmentStatement(variable, value, line=line, column=column)
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
            PythonAssignmentStatement(
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
        alias = alias or name
        name = PythonAliasName(name, alias, line=name.line, column=name.column)
        base = PythonImportBase(
            dotted_name[:-1],
            line=dotted_name[0].line,
            column=dotted_name[0].column,
        )
        return PythonImportedName(base, name)

    def dotted_as_names(self, names: Iterable[PythonImportedName]) -> Tuple[PythonImportedName]:
        return tuple(names)

    @v_args(inline=True)
    def import_name(self, dotted_as_names: Tuple[PythonImportedName]) -> PythonImportStatement:
        assert len(dotted_as_names) > 0, f'import_name: {dotted_as_names}'
        return PythonImportStatement(
            dotted_as_names,
            line=dotted_as_names[0].line,
            column=dotted_as_names[0].column,
        )

    @v_args(inline=True)
    def import_as_name(self, name: Token, alias: Optional[Token] = None) -> PythonAliasName:
        return PythonAliasName(name, alias or name, line=name.line, column=name.column)

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
        base = PythonImportBase((), dots=dots, line=line, column=column)

        items = children[i]
        assert isinstance(items, tuple), f'import_from: {children}'
        if isinstance(items[0], Token):
            # e.g.: from dotted_name
            # e.g.: from .dotted_name
            i += 1
            line = line or items[0].line
            column = column or items[0].column
            base = PythonImportBase(items, dots=dots, line=line, column=column)
        # else: from ..

        assert i > 0, f'import_from: missing "from" part: {children}'
        if i == len(children):
            # e.g.: from .. import *
            # e.g.: from .dotted_name import *
            # FIXME: line and column are wrong, there's no Token for *
            wildcard = PythonAliasName.wildcard(line=line, column=column)
            names = (PythonImportedName(base, wildcard),)
            return PythonImportStatement(names, line=line, column=column)

        # e.g.: from . import name as alias
        # e.g.: from .dotted_name import name as alias, name as alias
        items = children[i]
        assert isinstance(items, tuple), f'import_from: {children}'
        assert isinstance(items[0], PythonAliasName), f'import_from: {children}'
        names = tuple(PythonImportedName(base, name) for name in items)
        return PythonImportStatement(names, line=line, column=column)

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
        return PythonFunctionDefStatement(
            name,
            parameters or (),
            body,
            type_hint=type_hint,
            is_async=False,
            line=name.line,
            column=name.column,
        )

    @v_args(inline=True)
    def async_funcdef(self, funcdef: PythonFunctionDefStatement) -> PythonFunctionDefStatement:
        object.__setattr__(funcdef, 'is_async', True)
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
            param = PythonFunctionParameter(param, line=param.line, column=param.column)
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
                param = PythonFunctionParameter(
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
            param = PythonFunctionParameter(param, line=param.line, column=param.column)
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
            param = PythonFunctionParameter(
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
        return PythonFunctionParameter(
            name,
            type_hint=str(type_hint),  # FIXME
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
        else_branch: Optional[Tuple[PythonStatement]]
    ) -> PythonIfStatement:
        then_branch = PythonConditionalBlock(test, body, line=test.line, column=test.column)
        else_branch = else_branch or ()  # avoid None
        return PythonIfStatement(then_branch, elif_branches, else_branch)

    def elifs(self, children: Iterable[PythonConditionalBlock]) -> Tuple[PythonConditionalBlock]:
        return tuple(children)

    @v_args(inline=True)
    def elif_(
        self,
        condition: PythonExpression,
        body: Tuple[PythonStatement]
    ) -> PythonConditionalBlock:
        return PythonConditionalBlock(condition, body)

    # Loop Statements ######################################

    @v_args(inline=True)
    def while_stmt(
        self,
        test: PythonExpression,
        body: Tuple[PythonStatement],
        else_branch: Optional[Tuple[PythonStatement]],
    ) -> PythonWhileStatement:
        loop = PythonConditionalBlock(test, body, line=test.line, column=test.column)
        else_branch = else_branch or ()  # avoid None
        return PythonWhileStatement(loop, else_branch)

    def for_stmt(
        self,
        variables: Tuple[PythonExpression],
        iterables: Tuple[PythonExpression],
    ) -> PythonForStatement:
        pass

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
        return PythonLambdaExpression(parameters, expression, line=line, column=column)

    def lambda_params(self, children: Iterable[MaybeParams]) -> Tuple[PythonFunctionParameter]:
        return self.parameters(children)

    @v_args(inline=True)
    def lambda_paramvalue(
        self,
        name: Token,
        default_value: Optional[PythonExpression] = None,
    ) -> PythonFunctionParameter:
        return PythonFunctionParameter(
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
            param = PythonFunctionParameter(
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
        return PythonFunctionParameter(
            name,
            modifier='**',
            line=name.line,
            column=name.column,
        )

    # Other Expressions ####################################

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
        return PythonYieldExpression(expressions, is_from=False, line=line, column=column)

    @v_args(inline=True)
    def yield_from(self, expression: PythonExpression) -> PythonYieldExpression:
        return PythonYieldExpression(
            (expression,),
            is_from=True,
            line=expression.line,
            column=expression.column,
        )

    @v_args(inline=True)
    def assign_expr(self, name: Token, expression: PythonExpression) -> PythonAssignmentExpression:
        return PythonAssignmentExpression(name, expression, line=name.line, column=name.column)

    @v_args(inline=True)
    def star_expr(self, expression: PythonExpression) -> PythonStarExpression:
        return PythonStarExpression(expression, line=expression.line, column=expression.column)

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
        return PythonConditionalExpression(
            condition,
            then_expression,
            else_expression,
            line=then_expression.line,
            column=then_expression.column,
        )

    # Operators ############################################

    def comparison(self, children) -> PythonBinaryOperator:
        assert len(children) % 2 == 1, f'comparison: {children}'
        op = children[0]
        for i in range(1, len(children), 2):
            assert isinstance(children[i], Token), f'comparison: {children}'
            assert isinstance(children[i+1], PythonExpression), f'comparison: {children}'
            op = PythonBinaryOperator(
                children[i],
                op,
                children[i+1],
                line=children[1].line,
                column=children[1].column,
            )
        return op

    @v_args(inline=True)
    def comp_op(self, operator: Token) -> Token:
        return operator

    # Complex Literals #####################################

    def tuple(self, items: Iterable[PythonExpression]) -> PythonTupleLiteral:
        # missing: line and column; requires modified grammar
        return PythonTupleLiteral(tuple(items))

    @v_args(inline=True)
    def tuple_comprehension(self, gen: PythonGenerator) -> PythonTupleComprehension:
        # missing: line and column; requires modified grammar
        assert gen.result.is_expression, f'tuple_comprehension: {gen}'
        return PythonTupleComprehension(gen.result, gen.iterators, test=gen.test)

    def list(self, items: Iterable[PythonExpression]) -> PythonListLiteral:
        # missing: line and column; requires modified grammar
        return PythonListLiteral(tuple(items))

    @v_args(inline=True)
    def list_comprehension(self, gen: PythonGenerator) -> PythonListComprehension:
        # missing: line and column; requires modified grammar
        assert gen.result.is_expression, f'list_comprehension: {gen}'
        return PythonListComprehension(gen.result, gen.iterators, test=gen.test)

    def dict(self, entries: Iterable[PythonDictEntry]) -> PythonDictLiteral:
        # missing: line and column; requires modified grammar
        return PythonDictLiteral(tuple(entries))

    @v_args(inline=True)
    def dict_comprehension(self, gen: PythonGenerator) -> PythonDictComprehension:
        # missing: line and column; requires modified grammar
        assert gen.result.is_helper, f'dict_comprehension: {gen}'
        assert gen.result.is_key_value, f'dict_comprehension: {gen}'
        return PythonDictComprehension(gen.result, gen.iterators, test=gen.test)

    def set(self, items: Iterable[PythonExpression]) -> PythonSetLiteral:
        # missing: line and column; requires modified grammar
        return PythonSetLiteral(tuple(items))

    @v_args(inline=True)
    def set_comprehension(self, gen: PythonGenerator) -> PythonSetComprehension:
        # missing: line and column; requires modified grammar
        assert gen.result.is_expression, f'set_comprehension: {gen}'
        return PythonSetComprehension(gen.result, gen.iterators, test=gen.test)

    # Simple Expressions ###################################

    @v_args(inline=True)
    def var(self, name: Token) -> PythonReference:
        return PythonReference(
            str(name),
            object=None,
            line=name.line,
            column=name.column,
        )

    @v_args(inline=True)
    def getattr(self, expr: PythonExpression, name: Token) -> PythonReference:
        try:
            line = expr.line
            column = expr.column
        except AttributeError:
            line = name.line
            column = name.column
        return PythonReference(name, object=expr, line=line, column=column)

    # @v_args(inline=True)
    # def getitem(
    #     self,
    #     expr: PythonExpression,
    #     subscripts: Union[PythonExpression, Tuple[PythonExpression]],
    # ) -> PythonSubscript:
    #     pass

    def funccall(self, children: Iterable[Any]) -> PythonFunctionCall:
        function = children[0]
        arguments = () if len(children) == 1 else children[1]
        return PythonFunctionCall(function, arguments, line=function.line, column=function.column)

    def arguments(self, children: Iterable[PythonAst]) -> Tuple[PythonArgument]:
        args = []
        for arg in children:
            if arg is None:
                continue  # [ starargs | kwargs ]
            if isinstance(arg, tuple):
                args.extend(arg)
            elif arg.is_expression:
                args.append(PythonSimpleArgument(arg, line=arg.line, column=arg.column))
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
                args.append(PythonSimpleArgument(arg, line=arg.line, column=arg.column))
            elif arg.is_helper and arg.is_argument:
                args.append(arg)
            else:
                assert False, f'unexpected argument: {arg}'
        return tuple(args)

    @v_args(inline=True)
    def stararg(self, argument: PythonExpression) -> PythonSpecialArgument:
        return PythonSpecialArgument(
            argument,
            is_double_star=False,
            line=argument.line,
            column=argument.column,
        )

    def kwargs(self, children: Iterable[PythonAst]) -> Tuple[PythonArgument]:
        assert len(children) >= 1
        args = [PythonSpecialArgument(
            children[0],
            is_double_star=True,
            line=children[0].line,
            column=children[0].column,
        )]
        for i in range(1, len(children)):
            arg = children[1]
            args.append(
                PythonSimpleArgument(arg, line=arg.line, column=arg.column)
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
        return PythonSimpleArgument(value, name=name, line=value.line, column=value.column)

    def exprlist(self, children: Iterable[PythonExpression]) -> Tuple[PythonExpression]:
        return tuple(children)

    # Atomic Literals ######################################

    def const_none(self, children) -> PythonNoneLiteral:
        # missing: line and column; requires modified grammar
        assert not children
        return PythonNoneLiteral()

    def const_true(self, children) -> PythonBooleanLiteral:
        # missing: line and column; requires modified grammar
        assert not children
        return PythonBooleanLiteral(True)

    def const_false(self, children) -> PythonBooleanLiteral:
        # missing: line and column; requires modified grammar
        assert not children
        return PythonBooleanLiteral(False)

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
        return PythonStringLiteral(
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
        return PythonStringLiteral(
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
        return PythonNumberLiteral(v, line=n.line, column=n.column)

    def HEX_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = int(n, 16)
        return PythonNumberLiteral(v, line=n.line, column=n.column)

    def OCT_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = int(n, 8)
        return PythonNumberLiteral(v, line=n.line, column=n.column)

    def BIN_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = int(n, 2)
        return PythonNumberLiteral(v, line=n.line, column=n.column)

    def DECIMAL(self, n: Token) -> PythonNumberLiteral:
        v = float(n)
        return PythonNumberLiteral(v, line=n.line, column=n.column)

    def FLOAT_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = float(n)
        return PythonNumberLiteral(v, line=n.line, column=n.column)

    def IMAG_NUMBER(self, n: Token) -> PythonNumberLiteral:
        v = complex(0, float(n[:-1]))
        return PythonNumberLiteral(v, line=n.line, column=n.column)

    # Keywords and Utilities ###############################

    @v_args(inline=True)
    def name(self, token: Token) -> Token:
        return token
