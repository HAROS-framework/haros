# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, Callable, Iterable, Optional, Tuple, Union

import re

import attr
from lark import Lark, Token, Transformer, v_args
from lark.indenter import PythonIndenter

###############################################################################
# AST
###############################################################################


class PythonAst:
    @property
    def is_statement(self) -> bool:
        return False

    @property
    def is_expression(self) -> bool:
        return False

    @property
    def is_helper(self) -> bool:
        return False


class PythonExpression(PythonAst):
    @property
    def is_expression(self) -> bool:
        return True

    @property
    def is_literal(self) -> bool:
        return False

    @property
    def is_reference(self) -> bool:
        return False

    @property
    def is_function_call(self) -> bool:
        return False

    @property
    def is_star_argument(self) -> bool:
        return False

    @property
    def is_keyword_argument(self) -> bool:
        return False

    @property
    def is_generator(self) -> bool:
        return False

    @property
    def is_operator(self) -> bool:
        return False


class PythonHelperNode(PythonAst):
    @property
    def is_helper(self) -> bool:
        return True

    @property
    def is_key_value(self) -> bool:
        return False

    @property
    def is_iterator(self) -> bool:
        return False

    @property
    def is_argument(self) -> bool:
        return False

    @property
    def is_import_base(self) -> bool:
        return False

    @property
    def is_alias_name(self) -> bool:
        return False

    @property
    def is_imported_name(self) -> bool:
        return False

    @property
    def is_function_parameter(self) -> bool:
        return False

    @property
    def is_conditional_block(self) -> bool:
        return False


class PythonLiteral(PythonExpression):
    @property
    def is_literal(self) -> bool:
        return True

    @property
    def is_none(self) -> bool:
        return False

    @property
    def is_bool(self) -> bool:
        return False

    @property
    def is_number(self) -> bool:
        return False

    @property
    def is_string(self) -> bool:
        return False

    @property
    def is_tuple(self) -> bool:
        return False

    @property
    def is_list(self) -> bool:
        return False

    @property
    def is_dict(self) -> bool:
        return False

    @property
    def is_set(self) -> bool:
        return False


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonNoneLiteral(PythonLiteral):
    # meta
    line: int = 0
    column: int = 0

    @property
    def value(self) -> None:
        return None

    @property
    def is_none(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonBooleanLiteral(PythonLiteral):
    value: bool
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_bool(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonNumberLiteral(PythonLiteral):
    value: Union[int, float, complex]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_number(self) -> bool:
        return True

    @property
    def is_int(self) -> bool:
        return isinstance(self.value, int)

    @property
    def is_float(self) -> bool:
        return isinstance(self.value, float)

    @property
    def is_complex(self) -> bool:
        return isinstance(self.value, complex)


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonStringLiteral(PythonLiteral):
    value: str
    is_raw: bool = False
    is_unicode: bool = True
    is_format: bool = False
    is_long: bool = False
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_string(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonTupleLiteral(PythonLiteral):
    values: Tuple[PythonExpression]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_tuple(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return False


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonListLiteral(PythonLiteral):
    values: Tuple[PythonExpression]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_list(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return False


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonKeyValuePair(PythonHelperNode):
    key: PythonExpression
    value: PythonExpression
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_key_value(self) -> bool:
        return True


PythonDictEntry = Union[PythonKeyValuePair, PythonExpression]


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonDictLiteral(PythonLiteral):
    entries: Tuple[PythonDictEntry]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_dict(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return False


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonSetLiteral(PythonLiteral):
    values: Tuple[PythonExpression]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_set(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return False


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonReference(PythonExpression):
    name: str
    object: Optional[PythonExpression] = None
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_reference(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonIterator(PythonHelperNode):
    variables: Tuple[PythonExpression]
    iterable: PythonExpression
    asynchronous: bool = False

    @property
    def is_iterator(self):
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonTupleComprehension(PythonLiteral):
    expression: PythonExpression
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_tuple(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonListComprehension(PythonLiteral):
    expression: PythonExpression
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_list(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonDictComprehension(PythonLiteral):
    entry: PythonKeyValuePair
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_dict(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonSetComprehension(PythonLiteral):
    expression: PythonExpression
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_set(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonGenerator(PythonExpression):
    result: PythonAst
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_generator(self) -> bool:
        return True


class PythonOperator(PythonExpression):
    @property
    def is_operator(self) -> bool:
        return True

    @property
    def arity(self) -> int:
        return 0

    @property
    def is_unary(self) -> bool:
        return self.arity == 1

    @property
    def is_binary(self) -> bool:
        return self.arity == 2

    @property
    def is_ternary(self) -> bool:
        return self.arity == 3


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonBinaryOperator(PythonOperator):
    operator: str
    operand1: PythonExpression
    operand2: PythonExpression

    @property
    def arity(self) -> int:
        return 2

    @property
    def is_arithmetic(self) -> bool:
        return self.operator in ('+', '-', '*', '/', '**')

    @property
    def is_comparison(self) -> bool:
        return self.operator in ('==', '!=', '<', '<=', '>', '>=')


class PythonArgument(PythonHelperNode):
    @property
    def is_argument(self) -> bool:
        return True

    @property
    def is_positional(self) -> bool:
        return False

    @property
    def is_key_value(self) -> bool:
        return False

    @property
    def is_star(self) -> bool:
        return False

    @property
    def is_keyword(self) -> bool:
        return False


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonFunctionCall(PythonExpression):
    function: PythonExpression
    arguments: Tuple[PythonArgument]

    @property
    def is_function_call(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonSimpleArgument(PythonArgument):
    argument: PythonExpression
    name: Optional[str] = None
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_positional(self) -> bool:
        return self.name is None

    @property
    def is_key_value(self) -> bool:
        return self.name is not None


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonSpecialArgument(PythonArgument):
    argument: PythonExpression
    is_double_star: bool = False
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_star(self) -> bool:
        return not self.is_double_star

    @property
    def is_keyword(self) -> bool:
        return self.is_double_star


class PythonStatement(PythonAst):
    @property
    def is_statement(self) -> bool:
        return True

    @property
    def is_import(self) -> bool:
        return False

    @property
    def is_function_def(self) -> bool:
        return False

    @property
    def is_if(self) -> bool:
        return False


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonImportBase(PythonHelperNode):
    names: Tuple[str]
    dots: int = 0
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_import_base(self) -> bool:
        return True

    @property
    def dotted_name(self) -> str:
        return ('.' * self.dots) + '.'.join(self.names)

    @property
    def is_relative(self) -> bool:
        return self.dots >= 1

    @property
    def is_parent(self) -> bool:
        return self.dots >= 2

    @property
    def is_global(self) -> bool:
        return not self.is_relative

    def add_dots(self, dots: int, line: int = 0, column: int = 0) -> 'PythonImportBase':
        return PythonImportBase(
            self.names,
            dots=(self.dots + dots),
            line=(line or self.line),
            column=(column or self.column),
        )

    def append(self, names: Iterable[str]) -> 'PythonImportBase':
        return PythonImportBase(
            self.names + names,
            dots=self.dots,
            line=self.line,
            column=self.column,
        )


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonAliasName(PythonHelperNode):
    name: str
    alias: str
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_alias_name(self) -> bool:
        return True

    @property
    def is_wildcard(self) -> bool:
        return self.name == '*'

    @classmethod
    def wildcard(cls, line: int = 0, column: int = 0) -> 'PythonAliasName':
        return cls('*', '*', line=line, column=column)


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonImportedName(PythonHelperNode):
    base: PythonImportBase
    name: PythonAliasName

    @property
    def is_imported_name(self) -> bool:
        return True

    @property
    def line(self) -> int:
        return self.base.line

    @property
    def column(self) -> int:
        return self.base.column

    @property
    def is_wildcard(self) -> bool:
        return self.name.is_wildcard


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonImportStatement(PythonStatement):
    names: Tuple[PythonImportedName]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_import(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonFunctionParameter(PythonHelperNode):
    name: str
    default_value: Optional[PythonExpression] = None
    type_hint: Optional[str] = None
    modifier: str = ''
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_function_parameter(self) -> bool:
        return True

    @property
    def is_standard(self) -> bool:
        return not self.modifier

    @property
    def is_positional(self) -> bool:
        return self.modifier == '/'

    @property
    def is_keyword(self) -> bool:
        return self.modifier == '='

    @property
    def is_variadic_list(self) -> bool:
        return self.modifier == '*'

    @property
    def is_variadic_keywords(self) -> bool:
        return self.modifier == '**'


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonFunctionDefStatement(PythonStatement):
    name: str
    parameters: Tuple[PythonFunctionParameter]
    body: PythonStatement
    type_hint: Optional[str] = None
    is_async: bool = False
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_function_def(self) -> bool:
        return True

    @property
    def positional_parameters(self) -> Tuple[PythonFunctionParameter]:
        return tuple(p for p in self.parameters if p.is_positional)

    @property
    def standard_parameters(self) -> Tuple[PythonFunctionParameter]:
        return tuple(p for p in self.parameters if p.is_standard)

    @property
    def keyword_parameters(self) -> Tuple[PythonFunctionParameter]:
        return tuple(p for p in self.parameters if p.is_keyword)

    @property
    def has_variadic_list(self) -> bool:
        return any(p.is_variadic_list for p in self.parameters)

    @property
    def has_variadic_keywords(self) -> bool:
        return any(p.is_variadic_keywords for p in self.parameters)


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonConditionalBlock(PythonHelperNode):
    condition: PythonExpression
    body: Tuple[PythonStatement]

    @property
    def is_conditional_block(self) -> bool:
        return True


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonIfStatement(PythonStatement):
    then_branch: PythonConditionalBlock
    elif_branches: Tuple[PythonConditionalBlock]
    else_branch: Optional[PythonConditionalBlock]

    @property
    def is_if(self) -> bool:
        return True

    @property
    def condition(self) -> PythonExpression:
        return self.then_branch.condition

    @property
    def body(self) -> Tuple[PythonStatement]:
        return self.then_branch.body


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonExpressionStatement(PythonStatement):
    expression: PythonExpression

    @property
    def is_expression(self) -> bool:
        return True


###############################################################################
# Transformer
###############################################################################


class _ToAst(Transformer):
    # Top Level Rules ######################################

    def file_input(self, items):
        if len(items) == 0:
            return items[0]
        return items

    @v_args(inline=True)
    def expr_stmt(self, expression):
        return expression

    def suite(self, children) -> Tuple[PythonStatement]:
        if len(children) == 1:
            stmt = children[0]
            if not stmt.is_statement:
                assert stmt.is_expression, f'suite: {children}'
                stmt = PythonExpressionStatement(stmt)
            return (stmt,)
        # newline indent stmt+ dedent
        return tuple(children)

    # Helper Nodes #########################################

    @v_args(inline=True)
    def key_value(self, key: PythonExpression, value: PythonExpression) -> PythonKeyValuePair:
        return PythonKeyValuePair(key, value)

    def comprehension(self, children: Iterable[Any]) -> PythonGenerator:
        assert len(children) >= 2, f'comprehension: {children}'
        result = children[0]
        iterators = children[1]
        test = None if len(children) == 2 else children[2]
        return PythonGenerator(result, iterators, test=test)

    def comp_fors(self, children: Iterable[PythonIterator]) -> Tuple[PythonIterator]:
        return tuple(children)

    def comp_for(self, children: Iterable[Any]) -> PythonIterator:
        assert len(children) >= 2 and len(children) <= 3, str(children)
        asynchronous = children[0] == 'async'
        c = children[-2]
        variables = c if isinstance(c, tuple) else (c,)
        iterator = children[-1]
        return PythonIterator(variables, iterator, asynchronous=asynchronous)

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
        body: PythonStatement,
    ) -> PythonFunctionDefStatement:
        parameters = parameters or ()
        type_hint = None if type_hint is None else str(type_hint)  # FIXME
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
        params: Iterable[Union[PythonFunctionParameter, Token]],
    ) -> Tuple[PythonFunctionParameter]:
        return tuple(
            PythonFunctionParameter(
                param,
                modifier='=',
                line=name.line,
                column=name.column,
            )
            if not isinstance(param, PythonFunctionParameter)
            else param
            for param in params
        )

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
        condition: PythonExpression,
        body: Tuple[PythonStatement],
        elif_branches: Tuple[PythonConditionalBlock],
        else_branch: Optional[PythonConditionalBlock]
    ) -> PythonIfStatement:
        then_branch = PythonConditionalBlock(condition, body)
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

    # Operators ############################################

    def comparison(self, children) -> PythonBinaryOperator:
        assert len(children) % 2 == 1, f'comparison: {children}'
        op = children[0]
        for i in range(1, len(children), 2):
            assert isinstance(children[i], Token), f'comparison: {children}'
            assert isinstance(children[i+1], PythonExpression), f'comparison: {children}'
            op = PythonBinaryOperator(children[i], op, children[i+1])
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

    def funccall(self, children: Iterable[Any]) -> PythonFunctionCall:
        function = children[0]
        arguments = () if len(children) == 1 else children[1]
        return PythonFunctionCall(function, arguments)

    def arguments(self, children: Iterable[PythonAst]) -> Tuple[PythonExpression]:
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


###############################################################################
# Interface
###############################################################################


class _PythonParser:
    def __init__(self):
        self._parser = Lark.open_from_package(
            'lark',
            'python.lark',
            ['grammars'],
            parser='lalr',
            postlex=PythonIndenter(),
            start='file_input',
        )
        self._transformer = _ToAst()

    def parse(self, text: str):
        tree = self._parser.parse(text)
        return self._transformer.transform(tree)


_parser = None


def parser():
    global _parser
    if _parser is None:
        _parser = _PythonParser()
    return _parser


def parse(text):
    return parser().parse(text)
