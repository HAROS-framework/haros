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
class PythonFunctionCall(PythonExpression):
    function: PythonExpression
    arguments: Tuple[PythonExpression]

    @property
    def is_function_call(self) -> bool:
        return True


###############################################################################
# Transformer
###############################################################################


class _ToAst(Transformer):
    # Top Level Rules ######################################

    @v_args(inline=True)
    def file_input(self, item):
        return item

    @v_args(inline=True)
    def expr_stmt(self, expression):
        return expression

    # Helper Nodes #########################################

    @v_args(inline=True)
    def key_value(self, key: PythonExpression, value: PythonExpression) -> PythonKeyValuePair:
        return PythonKeyValuePair(key, value)

    def comprehension(self, children: Iterable[Any]) -> Tuple[PythonAst]:
        assert len(children) >= 2, f'comprehension: {children}'
        result = children[0]
        iterators = children[1]
        test = None if len(children) == 2 else children[2]
        return result, iterators, test

    def comp_fors(self, children: Iterable[PythonIterator]) -> Tuple[PythonIterator]:
        return tuple(children)

    def comp_for(self, children: Iterable[Any]) -> PythonIterator:
        assert len(children) >= 2 and len(children) <= 3, str(children)
        asynchronous = children[0] == 'async'
        c = children[-2]
        variables = c if isinstance(c, tuple) else (c,)
        iterator = children[-1]
        return PythonIterator(variables, iterator, asynchronous=asynchronous)

    # Complex Literals #####################################

    def tuple(self, items: Iterable[PythonExpression]) -> PythonTupleLiteral:
        # missing: line and column; requires modified grammar
        return PythonTupleLiteral(tuple(items))

    @v_args(inline=True)
    def tuple_comprehension(self, children: Iterable[Any]) -> PythonTupleComprehension:
        # missing: line and column; requires modified grammar
        assert len(children) == 3, f'tuple_comprehension: {children}'
        return PythonTupleComprehension(children[0], children[1], test=children[2])

    def list(self, items: Iterable[PythonExpression]) -> PythonListLiteral:
        # missing: line and column; requires modified grammar
        return PythonListLiteral(tuple(items))

    @v_args(inline=True)
    def list_comprehension(self, children: Iterable[Any]) -> PythonListComprehension:
        # missing: line and column; requires modified grammar
        assert len(children) == 3, f'list_comprehension: {children}'
        return PythonListComprehension(children[0], children[1], test=children[2])

    def dict(self, entries: Iterable[PythonDictEntry]) -> PythonDictLiteral:
        # missing: line and column; requires modified grammar
        return PythonDictLiteral(tuple(entries))

    @v_args(inline=True)
    def dict_comprehension(self, children: Iterable[Any]) -> PythonDictComprehension:
        # missing: line and column; requires modified grammar
        assert len(children) == 3, f'dict_comprehension: {children}'
        assert isinstance(children[0], PythonKeyValuePair), repr(children[0])
        assert isinstance(children[1], tuple), repr(children[1])
        assert children[2] is None or isinstance(children[2], PythonExpression), repr(children[2])
        return PythonDictComprehension(children[0], children[1], test=children[2])

    def set(self, items: Iterable[PythonExpression]) -> PythonSetLiteral:
        # missing: line and column; requires modified grammar
        return PythonSetLiteral(tuple(items))

    @v_args(inline=True)
    def set_comprehension(self, children: Iterable[Any]) -> PythonSetComprehension:
        # missing: line and column; requires modified grammar
        assert len(children) == 3, f'set_comprehension: {children}'
        return PythonSetComprehension(children[0], children[1], test=children[2])

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

    def arguments(self, children: Iterable[PythonExpression]) -> Tuple[PythonExpression]:
        return tuple(children)

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
