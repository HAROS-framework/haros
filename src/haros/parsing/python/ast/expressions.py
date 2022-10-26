# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Optional, Tuple, Union

from attrs import frozen

from haros.parsing.python.ast.common import PythonAst, PythonExpression, PythonHelperNode
from haros.parsing.python.ast.helpers import (
    PythonArgument,
    PythonDictEntry,
    PythonFunctionParameter,
    PythonIterator,
    PythonKeyValuePair,
    PythonSubscript,
)

###############################################################################
# Literals
###############################################################################


@frozen
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


@frozen
class PythonNoneLiteral(PythonLiteral):
    @property
    def value(self) -> None:
        return None

    @property
    def is_none(self) -> bool:
        return True

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        return ws + 'None Literal'


@frozen
class PythonBooleanLiteral(PythonLiteral):
    value: bool

    @property
    def is_bool(self) -> bool:
        return True

    @classmethod
    def const_true(cls) -> 'PythonBooleanLiteral':
        return cls(True)

    @classmethod
    def const_false(cls) -> 'PythonBooleanLiteral':
        return cls(False)

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws1 = ' ' * indent
        ws2 = ' ' * (indent + step)
        return f'{ws1}Boolean Literal\n{ws2}{self.value}'


@frozen
class PythonNumberLiteral(PythonLiteral):
    value: Union[int, float, complex]

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

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws1 = ' ' * indent
        ws2 = ' ' * (indent + step)
        return f'{ws1}Number Literal\n{ws2}{self.value}'


@frozen
class PythonStringLiteral(PythonLiteral):
    value: str
    is_raw: bool = False
    is_unicode: bool = True
    is_format: bool = False
    is_long: bool = False

    @property
    def is_string(self) -> bool:
        return True

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws1 = ' ' * indent
        ws2 = ' ' * (indent + step)
        return f'{ws1}String Literal\n{ws2}{repr(self.value)}'


@frozen
class PythonTupleLiteral(PythonLiteral):
    values: Tuple[PythonExpression]

    @property
    def is_tuple(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return False

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        values = '\n'.join(v.pretty(indent=(indent+step)) for v in self.values)
        return f'{ws}Tuple Literal\n{values}'


@frozen
class PythonListLiteral(PythonLiteral):
    values: Tuple[PythonExpression]

    @property
    def is_list(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return False

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        values = '\n'.join(v.pretty(indent=(indent+step)) for v in self.values)
        return f'{ws}List Literal\n{values}'


@frozen
class PythonDictLiteral(PythonLiteral):
    entries: Tuple[PythonDictEntry]

    @property
    def is_dict(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return False

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        entries = '\n'.join(e.pretty(indent=(indent+step)) for e in self.entries)
        return f'{ws}Dict Literal\n{entries}'


@frozen
class PythonSetLiteral(PythonLiteral):
    values: Tuple[PythonExpression]

    @property
    def is_set(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return False

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        values = '\n'.join(v.pretty(indent=(indent+step)) for v in self.values)
        return f'{ws}Set Literal\n{values}'


@frozen
class PythonTupleComprehension(PythonLiteral):
    expression: PythonExpression
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None

    @property
    def is_tuple(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return True


@frozen
class PythonListComprehension(PythonLiteral):
    expression: PythonExpression
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None

    @property
    def is_list(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return True


@frozen
class PythonDictComprehension(PythonLiteral):
    entry: PythonKeyValuePair
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None

    @property
    def is_dict(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return True


@frozen
class PythonSetComprehension(PythonLiteral):
    expression: PythonExpression
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None

    @property
    def is_set(self) -> bool:
        return True

    @property
    def is_comprehension(self) -> bool:
        return True


###############################################################################
# Simple Expressions
###############################################################################


@frozen
class PythonReference(PythonExpression):
    name: str
    object: Optional[PythonExpression] = None

    @property
    def is_reference(self) -> bool:
        return True

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws1 = ' ' * indent
        ws2 = ' ' * (indent + step)
        if self.object is None:
            return f'{ws1}Reference\n{ws2}{name}'
        object = self.object.pretty(indent=(indent+step), step=step)
        return f'{ws1}Reference\n{ws2}{name}\n{object}'


@frozen
class PythonItemAccess(PythonExpression):
    object: PythonExpression
    key: PythonSubscript

    # @property
    # def line(self) -> int:
    #     return self.object.line

    # @property
    # def column(self) -> int:
    #     return self.object.column

    @property
    def is_item_access(self) -> bool:
        return True

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        subindent = indent + step
        object = self.object.pretty(indent=subindent, step=step)
        key = self.key.pretty(indent=subindent, step=step)
        return f'{ws}Subscript\n{object}\n{key}'


###############################################################################
# Operators
###############################################################################


@frozen
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


@frozen
class PythonUnaryOperator(PythonOperator):
    operator: str
    operand: PythonExpression

    @property
    def arity(self) -> int:
        return 1

    @property
    def is_arithmetic(self) -> bool:
        return self.operator in ('+', '-', '~')

    @property
    def is_bitwise(self) -> bool:
        return self.operator == '~'

    @property
    def is_logic(self) -> bool:
        return self.operator == 'not'


@frozen
class PythonBinaryOperator(PythonOperator):
    operator: str
    operand1: PythonExpression
    operand2: PythonExpression

    @property
    def arity(self) -> int:
        return 2

    @property
    def is_arithmetic(self) -> bool:
        return self.operator in ('+', '-', '*', '/', '//', '**')

    @property
    def is_comparison(self) -> bool:
        return self.operator in ('==', '!=', '<', '<=', '>', '>=', 'in', 'not in', 'is', 'is not')

    @property
    def is_logic(self) -> bool:
        return self.operator in ('and', 'or')

    def invert(self) -> 'PythonBinaryOperator':
        op = self.operator
        if op == '+':
            return PythonBinaryOperator('-', self.operand1, self.operand2)
        if op == '-':
            return PythonBinaryOperator('+', self.operand1, self.operand2)

        if op == '*':
            return PythonBinaryOperator('/', self.operand1, self.operand2)
        if op == '/':
            return PythonBinaryOperator('*', self.operand1, self.operand2)

        if op == '==':
            return PythonBinaryOperator('!=', self.operand1, self.operand2)
        if op == '!=':
            return PythonBinaryOperator('==', self.operand1, self.operand2)

        if op == '<':
            return PythonBinaryOperator('>=', self.operand1, self.operand2)
        if op == '>':
            return PythonBinaryOperator('<=', self.operand1, self.operand2)

        if op == '<=':
            return PythonBinaryOperator('>', self.operand1, self.operand2)
        if op == '>=':
            return PythonBinaryOperator('<', self.operand1, self.operand2)

        raise ValueError(f'unable to invert operator {op!r}')


@frozen
class PythonConditionalExpression(PythonExpression):
    condition: PythonExpression
    expression1: PythonExpression
    expression2: PythonExpression

    @property
    def is_conditional(self) -> bool:
        return True

    @property
    def if_expression(self) -> PythonExpression:
        return self.condition

    @property
    def then_expression(self) -> PythonExpression:
        return self.expression1

    @property
    def else_expression(self) -> PythonExpression:
        return self.expression2


###############################################################################
# Complex Expressions
###############################################################################


@frozen
class PythonGenerator(PythonExpression):
    result: PythonAst
    iterators: Tuple[PythonIterator]
    test: Optional[PythonExpression] = None

    @property
    def is_generator(self) -> bool:
        return True


@frozen
class PythonFunctionCall(PythonExpression):
    function: PythonExpression
    arguments: Tuple[PythonArgument]

    @property
    def is_function_call(self) -> bool:
        return True


@frozen
class PythonLambdaExpression(PythonExpression):
    parameters: Tuple[PythonFunctionParameter]
    expression: PythonExpression

    @property
    def is_lambda(self) -> bool:
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


@frozen
class PythonAssignmentExpression(PythonExpression):
    name: str
    expression: PythonExpression

    @property
    def is_assignment(self) -> bool:
        return True


@frozen
class PythonYieldExpression(PythonExpression):
    expressions: Tuple[PythonExpression]
    is_from: bool = False

    @property
    def is_yield(self) -> bool:
        return True


@frozen
class PythonStarExpression(PythonExpression):
    expression: PythonExpression

    @property
    def is_star_expression(self) -> bool:
        return True


@frozen
class PythonAwaitExpression(PythonExpression):
    expression: PythonExpression

    @property
    def is_await(self) -> bool:
        return True
