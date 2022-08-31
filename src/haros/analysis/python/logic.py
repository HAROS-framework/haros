# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Optional

from haros.metamodel.logic import (
    FALSE,
    TRUE,
    LogicAnd,
    LogicOr,
    LogicValue,
    LogicVariable,
)
from haros.parsing.python.ast import PythonAst, PythonExpression

###############################################################################
# Logic
###############################################################################


def to_condition(expr: PythonExpression) -> LogicValue:
    if not isinstance(expr, PythonAst) or not expr.is_expression:
        raise TypeError(f'expected expression, got: {expr!r}')

    if expr.is_literal:
        if expr.is_none:
            return FALSE
        if expr.is_bool or expr.is_number or expr.is_string:
            return LogicValue.deserialize(bool(expr.value))
        if expr.is_tuple or expr.is_list or expr.is_set:
            if not expr.is_comprehension:
                return LogicValue.deserialize(len(expr.values) > 0)
        if expr.is_dict:
            if not expr.is_comprehension:
                return LogicValue.deserialize(len(expr.entries) > 0)

    elif expr.is_operator:
        if expr.is_unary:
            value = to_condition(expr.operand)
            if expr.operator.is_logic or expr.operator.is_bitwise:
                return value.negate()
            else:
                assert expr.operator in ('+', '-'), f'unknown operator: {expr!r}'
                return value

        if expr.is_binary:
            if expr.is_logic:
                phi = to_condition(expr.operand1)
                psi = to_condition(expr.operand2)
                if expr.operator == 'and':
                    return LogicAnd((phi, psi)).simplify()
                else:
                    assert expr.operator == 'or', f'unknown operator: {expr!r}'
                    return LogicOr((phi, psi)).simplify()

            elif expr.is_comparison:
                result = _compare(expr)
                if value is not None:
                    return TRUE if result else FALSE

            else:
                assert expr.is_arithmetic, f'unknown operator: {expr!r}'
                value = _solve_number(expr)
                if value is not None:
                    return FALSE if not value else TRUE

    elif expr.is_conditional:
        rho = to_condition(expr.condition)
        phi = to_condition(expr.expression1)
        psi = to_condition(expr.expression2)
        if rho.is_true:
            return phi
        elif rho.is_false:
            return psi
        else:
            phi = rho.implies(phi)
            psi = rho.negate().implies(psi)
            return phi.join(psi)

    else:
        return LogicVariable(expr)


def _compare(expr: PythonExpression) -> Optional[bool]:
    a = _solve_number(expr.operand1)
    b = _solve_number(expr.operand2)
    if a is None or b is None:
        return None

    if expr.operator == '==':
        return a == b
    if expr.operator == '!=':
        return a != b
    if expr.operator == '<':
        return a < b
    if expr.operator == '>':
        return a > b
    if expr.operator == '<=':
        return a <= b
    if expr.operator == '>=':
        return a >= b

    return None


def _solve_number(expr: PythonExpression) -> Optional[int]:
    if expr.is_literal:
        if expr.is_number:
            return expr.value
        if expr.is_bool:
            return int(value)
        return None

    if expr.is_operator:
        if expr.is_unary:
            value = _solve_number(expr.operand)
            if value is None:
                return None
            if expr.operator == '~':
                return ~value
            if expr.operator == '+':
                return value
            if expr.operator == '-':
                return -value

        if expr.is_binary:
            a = _solve_number(expr.operand1)
            b = _solve_number(expr.operand2)
            if a is None or b is None:
                return None
            if expr.operator == '+':
                return a + b
            if expr.operator == '-':
                return a - b
            if expr.operator == '*':
                return a * b
            if expr.operator == '/':
                return a / b
            if expr.operator == '//':
                return a // b
            if expr.operator == '**':
                return a ** b

    return None
