# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from haros.parsing.python.ast.common import (
    PythonAst,
    PythonAstNodeType,
    PythonExpression,
    PythonHelperNode,
    PythonModule,
    PythonStatement,
)
from haros.parsing.python.ast.expressions import (
    PythonAssignmentExpression,
    PythonAwaitExpression,
    PythonBinaryOperator,
    PythonBooleanLiteral,
    PythonConditionalExpression,
    PythonDictComprehension,
    PythonDictLiteral,
    PythonFunctionCall,
    PythonGenerator,
    PythonItemAccess,
    PythonLambdaExpression,
    PythonListComprehension,
    PythonListLiteral,
    PythonLiteral,
    PythonNoneLiteral,
    PythonNumberLiteral,
    PythonOperator,
    PythonReference,
    PythonSetComprehension,
    PythonSetLiteral,
    PythonStarExpression,
    PythonStringLiteral,
    PythonTupleComprehension,
    PythonTupleLiteral,
    PythonUnaryOperator,
    PythonYieldExpression,
)
from haros.parsing.python.ast.helpers import (
    PythonArgument,
    PythonCasePattern,
    PythonCaseStatement,
    PythonClassCasePattern,
    PythonConditionalBlock,
    PythonDecorator,
    PythonDictEntry,
    PythonExceptClause,
    PythonFunctionParameter,
    PythonImportBase,
    PythonImportedName,
    PythonIterator,
    PythonKeyAccess,
    PythonKeyCasePattern,
    PythonKeyValuePair,
    PythonMappingCasePattern,
    PythonNamedCasePattern,
    PythonOrCasePattern,
    PythonSequenceCasePattern,
    PythonSimpleArgument,
    PythonSimpleCasePattern,
    PythonSlice,
    PythonSpecialArgument,
    PythonSubscript,
    PythonWildcardCasePattern,
)
from haros.parsing.python.ast.statements import (
    PythonAssertStatement,
    PythonAssignmentStatement,
    PythonBreakStatement,
    PythonClassDefStatement,
    PythonContextManager,
    PythonContinueStatement,
    PythonDeleteStatement,
    PythonExpressionStatement,
    PythonForStatement,
    PythonFunctionDefStatement,
    PythonIfStatement,
    PythonImportStatement,
    PythonMatchStatement,
    PythonPassStatement,
    PythonRaiseStatement,
    PythonReturnStatement,
    PythonScopeStatement,
    PythonTryStatement,
    PythonWhileStatement,
    PythonWithStatement,
)

###############################################################################
# Utility Functions
###############################################################################


def negate(expr: PythonExpression) -> PythonExpression:
    if not isinstance(expr, PythonAst) or not expr.is_expression:
        raise TypeError(f'expected expression, got: {expr!r}')

    if expr.is_literal:
        if expr.is_none:
            return PythonBooleanLiteral.const_true()
        if expr.is_bool or expr.is_number or expr.is_string:
            if not expr.value:
                return PythonBooleanLiteral.const_true()
            else:
                return PythonBooleanLiteral.const_false()
        if expr.is_tuple or expr.is_list or expr.is_set:
            if not expr.is_comprehension:
                if not expr.values:
                    return PythonBooleanLiteral.const_true()
                else:
                    return PythonBooleanLiteral.const_false()
        if expr.is_dict:
            if not expr.is_comprehension:
                if not expr.entries:
                    return PythonBooleanLiteral.const_true()
                else:
                    return PythonBooleanLiteral.const_false()

    elif expr.is_operator:
        if expr.is_unary and not expr.is_arithmetic:
            assert expr.operator == 'not', f'unknown operator: {expr!r}'
            return expr.operand
        if expr.is_binary and expr.is_comparison:
            return expr.invert()

    elif expr.is_conditional:
        a = negate(expr.expression1)
        b = negate(expr.expression2)
        return PythonConditionalExpression(expr.condition, a, b)

    else:
        return PythonUnaryOperator('not', expr)
