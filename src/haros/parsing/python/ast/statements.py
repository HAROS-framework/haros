# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Optional, Tuple

from attrs import field, frozen

from haros.parsing.python.ast.common import PythonExpression, PythonStatement
from haros.parsing.python.ast.helpers import (
    PythonConditionalBlock,
    PythonDecorator,
    PythonFunctionParameter,
    PythonImportedName,
)

###############################################################################
# Simple Statements
###############################################################################


@frozen
class PythonExpressionStatement(PythonStatement):
    expression: PythonExpression
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_expression(self) -> bool:
        return True


@frozen
class PythonImportStatement(PythonStatement):
    names: Tuple[PythonImportedName]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_import(self) -> bool:
        return True


@frozen
class PythonDeleteStatement(PythonStatement):
    expressions: Tuple[PythonExpression]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_delete(self) -> bool:
        return True


###############################################################################
# Class and Function Definitions
###############################################################################


@frozen
class PythonFunctionDefStatement(PythonStatement):
    name: str
    parameters: Tuple[PythonFunctionParameter]
    body: PythonStatement
    type_hint: Optional[str] = None
    is_async: bool = False
    decorators: Tuple[PythonDecorator] = field(factory=tuple)
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


@frozen
class PythonClassDefStatement(PythonStatement):
    decorators: Tuple[PythonDecorator] = field(factory=tuple)
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_class_def(self) -> bool:
        return True


###############################################################################
# Compound Statements
###############################################################################


@frozen
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

    @property
    def line(self) -> int:
        return self.then_branch.line

    @property
    def column(self) -> int:
        return self.then_branch.column
