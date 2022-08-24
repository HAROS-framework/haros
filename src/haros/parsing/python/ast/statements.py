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
    PythonContextManager,
    PythonDecorator,
    PythonExceptClause,
    PythonFunctionParameter,
    PythonImportedName,
    PythonIterator,
)

###############################################################################
# Simple Statements
###############################################################################


@frozen
class PythonPassStatement(PythonStatement):
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_pass(self) -> bool:
        return True


@frozen
class PythonBreakStatement(PythonStatement):
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_break(self) -> bool:
        return True


@frozen
class PythonContinueStatement(PythonStatement):
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_continue(self) -> bool:
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


@frozen
class PythonReturnStatement(PythonStatement):
    value: Optional[PythonExpression]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_return(self) -> bool:
        return True


@frozen
class PythonRaiseStatement(PythonStatement):
    exception: Optional[PythonExpression]
    cause: Optional[PythonExpression]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_raise(self) -> bool:
        return True


@frozen
class PythonExpressionStatement(PythonStatement):
    expression: PythonExpression

    @property
    def is_expression_statement(self) -> bool:
        return True

    @property
    def line(self) -> int:
        return self.expression.line

    @property
    def column(self) -> int:
        return self.expression.column


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
class PythonAssignmentStatement(PythonStatement):
    variable: PythonExpression
    value: PythonExpression
    operator: str = '='
    type_hint: Optional[PythonExpression] = None
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_assignment(self) -> bool:
        return True

    @property
    def is_standard(self) -> bool:
        return self.operator == '='

    @property
    def is_augmented(self) -> bool:
        return not self.is_standard

    @property
    def is_annotated(self) -> bool:
        return self.type_hint is not None

    @property
    def is_unpacked(self) -> bool:
        return self.variable.is_tuple

    @property
    def is_packed(self) -> bool:
        return self.variable.is_star_expression


@frozen
class PythonScopeStatement(PythonStatement):
    names: Tuple[str]
    global_scope: bool = True
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_global(self) -> bool:
        return self.global_scope

    @property
    def is_nonlocal(self) -> bool:
        return not self.global_scope


@frozen
class PythonAssertStatement(PythonStatement):
    condition: PythonExpression
    message: Optional[PythonExpression]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_assert(self) -> bool:
        return True


###############################################################################
# Class and Function Definitions
###############################################################################


@frozen
class PythonFunctionDefStatement(PythonStatement):
    name: str
    parameters: Tuple[PythonFunctionParameter]
    body: Tuple[PythonStatement]
    type_hint: Optional[PythonExpression] = None
    asynchronous: bool = False
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
    name: str
    body: Tuple[PythonStatement]
    arguments: Tuple[PythonArgument] = field(factory=tuple)
    decorators: Tuple[PythonDecorator] = field(factory=tuple)
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_class_def(self) -> bool:
        return True

    @property
    def has_arguments(self) -> bool:
        return len(self.arguments) > 0

    @property
    def is_decorated(self) -> bool:
        return len(self.decorators) > 0


###############################################################################
# Compound Statements
###############################################################################


@frozen
class PythonIfStatement(PythonStatement):
    then_branch: PythonConditionalBlock
    elif_branches: Tuple[PythonConditionalBlock]
    else_branch: Tuple[PythonStatement]

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

    @property
    def has_else_branch(self) -> bool:
        return len(self.else_branch) > 0


@frozen
class PythonWhileStatement(PythonStatement):
    loop: PythonConditionalBlock
    else_branch: Tuple[PythonStatement]

    @property
    def is_while(self) -> bool:
        return True

    @property
    def condition(self) -> PythonExpression:
        return self.loop.condition

    @property
    def body(self) -> Tuple[PythonStatement]:
        return self.loop.body

    @property
    def line(self) -> int:
        return self.loop.line

    @property
    def column(self) -> int:
        return self.loop.column

    @property
    def has_else_branch(self) -> bool:
        return len(self.else_branch) > 0


@frozen
class PythonForStatement(PythonStatement):
    iterator: PythonIterator
    body: Tuple[PythonStatement]
    else_branch: Tuple[PythonStatement]

    @property
    def is_for(self) -> bool:
        return True

    @property
    def variables(self) -> Tuple[PythonExpression]:
        return self.iterator.variables

    @property
    def iterable(self) -> PythonExpression:
        return self.iterator.iterable

    @property
    def asynchronous(self) -> bool:
        return self.iterator.asynchronous

    @property
    def line(self) -> int:
        return self.iterator.variables[0].line

    @property
    def column(self) -> int:
        return self.iterator.variables[0].column

    @property
    def has_else_branch(self) -> bool:
        return len(self.else_branch) > 0


@frozen
class PythonTryStatement(PythonStatement):
    body: Tuple[PythonStatement]
    except_clauses: Tuple[PythonExceptClause] = field(factory=tuple)
    else_branch: Tuple[PythonStatement] = field(factory=tuple)
    finally_block: Tuple[PythonStatement] = field(factory=tuple)
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_try(self) -> bool:
        return True

    @property
    def is_try_finally(self) -> bool:
        return not self.has_except_clauses

    @property
    def has_except_clauses(self) -> bool:
        return len(self.except_clauses) > 0

    @property
    def has_else_branch(self) -> bool:
        return len(self.else_branch) > 0

    @property
    def has_finally_block(self) -> bool:
        return len(self.finally_block) > 0


@frozen
class PythonWithStatement(PythonStatement):
    managers: Tuple[PythonContextManager]
    body: Tuple[PythonStatement]
    asynchronous: bool = False
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_with(self) -> bool:
        return True
