# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, List, NewType, Tuple

from enum import Enum, auto

from attrs import field, frozen

###############################################################################
# Constants
###############################################################################

PythonAstNodeId = NewType('PythonAstNodeId', int)


class PythonAstNodeType(Enum):
    MODULE = auto()

    # Python Statement
    STATEMENT = auto()
    EXPRESSION_STMT = auto()
    ASSIGNMENT_STMT = auto()
    DELETE_STMT = auto()
    PASS_STMT = auto()
    BREAK_STMT = auto()
    CONTINUE_STMT = auto()
    RETURN_STMT = auto()
    RAISE_STMT = auto()
    IMPORT_STMT = auto()
    GLOBAL_STMT = auto()
    NONLOCAL_STMT = auto()
    ASSERT_STMT = auto()
    IF_STMT = auto()
    WHILE_STMT = auto()
    FOR_STMT = auto()
    TRY_STMT = auto()
    MATCH_STMT = auto()
    WITH_STMT = auto()
    FUNCTION_DEF = auto()
    CLASS_DEF = auto()

    # Python Expression
    EXPRESSION = auto()
    LITERAL = auto()
    REFERENCE = auto()
    ITEM_ACCESS = auto()
    FUNCTION_CALL = auto()
    STAR_EXPR = auto()
    GENERATOR_EXPR = auto()
    OPERATOR = auto()
    CONDITIONAL_EXPR = auto()
    LAMBDA_EXPR = auto()
    ASSIGNMENT_EXPR = auto()  # Python >= 3.8
    YIELD_EXPR = auto()
    AWAIT_EXPR = auto()

    # Python Helper Node
    HELPER = auto()
    KEY_VALUE_NODE = auto()
    SUBSCRIPT_NODE = auto()
    ITERATOR_NODE = auto()
    ARGUMENT_NODE = auto()
    IMPORT_BASE = auto()
    IMPORTED_NAME = auto()
    FUNCTION_PARAMETER = auto()
    CONDITIONAL_BLOCK = auto()
    EXCEPT_CLAUSE = auto()
    DECORATOR = auto()
    CONTEXT_MANAGER = auto()
    CASE_STATEMENT = auto()
    CASE_PATTERN = auto()

    @property
    def is_module(self) -> bool:
        return self == PythonAstNodeType.MODULE

    @property
    def is_statement(self) -> bool:
        return (
            self == PythonAstNodeType.EXPRESSION_STMT
            or self == PythonAstNodeType.ASSIGNMENT_STMT
            or self == PythonAstNodeType.DELETE_STMT
            or self == PythonAstNodeType.PASS_STMT
            or self == PythonAstNodeType.BREAK_STMT
            or self == PythonAstNodeType.CONTINUE_STMT
            or self == PythonAstNodeType.RETURN_STMT
            or self == PythonAstNodeType.RAISE_STMT
            or self == PythonAstNodeType.IMPORT_STMT
            or self == PythonAstNodeType.GLOBAL_STMT
            or self == PythonAstNodeType.NONLOCAL_STMT
            or self == PythonAstNodeType.ASSERT_STMT
            or self == PythonAstNodeType.IF_STMT
            or self == PythonAstNodeType.WHILE_STMT
            or self == PythonAstNodeType.FOR_STMT
            or self == PythonAstNodeType.TRY_STMT
            or self == PythonAstNodeType.MATCH_STMT
            or self == PythonAstNodeType.WITH_STMT
            or self == PythonAstNodeType.FUNCTION_DEF
            or self == PythonAstNodeType.CLASS_DEF
        )

    @property
    def is_expression(self) -> bool:
        return (
            self == PythonAstNodeType.LITERAL
            or self == PythonAstNodeType.REFERENCE
            or self == PythonAstNodeType.ITEM_ACCESS
            or self == PythonAstNodeType.FUNCTION_CALL
            or self == PythonAstNodeType.STAR_EXPR
            or self == PythonAstNodeType.GENERATOR_EXPR
            or self == PythonAstNodeType.OPERATOR
            or self == PythonAstNodeType.CONDITIONAL_EXPR
            or self == PythonAstNodeType.LAMBDA_EXPR
            or self == PythonAstNodeType.ASSIGNMENT_EXPR
            or self == PythonAstNodeType.YIELD_EXPR
            or self == PythonAstNodeType.AWAIT_EXPR
        )

    @property
    def is_helper(self) -> bool:
        return (
            self == PythonAstNodeType.KEY_VALUE_NODE
            or self == PythonAstNodeType.SUBSCRIPT_NODE
            or self == PythonAstNodeType.ITERATOR_NODE
            or self == PythonAstNodeType.ARGUMENT_NODE
            or self == PythonAstNodeType.IMPORT_BASE
            or self == PythonAstNodeType.IMPORTED_NAME
            or self == PythonAstNodeType.FUNCTION_PARAMETER
            or self == PythonAstNodeType.CONDITIONAL_BLOCK
            or self == PythonAstNodeType.EXCEPT_CLAUSE
            or self == PythonAstNodeType.DECORATOR
            or self == PythonAstNodeType.CONTEXT_MANAGER
            or self == PythonAstNodeType.CASE_STATEMENT
            or self == PythonAstNodeType.CASE_PATTERN
        )


###############################################################################
# AST
###############################################################################


@frozen
class PythonAstNodeMetadata:
    id: PythonAstNodeId
    type: PythonAstNodeType
    line: int = 0
    column: int = 0
    annotations: Dict[str, Any] = field(factory=dict)


class PythonAst:
    # node metadata goes here

    @property
    def ast_node_type(self) -> PythonAstNodeType:
        raise NotImplementedError()

    @property
    def is_module(self) -> bool:
        return self.ast_node_type is PythonAstNodeType.MODULE

    @property
    def is_statement(self) -> bool:
        return self.ast_node_type is PythonAstNodeType.STATEMENT

    @property
    def is_expression(self) -> bool:
        return self.ast_node_type is PythonAstNodeType.EXPRESSION

    @property
    def is_helper(self) -> bool:
        return self.ast_node_type is PythonAstNodeType.HELPER

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        return ws + str(self)


class PythonStatement(PythonAst):
    @property
    def ast_node_type(self) -> PythonAstNodeType:
        return PythonAstNodeType.STATEMENT

    @property
    def is_simple_statement(self) -> bool:
        return (
            self.is_expression_statement
            or self.is_assignment
            or self.is_delete
            or self.is_pass
            or self.is_break
            or self.is_continue
            or self.is_return
            or self.is_raise
            or self.is_yield
            or self.is_global
            or self.is_nonlocal
            or self.is_assert
        )

    @property
    def is_compound_statement(self) -> bool:
        return not self.is_simple_statement

    @property
    def is_expression_statement(self) -> bool:
        return False

    @property
    def is_assignment(self) -> bool:
        return False

    @property
    def is_delete(self) -> bool:
        return False

    @property
    def is_pass(self) -> bool:
        return False

    @property
    def is_break(self) -> bool:
        return False

    @property
    def is_continue(self) -> bool:
        return False

    @property
    def is_return(self) -> bool:
        return False

    @property
    def is_raise(self) -> bool:
        return False

    @property
    def is_yield(self) -> bool:
        return self.is_expression_statement and self.expression.is_yield

    @property
    def is_import(self) -> bool:
        return False

    @property
    def is_global(self) -> bool:
        return False

    @property
    def is_nonlocal(self) -> bool:
        return False

    @property
    def is_assert(self) -> bool:
        return False

    @property
    def is_if(self) -> bool:
        return False

    @property
    def is_while(self) -> bool:
        return False

    @property
    def is_for(self) -> bool:
        return False

    @property
    def is_try(self) -> bool:
        return False

    @property
    def is_match(self) -> bool:
        return False

    @property
    def is_with(self) -> bool:
        return False

    @property
    def is_function_def(self) -> bool:
        return False

    @property
    def is_class_def(self) -> bool:
        return False

    def substatements(self) -> List['PythonStatement']:
        return []


class PythonExpression(PythonAst):
    @property
    def ast_node_type(self) -> PythonAstNodeType:
        return PythonAstNodeType.EXPRESSION

    @property
    def is_literal(self) -> bool:
        return False

    @property
    def is_reference(self) -> bool:
        return False

    @property
    def is_item_access(self) -> bool:
        return False

    @property
    def is_function_call(self) -> bool:
        return False

    @property
    def is_star_expression(self) -> bool:
        return False

    @property
    def is_generator(self) -> bool:
        return False

    @property
    def is_operator(self) -> bool:
        return False

    @property
    def is_conditional(self) -> bool:
        return False

    @property
    def is_lambda(self) -> bool:
        return False

    @property
    def is_assignment(self) -> bool:
        return False  # Python >= 3.8

    @property
    def is_yield(self) -> bool:
        return False

    @property
    def is_await(self) -> bool:
        return False


class PythonHelperNode(PythonAst):
    @property
    def ast_node_type(self) -> PythonAstNodeType:
        return PythonAstNodeType.HELPER

    @property
    def is_key_value(self) -> bool:
        return False

    @property
    def is_subscript(self) -> bool:
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
    def is_imported_name(self) -> bool:
        return False

    @property
    def is_function_parameter(self) -> bool:
        return False

    @property
    def is_conditional_block(self) -> bool:
        return False

    @property
    def is_except_clause(self) -> bool:
        return False

    @property
    def is_decorator(self) -> bool:
        return False

    @property
    def is_context_manager(self) -> bool:
        return False

    @property
    def is_case_statement(self) -> bool:
        return False

    @property
    def is_case_pattern(self) -> bool:
        return False


@frozen
class PythonModule(PythonAst):
    statements: Tuple[PythonStatement]
    name: str = '__main__'

    @property
    def ast_node_type(self) -> PythonAstNodeType:
        return PythonAstNodeType.MODULE
