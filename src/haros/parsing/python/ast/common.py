# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

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

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        return ws + str(self)


class PythonStatement(PythonAst):
    @property
    def is_statement(self) -> bool:
        return True

    @property
    def is_simple_statement(self) -> bool:
        return (
            self.is_assignment
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
        return False

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

    @property
    def is_conditional(self) -> bool:
        return False

    @property
    def is_lambda(self) -> bool:
        return False

    @property
    def is_assignment(self) -> bool:
        return False  # Python >= 3.8


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

    @property
    def is_decorator(self) -> bool:
        return False