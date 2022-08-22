# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Iterable, Optional, Tuple, Union

from attrs import frozen

from haros.parsing.python.ast.common import PythonExpression, PythonHelperNode, PythonStatement

###############################################################################
# Mapping Helpers
###############################################################################


@frozen
class PythonKeyValuePair(PythonHelperNode):
    key: PythonExpression
    value: PythonExpression
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_key_value(self) -> bool:
        return True

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        key = self.key.pretty(indent=(indent+step))
        value = self.value.pretty(indent=(indent+step))
        return f'{ws}Key Value\n{key}\n{value}'


PythonDictEntry = Union[PythonKeyValuePair, PythonExpression]


###############################################################################
# Generator Helpers
###############################################################################


@frozen
class PythonIterator(PythonHelperNode):
    variables: Tuple[PythonExpression]
    iterable: PythonExpression
    asynchronous: bool = False

    @property
    def is_iterator(self):
        return True

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws1 = ' ' * indent
        ws2 = ' ' * (indent + step)
        vs = '\n'.join(v.pretty(indent=(indent+step+step)) for v in self.variables)
        it = self.iterable.pretty(indent=(indent+step))
        asynchronous = f'{ws2}async\n' if self.asynchronous else ''
        return f'{ws1}Iterator\n{asynchronous}{ws2}variables\n{vs}\niterable\n{it}'


###############################################################################
# Arguments
###############################################################################


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


@frozen
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


@frozen
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


###############################################################################
# Import Helpers
###############################################################################


@frozen
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


@frozen
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


@frozen
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


###############################################################################
# Class and Function Helpers
###############################################################################


@frozen
class PythonDecorator(PythonHelperNode):
    names: Tuple[str]
    arguments: Tuple[PythonArgument]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_decorator(self) -> bool:
        return True

    @property
    def dotted_name(self) -> str:
        return ('.' * self.dots) + '.'.join(self.names)


@frozen
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


###############################################################################
# Compound Statement Helpers
###############################################################################


@frozen
class PythonConditionalBlock(PythonHelperNode):
    condition: PythonExpression
    body: Tuple[PythonStatement]
    # meta
    line: int = 0
    column: int = 0

    @property
    def is_conditional_block(self) -> bool:
        return True
