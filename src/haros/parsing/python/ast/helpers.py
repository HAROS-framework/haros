# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Optional, Union

from collections.abc import Sequence

from attrs import field, frozen

from haros.parsing.python.ast.common import PythonExpression, PythonHelperNode, PythonStatement

###############################################################################
# Mapping Helpers
###############################################################################


@frozen
class PythonKeyValuePair(PythonHelperNode):
    key: PythonExpression
    value: PythonExpression

    @property
    def is_key_value(self) -> bool:
        return True

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws = ' ' * indent
        key = self.key.pretty(indent=(indent + step))
        value = self.value.pretty(indent=(indent + step))
        return f'{ws}Key Value\n{key}\n{value}'


PythonDictEntry = Union[PythonKeyValuePair, PythonExpression]

###############################################################################
# Subscript Helpers
###############################################################################


@frozen
class PythonSubscript(PythonHelperNode):
    @property
    def is_subscript(self) -> bool:
        return True

    @property
    def is_key(self) -> bool:
        return False

    @property
    def is_slice(self) -> bool:
        return False


@frozen
class PythonKeyAccess(PythonSubscript):
    expression: PythonExpression

    # @property
    # def line(self) -> int:
    #     return self.expression.meta.line

    # @property
    # def column(self) -> int:
    #     return self.expression.meta.column

    @property
    def is_key(self) -> bool:
        return True


@frozen
class PythonSlice(PythonSubscript):
    start: Optional[PythonExpression] = None
    end: Optional[PythonExpression] = None
    step: Optional[PythonExpression] = None

    @property
    def is_slice(self) -> bool:
        return True

    @property
    def is_full_slice(self) -> bool:
        return self.start is None and self.end is None and self.step is None


###############################################################################
# Generator Helpers
###############################################################################


@frozen
class PythonIterator(PythonHelperNode):
    variables: Sequence[PythonExpression]
    iterable: PythonExpression
    asynchronous: bool = False

    @property
    def is_iterator(self) -> bool:
        return True

    def pretty(self, indent: int = 0, step: int = 2) -> str:
        ws1 = ' ' * indent
        ws2 = ' ' * (indent + step)
        vs = '\n'.join(v.pretty(indent=(indent + step + step)) for v in self.variables)
        it = self.iterable.pretty(indent=(indent + step))
        asynchronous = f'{ws2}async\n' if self.asynchronous else ''
        return f'{ws1}Iterator\n{asynchronous}{ws2}variables\n{vs}\niterable\n{it}'


###############################################################################
# Arguments
###############################################################################


@frozen
class PythonArgument(PythonHelperNode):
    value: PythonExpression
    name: str = ''

    @property
    def is_argument(self) -> bool:
        return True

    @property
    def is_positional(self) -> bool:
        return not self.name

    @property
    def is_key_value(self) -> bool:
        return self.name and not self.is_star and not self.is_double_star

    @property
    def is_star(self) -> bool:
        return self.name == '*'

    @property
    def is_double_star(self) -> bool:
        return self.name == '**'

    @property
    def is_keyword(self) -> bool:
        return self.name == '**'


###############################################################################
# Import Helpers
###############################################################################


@frozen
class PythonImportBase(PythonHelperNode):
    names: Sequence[str]
    dots: int = 0

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

    # YAGNI
    # def add_dots(self, dots: int, line: int = 0, column: int = 0) -> 'PythonImportBase':
    #     return PythonImportBase(
    #         self.names,
    #         dots=(self.dots + dots),
    #         line=(line or self.line),
    #         column=(column or self.column),
    #     )

    # YAGNI
    # def append(self, names: Iterable[str]) -> 'PythonImportBase':
    #     return PythonImportBase(
    #         self.names + names,
    #         dots=self.dots,
    #         line=self.line,
    #         column=self.column,
    #     )


@frozen
class PythonImportedName(PythonHelperNode):
    base: PythonImportBase
    name: str = field()
    alias: Optional[str] = None

    @name.validator
    def _check_name(self, attribute, value):
        if not isinstance(value, str):
            raise TypeError(f'expected string "name", got: {value!r}')

    @property
    def is_imported_name(self) -> bool:
        return True

    # @property
    # def line(self) -> int:
    #     return self.base.line

    # @property
    # def column(self) -> int:
    #     return self.base.column

    @property
    def is_wildcard(self) -> bool:
        return self.name == '*'


###############################################################################
# Class and Function Helpers
###############################################################################


@frozen
class PythonDecorator(PythonHelperNode):
    names: Sequence[str]
    arguments: Sequence[PythonArgument]

    @property
    def is_decorator(self) -> bool:
        return True

    @property
    def dotted_name(self) -> str:
        return '.'.join(self.names)


@frozen
class PythonFunctionParameter(PythonHelperNode):
    name: str
    default_value: Optional[PythonExpression] = None
    type_hint: Optional[str] = None
    modifier: str = ''

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
    body: Sequence[PythonStatement]

    @property
    def is_conditional_block(self) -> bool:
        return True


@frozen
class PythonExceptClause(PythonHelperNode):
    body: Sequence[PythonStatement]
    exception: Optional[PythonExpression] = None
    alias: Optional[str] = None

    @property
    def is_except_clause(self) -> bool:
        return True


@frozen
class PythonContextManager(PythonHelperNode):
    manager: PythonExpression
    alias: Optional[str] = None

    @property
    def is_context_manager(self) -> bool:
        return True

    # @property
    # def line(self) -> int:
    #     return self.manager.line

    # @property
    # def column(self) -> int:
    #     return self.manager.column


@frozen
class PythonCasePattern(PythonHelperNode):
    @property
    def is_case_pattern(self) -> bool:
        return True

    @property
    def is_named_pattern(self) -> bool:
        return False

    @property
    def is_wildcard_pattern(self) -> bool:
        return False

    @property
    def is_simple_pattern(self) -> bool:
        return False

    @property
    def is_key_pattern(self) -> bool:
        return False

    @property
    def is_class_pattern(self) -> bool:
        return False

    @property
    def is_or_pattern(self) -> bool:
        return False

    @property
    def is_sequence_pattern(self) -> bool:
        return False

    @property
    def is_mapping_pattern(self) -> bool:
        return False


@frozen
class PythonNamedCasePattern(PythonHelperNode):
    name: str
    pattern: PythonCasePattern

    @property
    def is_named_pattern(self) -> bool:
        return True


@frozen
class PythonWildcardCasePattern(PythonHelperNode):
    # captures `_`, `*_`, `*name`, `**name`
    name: Optional[str] = None
    is_star_pattern: bool = False

    @property
    def is_wildcard_pattern(self) -> bool:
        return True


@frozen
class PythonSimpleCasePattern(PythonHelperNode):
    expression: PythonExpression
    alias: Optional[str] = None

    # @property
    # def line(self) -> int:
    #     return self.expression.line

    # @property
    # def column(self) -> int:
    #     return self.expression.column

    @property
    def is_simple_pattern(self) -> bool:
        return True

    @property
    def is_variable_binding(self) -> bool:
        return self.expression.is_reference


@frozen
class PythonKeyCasePattern(PythonHelperNode):
    key: PythonExpression
    pattern: PythonCasePattern

    # @property
    # def line(self) -> int:
    #     return self.key.line

    # @property
    # def column(self) -> int:
    #     return self.key.column

    @property
    def is_key_pattern(self) -> bool:
        return True


@frozen
class PythonClassCasePattern(PythonHelperNode):
    type_reference: PythonExpression
    arguments: Sequence[PythonCasePattern]
    alias: Optional[str] = None

    # @property
    # def line(self) -> int:
    #     return self.type_reference.line

    # @property
    # def column(self) -> int:
    #     return self.type_reference.column

    @property
    def is_class_pattern(self) -> bool:
        return True

    @property
    def positional_arguments(self) -> Sequence[PythonSimpleCasePattern]:
        return tuple(p for p in self.arguments if not p.is_named_pattern)

    @property
    def keyword_arguments(self) -> Sequence[PythonKeyCasePattern]:
        return tuple(p for p in self.arguments if p.is_named_pattern)


@frozen
class PythonOrCasePattern(PythonHelperNode):
    patterns: Sequence[PythonCasePattern]
    alias: Optional[str] = None

    # @property
    # def line(self) -> int:
    #     return self.patterns[0].line

    # @property
    # def column(self) -> int:
    #     return self.patterns[0].column

    @property
    def is_or_pattern(self) -> bool:
        return True

    def __len__(self) -> int:
        return len(self.patterns)

    def __getitem__(self, key: Any) -> PythonCasePattern:
        return self.patterns[key]


@frozen
class PythonSequenceCasePattern(PythonHelperNode):
    patterns: Sequence[PythonCasePattern]

    @property
    def is_sequence_pattern(self) -> bool:
        return True

    def __len__(self) -> int:
        return len(self.patterns)

    def __getitem__(self, key: Any) -> PythonCasePattern:
        return self.patterns[key]


@frozen
class PythonMappingCasePattern(PythonHelperNode):
    patterns: Sequence[PythonKeyCasePattern]

    @property
    def is_mapping_pattern(self) -> bool:
        return True

    def get(self, key: str, default: Any = None) -> Optional[PythonKeyCasePattern]:
        for pattern in self.patterns:
            if pattern.key == key:
                return pattern
        return default

    def __len__(self) -> int:
        return len(self.patterns)

    def __getitem__(self, key: Any) -> PythonKeyCasePattern:
        pattern = self.get(key)
        if pattern is None:
            raise KeyError(key)
        return pattern


@frozen
class PythonCaseStatement(PythonHelperNode):
    pattern: PythonCasePattern
    body: Sequence[PythonStatement]
    condition: Optional[PythonExpression] = None

    @property
    def is_case_statement(self) -> bool:
        return True

    @property
    def guard(self) -> Optional[PythonExpression]:
        return self.condition
