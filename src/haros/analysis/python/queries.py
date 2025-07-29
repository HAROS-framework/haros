# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Callable, Iterable, Optional, Set, TypeVar

from attrs import define

from haros.parsing.python.ast import (
    PythonExpression,
    PythonFunctionDefStatement,
    PythonImportStatement,
    PythonModule,
    PythonStatement,
)

###############################################################################
# Constants
###############################################################################

T = TypeVar('T')

###############################################################################
# Interface
###############################################################################


@define
class Query[T]:
    matches: Set[T]

    @classmethod
    def q(cls, matches: Iterable[T]) -> 'Query':
        return cls(set(matches))

    def where(self, pred: Callable[[T], bool]) -> 'Query':
        return type(self)(set(filter(pred, self.matches)))

    def __len__(self) -> int:
        return len(self.matches)

    def __iter__(self):
        yield from self.matches


@define
class ImportQuery(Query[PythonImportStatement]):
    def relatives(self) -> 'ImportQuery':
        return self.q(i for i in self.matches if i.base.is_relative)

    def globals(self) -> 'ImportQuery':
        return self.q(i for i in self.matches if i.base.is_global)

    def from_package(self, name: str) -> 'ImportQuery':
        return self.q(i for i in self.matches if i.base.dotted_name == name)


@define
class FunctionQuery(Query[PythonFunctionDefStatement]):
    def named(self, name: str) -> 'FunctionQuery':
        return self.q(f for f in self.matches if f.name == name)

    def typed(self, hint: Optional[PythonExpression] = None) -> 'FunctionQuery':
        if hint is not None:
            return self.q(f for f in self.matches if f.type_hint == hint)
        return self.q(f for f in self.matches if f.type_hint is not None)

    def without_parameters(self) -> 'FunctionQuery':
        return self.q(f for f in self.matches if len(f.parameters) == 0)

    def with_parameters(self) -> 'FunctionQuery':
        return self.q(f for f in self.matches if len(f.parameters) > 0)

    def asynchronous(self) -> 'FunctionQuery':
        return self.q(f for f in self.matches if f.asynchronous)

    def not_asynchronous(self) -> 'FunctionQuery':
        return self.q(f for f in self.matches if not f.asynchronous)

    def decorated(self) -> 'FunctionQuery':
        return self.q(f for f in self.matches if len(f.decorators) > 0)


@define
class ModuleQuery(Query[PythonModule]):
    def functions(self) -> FunctionQuery:
        functions = self._find_statements('is_function_def')
        return FunctionQuery(functions)

    def imports(self) -> ImportQuery:
        imports = self._find_statements('is_import')
        return ImportQuery(imports)

    def _find_statements(self, is_what: str) -> Set[PythonStatement]:
        found = set()
        for module in self.matches:
            stack = list(reversed(module.statements))
            while stack:
                stmt = stack.pop()
                if getattr(stmt, is_what):
                    found.add(stmt)
                elif stmt.is_simple_statement:
                    continue
                elif stmt.is_class_def:
                    continue
                elif stmt.is_function_def:
                    continue
                else:
                    stack.extend(reversed(stmt.substatements()))
        return found
