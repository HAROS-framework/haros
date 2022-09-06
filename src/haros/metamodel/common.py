# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, List, Mapping, Optional, Set, Tuple

from collections import defaultdict

from attrs import define, field, frozen

from haros.metamodel.logic import TRUE, LogicValue

###############################################################################
# File System
###############################################################################


@define
class StorageMetadata:
    path: Optional[str] = None  # real path (e.g. '/home/user/file')
    size: Optional[int] = None  # in bytes
    timestamp: int = 0


###############################################################################
# Source Code
###############################################################################


@define
class SourceCodeMetadata:
    language: str = 'Text'  # e.g. C++, Python...
    lines: int = 1  # always at least 1, even if the file is empty
    ast: Any = None


@define
class DevelopmentMetadata:
    description: str = ''
    authors: Set[str] = field(factory=set)
    maintainers: Set[str] = field(factory=set)
    version: str = 'unknown'
    license: str = 'unknown'
    url_home: Optional[str] = None
    url_source: Optional[str] = None
    url_tracker: Optional[str] = None


@frozen
class SourceCodeDependencies:
    build: Set[str] = field(factory=set)
    runtime: Set[str] = field(factory=set)


###############################################################################
# Conditional Data
###############################################################################


@frozen
class VariantValue:
    value: Any
    condition: LogicValue

    def __str__(self) -> str:
        return f'({self.value} if {self.condition})'


@define
class VariantData:
    _base_value: Optional[Any] = None
    _variants: List[VariantValue] = field(factory=list)

    @property
    def is_deterministic(self) -> bool:
        return not self._variants

    def possible_values(self) -> List[VariantValue]:
        values = list(reversed(self._variants))
        if self._base_value is not None:
            values.append(VariantValue(self._base_value, TRUE))
        return values

    def get(self, strict: bool = True) -> Any:
        if self._variants:
            if strict:
                raise ValueError('multiple possible values')
            return None
        return self._base_value

    def set(self, value: Any, condition: LogicValue):
        if condition.is_true:
            self._base_value = value
            self._variants = []
        elif not condition.is_false:
            self._variants.append(VariantValue(value, condition))

    def serialize(self) -> Mapping[str, Any]:
        return {
            'base': self._base_value,
            'variants': [[v.value, v.condition.serialize()] for v in self._variants]
        }

    @classmethod
    def deserialize(cls, data: Mapping[str, Any]) -> 'VariantData':
        base = data['base']
        variants = [VariantValue(d[0], LogicValue.deserialize(d[1])) for d in data['variants']]
        return cls(base_value=base, variants=variants)

    def __str__(self) -> str:
        values = list(map(str, reversed(self._variants)))
        values.append(str(self._base_value))
        return ' or '.join(values)


def VariantDict(other: Mapping[Any, Any] = None) -> Mapping[Any, VariantData]:
    if other is None:
        return defaultdict(VariantData)
    return defaultdict(VariantData, other)
