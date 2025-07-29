# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Mapping, Optional, Set, TypeVar

from collections import defaultdict

from attrs import asdict, define, field, frozen

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
    language: str = 'Unknown'  # e.g. C++, Python...
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

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


@frozen
class SourceCodeDependencies:
    build: Set[str] = field(factory=set)
    runtime: Set[str] = field(factory=set)

    def asdict(self) -> dict[str, Any]:
        return {
            'build': list(self.build),
            'runtime': list(self.runtime),
        }


@frozen
class SourceCodeLocation:
    file: Optional[str] = None
    package: Optional[str] = None
    line: int = 0
    column: int = 0
    language: Optional[str] = None

    def serialize(self) -> Mapping[str, Any]:
        return asdict(self)


@frozen
class TrackedCode:
    snippet: Any
    location: SourceCodeLocation

    @classmethod
    def unknown(cls) -> 'TrackedCode':
        return cls(None, SourceCodeLocation())


###############################################################################
# Conditional Data
###############################################################################


T = TypeVar('T')


@frozen
class VariantValue[T]:
    value: T
    condition: LogicValue

    def __str__(self) -> str:
        return f'({self.value} if {self.condition})'


@define
class VariantData[T]:
    _base_value: Optional[T] = None
    _variants: list[VariantValue[T]] = field(factory=list)

    @property
    def has_base_value(self) -> bool:
        return self._base_value is not None

    @property
    def has_values(self) -> bool:
        return self.has_base_value or len(self._variants) > 0

    @property
    def is_deterministic(self) -> bool:
        return not self._variants

    @classmethod
    def with_base_value(cls, value: T) -> 'VariantData':
        return cls(base_value=value)

    def possible_values(self) -> list[VariantValue[T]]:
        values = list(reversed(self._variants))
        if self._base_value is not None:
            values.append(VariantValue(self._base_value, TRUE))
        return values

    def get(self, strict: bool = True) -> T:
        if self._variants:
            if strict:
                raise ValueError('multiple possible values')
            return None
        return self._base_value

    def set(self, value: T, condition: LogicValue):
        if condition.is_true:
            self._base_value = value
            self._variants = []
        elif not condition.is_false:
            self._variants.append(VariantValue(value, condition))

    def duplicate(self) -> 'VariantData':
        return VariantData(base_value=self._base_value, variants=list(self._variants))

    def serialize(self) -> Mapping[str, Any]:
        return {
            'base': self._base_value,
            'variants': [[v.value, v.condition.serialize()] for v in self._variants],
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


def variant_dict(other: Mapping[Any, T] = None) -> Mapping[Any, VariantData[T]]:
    d = defaultdict(VariantData)
    if other:
        for key, value in other.items():
            d[key] = VariantData.with_base_value(value)
    return d
