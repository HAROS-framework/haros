# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Generic, List, Mapping, Optional, Set, Tuple, TypeVar, Union

from collections import defaultdict
from enum import Enum

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
# Data Analysis
###############################################################################

T = TypeVar('T')


class VariableType(Enum):
    BOOL = 'bool'
    INT = 'int'
    DOUBLE = 'double'
    STRING = 'string'
    YAML = 'yaml'
    AUTO = 'auto'
    OBJECT = 'object'


@frozen
class UnknownValue:
    operator: str
    arguments: Tuple[Any]

    def serialize(self) -> Mapping[str, Any]:
        return asdict(self)

    @classmethod
    def deserialize(cls, data: Mapping[str, Any]) -> 'UnknownValue':
        if not isinstance(data, dict):
            raise TypeError(f'expected a Mapping, got {data!r}')
        return cls(data['operator'], tuple(data['arguments']))

    def __str__(self) -> str:
        if not self.arguments:
            return f'$({self.operator})'
        return f'$({self.operator} {" ".join(map(str, self.arguments))})'


@frozen
class SolverResult(Generic[T]):
    type: VariableType
    value: Union[T, UnknownValue]

    @property
    def is_resolved(self) -> bool:
        return not isinstance(self.value, UnknownValue)

    def serialize(self) -> Mapping[str, Any]:
        return asdict(self)

    @classmethod
    def deserialize(cls, data: Mapping[str, Any]) -> 'SolverResult':
        if not isinstance(data, dict):
            raise TypeError(f'expected a Mapping, got {data!r}')
        value = data['value']
        try:
            value = UnknownValue.deserialize(value)
        except (TypeError, KeyError):
            pass
        return cls(data['type'], value)

    def __str__(self) -> str:
        return str(self.value)


###############################################################################
# Conditional Data
###############################################################################


@frozen
class VariantValue(Generic[T]):
    value: T
    condition: LogicValue

    def __str__(self) -> str:
        return f'({self.value} if {self.condition})'


@define
class VariantData(Generic[T]):
    _base_value: Optional[T] = None
    _variants: List[VariantValue[T]] = field(factory=list)

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
        return cls(_base_value=value)

    def possible_values(self) -> List[VariantValue[T]]:
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
        return VariantData(_base_value=self._base_value, _variants=list(self._variants))

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


def variant_dict(other: Mapping[Any, T] = None) -> Mapping[Any, VariantData[T]]:
    d = defaultdict(VariantData)
    if other:
        for key, value in other.items():
            d[key] = VariantData.with_base_value(value)
    return d
