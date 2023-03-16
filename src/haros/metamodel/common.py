# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Generic, Iterable, List, Mapping, Optional, Self, Set, Type, TypeVar

from collections import defaultdict
from collections.abc import Iterable as IterableType, Mapping as MappingType

from attrs import asdict, define, evolve, field, frozen

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


@frozen
class SourceCodeLocation:
    file: Optional[str] = None
    package: Optional[str] = None
    line: int = 0
    column: int = 0
    language: Optional[str] = None


@frozen
class TrackedCode:
    snippet: Any
    location: SourceCodeLocation

    @classmethod
    def unknown(cls) -> 'TrackedCode':
        return cls(None, SourceCodeLocation())


###############################################################################
# Data Analysis
###############################################################################


def noop(*args, **kwargs):
    pass


T = TypeVar('T')
V = TypeVar('V')
K = TypeVar('K')

BUILTIN_FUNCTION_TYPE = type(min)
DEF_FUNCTION_TYPE = type(noop)
CLASS_TYPE = type(TrackedCode)


@frozen
class TypeToken(Generic[V]):
    token: Type[V]

    @classmethod
    def of(cls, value: V) -> Self:
        return cls(type(value))

    @classmethod
    def of_anything(cls) -> Self:
        return cls(object)

    @classmethod
    def of_bool(cls) -> Self:
        return cls(bool)

    @classmethod
    def of_int(cls) -> Self:
        return cls(int)

    @classmethod
    def of_float(cls) -> Self:
        return cls(float)

    @classmethod
    def of_complex(cls) -> Self:
        return cls(complex)

    @classmethod
    def of_string(cls) -> Self:
        return cls(str)

    @classmethod
    def of_builtin_function(cls) -> Self:
        return cls(BUILTIN_FUNCTION_TYPE)

    @classmethod
    def of_def_function(cls) -> Self:
        return cls(DEF_FUNCTION_TYPE)

    @classmethod
    def of_class(cls) -> Self:
        return cls(CLASS_TYPE)

    @classmethod
    def of_exception(cls) -> Self:
        return cls(Exception)

    @classmethod
    def of_iterable(cls) -> Self:
        return cls(IterableType)

    @classmethod
    def of_list(cls) -> Self:
        return cls(list)

    @classmethod
    def of_tuple(cls) -> Self:
        return cls(tuple)

    @classmethod
    def of_set(cls) -> Self:
        return cls(set)

    @classmethod
    def of_mapping(cls) -> Self:
        return cls(MappingType)

    @classmethod
    def of_dict(cls) -> Self:
        return cls(dict)

    @property
    def is_bool(self) -> bool:
        return issubclass(self.token, bool)

    @property
    def is_int(self) -> bool:
        return issubclass(self.token, int)

    @property
    def is_float(self) -> bool:
        return issubclass(self.token, float)

    @property
    def is_complex(self) -> bool:
        return issubclass(self.token, complex)

    @property
    def is_number(self) -> bool:
        return self.can_be_int or self.can_be_float or self.can_be_complex

    @property
    def is_string(self) -> bool:
        return issubclass(self.token, str)

    @property
    def is_primitive(self) -> bool:
        return self.can_be_bool or self.can_be_number or self.can_be_string

    @property
    def is_function(self) -> bool:
        return issubclass(self.token, (BUILTIN_FUNCTION_TYPE, DEF_FUNCTION_TYPE))

    @property
    def is_class(self) -> bool:
        return issubclass(self.token, CLASS_TYPE)

    @property
    def is_exception(self) -> bool:
        return issubclass(self.token, Exception)

    @property
    def is_definition(self) -> bool:
        return self.can_be_function or self.can_be_class

    @property
    def is_iterable(self) -> bool:
        return issubclass(self.token, IterableType)

    @property
    def is_mapping(self) -> bool:
        return issubclass(self.token, MappingType)

    @property
    def has_items(self) -> bool:
        return self.is_string or self.is_iterable or self.is_mapping

    def __str__(self) -> str:
        return str(self.token)


TYPE_TOKEN_ANYTHING: Final[TypeToken[Any]] = TypeToken.of_anything()
TYPE_TOKEN_BOOL: Final[TypeToken[bool]] = TypeToken.of_bool()
TYPE_TOKEN_INT: Final[TypeToken[int]] = TypeToken.of_int()
TYPE_TOKEN_FLOAT: Final[TypeToken[float]] = TypeToken.of_float()
TYPE_TOKEN_COMPLEX: Final[TypeToken[complex]] = TypeToken.of_complex()
TYPE_TOKEN_STRING: Final[TypeToken[str]] = TypeToken.of_string()
TYPE_TOKEN_LIST: Final[TypeToken[list]] = TypeToken.of_list()
TYPE_TOKEN_TUPLE: Final[TypeToken[tuple]] = TypeToken.of_tuple()
TYPE_TOKEN_SET: Final[TypeToken[set]] = TypeToken.of_set()
TYPE_TOKEN_DICT: Final[TypeToken[dict]] = TypeToken.of_dict()
TYPE_TOKEN_BUILTIN: Final[TypeToken[BUILTIN_FUNCTION_TYPE]] = TypeToken.of_builtin_function()
TYPE_TOKEN_FUNCTION: Final[TypeToken[DEF_FUNCTION_TYPE]] = TypeToken.of_def_function()
TYPE_TOKEN_CLASS: Final[TypeToken[CLASS_TYPE]] = TypeToken.of_class()
TYPE_TOKEN_EXCEPTION: Final[TypeToken[Exception]] = TypeToken.of_exception()


@frozen
class Result(Generic[V]):
    type: TypeToken[V]
    source: Optional[TrackedCode]

    @property
    def is_resolved(self) -> bool:
        return False

    @property
    def is_traceable(self) -> bool:
        return self.source is not None

    def cast_to(self, new_type: TypeToken[V]) -> Self:
        return evolve(self, type=new_type)

    def pretty(self) -> str:
        return f'[{self.type}] {str(self)}'

    # def serialize(self) -> Mapping[str, Any]:
    #     return asdict(self)

    # @classmethod
    # def deserialize(cls, data: Mapping[str, Any]) -> 'SolverResult':
    #     if not isinstance(data, dict):
    #         raise TypeError(f'expected a Mapping, got {data!r}')
    #     value = data['value']
    #     try:
    #         value = UnknownValue.deserialize(value)
    #     except (TypeError, KeyError):
    #         pass
    #     return cls(data['type'], value=value)

    def __str__(self) -> str:
        return '$(?)'


@frozen
class Resolved(Result[V]):
    value: V

    @property
    def is_resolved(self) -> bool:
        return True

    def __str__(self) -> str:
        return str(self.value)


@frozen
class UnresolvedInt(Result[int]):
    min_value: Optional[int] = None
    max_value: Optional[int] = None

    def __str__(self) -> str:
        return '$(? int)'


@frozen
class UnresolvedFloat(Result[float]):
    min_value: Optional[float] = None
    max_value: Optional[float] = None

    def __str__(self) -> str:
        return '$(? float)'


@frozen
class UnresolvedString(Result[str]):
    parts: List[Optional[str]] = field(factory=list)

    def __str__(self) -> str:
        if not self.parts or (len(self.parts) == 1 and self.parts[0] is None):
            return '$(? str)'
        f = lambda s: '$(?)' if s is None else s
        return ''.join(map(f, self.parts))


@frozen
class UnresolvedIterable(Result[Iterable[V]]):
    parts: List[Result[V]] = field(factory=list)

    def __str__(self) -> str:
        if not self.parts:
            return '$(? list)'
        return str(list(map(str, self.parts)))


@frozen
class UnresolvedMapping(Result[Mapping[K, V]]):
    known: Dict[K, Result[V]] = field(factory=dict)

    def __str__(self) -> str:
        return f'$(? map {self.known})'


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
        return cls(base_value=value)

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
        return VariantData(base_value=self._base_value, variants=list(self._variants))

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
