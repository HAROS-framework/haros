# SPDX-License-Identifier: MIT
# Copyright © 2024 André Santos

###############################################################################
# Imports
###############################################################################

from typing import (
    Any,
    Dict,
    Final,
    Generic,
    Iterable,
    Iterator,
    List,
    Mapping,
    Optional,
    Set,
    Tuple,
    Type,
    TypeVar,
    Union,
)

from collections.abc import Iterable as IterableType, Mapping as MappingType

from attrs import evolve, frozen

from haros.metamodel.common import TrackedCode


###############################################################################
# Data Analysis
###############################################################################


def noop(*args, **kwargs):
    pass


T = TypeVar('T')
V = TypeVar('V')
K = TypeVar('K')

BUILTIN_FUNCTION_TYPE: Final[Type[min]] = type(min)
DEF_FUNCTION_TYPE: Final[Type[noop]] = type(noop)
CLASS_TYPE = type


@frozen
class TypeToken(Generic[V]):
    token: Type[V]

    @classmethod
    def of(cls, value: V) -> 'TypeToken[V]':
        return cls(type(value))

    @classmethod
    def of_anything(cls) -> 'TypeToken[Any]':
        return cls(object)

    @classmethod
    def of_bool(cls) -> 'TypeToken[bool]':
        return cls(bool)

    @classmethod
    def of_int(cls) -> 'TypeToken[int]':
        return cls(int)

    @classmethod
    def of_float(cls) -> 'TypeToken[float]':
        return cls(float)

    @classmethod
    def of_complex(cls) -> 'TypeToken[complex]':
        return cls(complex)

    @classmethod
    def of_string(cls) -> 'TypeToken[str]':
        return cls(str)

    @classmethod
    def of_builtin_function(cls) -> 'TypeToken[Type[min]]':
        return cls(BUILTIN_FUNCTION_TYPE)

    @classmethod
    def of_def_function(cls) -> 'TypeToken[Type[noop]]':
        return cls(DEF_FUNCTION_TYPE)

    @classmethod
    def of_class(cls) -> 'TypeToken[type]':
        return cls(CLASS_TYPE)

    @classmethod
    def of_exception(cls) -> 'TypeToken[BaseException]':
        return cls(Exception)

    @classmethod
    def of_iterable(cls) -> 'TypeToken[IterableType]':
        return cls(IterableType)

    @classmethod
    def of_list(cls) -> 'TypeToken[List[T]]':
        return cls(list)

    @classmethod
    def of_tuple(cls) -> 'TypeToken[Tuple[T]]':
        return cls(tuple)

    @classmethod
    def of_set(cls) -> 'TypeToken[Set[T]]':
        return cls(set)

    @classmethod
    def of_mapping(cls) -> 'TypeToken[MappingType]':
        return cls(MappingType)

    @classmethod
    def of_dict(cls) -> 'TypeToken[Dict[K, T]]':
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
        return self.is_int or self.is_float or self.is_complex

    @property
    def is_string(self) -> bool:
        return issubclass(self.token, str)

    @property
    def is_primitive(self) -> bool:
        return self.is_bool or self.is_number or self.is_string

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
        return self.is_function or self.is_class

    @property
    def is_iterable(self) -> bool:
        return not self.is_string and issubclass(self.token, IterableType)

    @property
    def is_mapping(self) -> bool:
        return issubclass(self.token, MappingType)

    @property
    def has_attributes(self) -> bool:
        return not self.is_bool and not self.is_number

    @property
    def has_items(self) -> bool:
        return self.is_string or self.is_iterable or self.is_mapping

    def __str__(self) -> str:
        return str(getattr(self.token, '__name__', self.token))


TYPE_TOKEN_ANYTHING: Final[TypeToken[Any]] = TypeToken(object)
TYPE_TOKEN_NONE: Final[TypeToken[Any]] = TYPE_TOKEN_ANYTHING
TYPE_TOKEN_BOOL: Final[TypeToken[bool]] = TypeToken(bool)
TYPE_TOKEN_INT: Final[TypeToken[int]] = TypeToken(int)
TYPE_TOKEN_FLOAT: Final[TypeToken[float]] = TypeToken(float)
TYPE_TOKEN_COMPLEX: Final[TypeToken[complex]] = TypeToken(complex)
TYPE_TOKEN_STRING: Final[TypeToken[str]] = TypeToken(str)
TYPE_TOKEN_ITERABLE: Final[TypeToken[Iterable]] = TypeToken(IterableType)
TYPE_TOKEN_LIST: Final[TypeToken[list]] = TypeToken(list)
TYPE_TOKEN_TUPLE: Final[TypeToken[tuple]] = TypeToken(tuple)
TYPE_TOKEN_SET: Final[TypeToken[set]] = TypeToken(set)
TYPE_TOKEN_MAPPING: Final[TypeToken[Mapping]] = TypeToken(MappingType)
TYPE_TOKEN_DICT: Final[TypeToken[dict]] = TypeToken(dict)
TYPE_TOKEN_BUILTIN: Final[TypeToken[Type[min]]] = TypeToken.of(min)
TYPE_TOKEN_FUNCTION: Final[TypeToken[Type[noop]]] = TypeToken.of(noop)
TYPE_TOKEN_CLASS: Final[TypeToken[type]] = TypeToken(type)
TYPE_TOKEN_EXCEPTION: Final[TypeToken[Exception]] = TypeToken(Exception)


###############################################################################
# Unknown Values
###############################################################################


@frozen
class UnknownValue:
    def __str__(self) -> str:
        return '$(?)'


UNKNOWN_VALUE: Final[UnknownValue] = UnknownValue()


@frozen
class BlackHole(UnknownValue):
    def __getattr__(self, _name: str) -> 'BlackHole':
        # could be __getattribute__ instead, if needed
        return self

    def __getitem__(self, _key: Any) -> 'BlackHole':
        return self

    def __call__(self, *args: Any, **kwds: Any) -> 'BlackHole':
        return self


###############################################################################
# Results
###############################################################################


@frozen
class Result(Generic[V]):
    _value: Union[V, UnknownValue]
    type: TypeToken[V]
    source: Optional[TrackedCode]

    @property
    def is_resolved(self) -> bool:
        return not isinstance(self._value, UnknownValue)

    @property
    def is_traceable(self) -> bool:
        return self.source is not None

    @property
    def value(self) -> V:
        if self.is_resolved:
            return self._value
        raise ValueError('unresolved value')

    @property
    def is_callable(self) -> bool:
        return self.is_resolved and callable(self._value)

    @classmethod
    def unknown_value(
        cls,
        of_type: TypeToken[V] = TYPE_TOKEN_ANYTHING,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[V]':
        assert isinstance(of_type, TypeToken)
        return cls(UNKNOWN_VALUE, of_type, source)

    @classmethod
    def of(cls, value: V, source: Optional[TrackedCode] = None) -> 'Result[V]':
        assert not isinstance(value, Result), 'doubly wrapped Result'
        if value is None:
            return cls.of_none(source=source)
        if isinstance(value, bool):
            return cls.of_bool(value=value, source=source)
        if isinstance(value, int):
            return cls.of_int(value=value, source=source)
        if isinstance(value, float):
            return cls.of_float(value=value, source=source)
        if isinstance(value, complex):
            return cls.of_complex(value=value, source=source)
        if isinstance(value, str):
            return cls.of_string(value=value, source=source)
        if isinstance(value, tuple):
            return cls.of_tuple(value=value, source=source)
        if isinstance(value, list):
            return cls.of_list(value=value, source=source)
        if isinstance(value, set):
            return cls.of_set(value=value, source=source)
        if isinstance(value, dict):
            return cls.of_dict(value=value, source=source)
        if isinstance(value, BaseException):
            return cls.of_exception(value=value, source=source)
        if isinstance(value, CLASS_TYPE):
            return cls.of_class(value=value, source=source)
        if isinstance(value, BUILTIN_FUNCTION_TYPE):
            return cls.of_builtin_function(value=value, source=source)
        if isinstance(value, DEF_FUNCTION_TYPE):
            return cls.of_def_function(value=value, source=source)
        d = {}
        g = (type(d.items()), type(d.keys()), type(d.values()))
        if isinstance(value, g):
            return cls.of_iterable(value=value, source=source)
        return cls(value, TypeToken.of(value), source)

    @classmethod
    def of_none(cls, source: Optional[TrackedCode] = None) -> 'Result[None]':
        return cls(None, TYPE_TOKEN_NONE, source)

    @classmethod
    def of_bool(
        cls,
        value: Union[bool, UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[bool]':
        assert isinstance(value, (bool, UnknownValue))
        return cls(value, TYPE_TOKEN_BOOL, source)

    @classmethod
    def of_int(
        cls,
        value: Union[int, UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[int]':
        assert isinstance(value, (int, UnknownValue))
        return cls(value, TYPE_TOKEN_INT, source)

    @classmethod
    def of_float(
        cls,
        value: Union[float, UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[float]':
        assert isinstance(value, (float, UnknownValue))
        return cls(value, TYPE_TOKEN_FLOAT, source)

    @classmethod
    def of_complex(
        cls,
        value: Union[complex, UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[complex]':
        assert isinstance(value, (complex, UnknownValue))
        return cls(value, TYPE_TOKEN_COMPLEX, source)

    @classmethod
    def of_number(
        cls,
        value: Union[int, float, complex, UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[Union[int, float, complex]]':
        if isinstance(value, int):
            return cls.of_int(value=value, source=source)
        if isinstance(value, float):
            return cls.of_float(value=value, source=source)
        if isinstance(value, complex):
            return cls.of_complex(value=value, source=source)
        assert isinstance(value, UnknownValue), repr(value)
        return cls.of_float(value=value, source=source)

    @classmethod
    def of_string(
        cls,
        value: Union[str, UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[str]':
        assert isinstance(value, (str, UnknownValue))
        return cls(value, TYPE_TOKEN_STRING, source)

    @classmethod
    def of_tuple(
        cls,
        value: Union[Tuple[T], UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[Tuple[T]]':
        assert isinstance(value, (tuple, UnknownValue))
        return cls(value, TYPE_TOKEN_TUPLE, source)

    @classmethod
    def of_list(
        cls,
        value: Union[List[T], UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[List[T]]':
        assert isinstance(value, (list, UnknownValue))
        return cls(value, TYPE_TOKEN_LIST, source)

    @classmethod
    def of_set(
        cls,
        value: Union[Set[T], UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[Set[T]]':
        assert isinstance(value, (set, UnknownValue))
        return cls(value, TYPE_TOKEN_SET, source)

    @classmethod
    def of_dict(
        cls,
        value: Union[Dict[K, T], UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[Dict[K, T]]':
        assert isinstance(value, (dict, UnknownValue))
        return cls(value, TYPE_TOKEN_DICT, source)

    @classmethod
    def of_iterable(
        cls,
        value: Union[IterableType[T], UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[IterableType[T]]':
        assert isinstance(value, (IterableType, UnknownValue))
        return cls(value, TYPE_TOKEN_ITERABLE, source)

    @classmethod
    def of_mapping(
        cls,
        value: Union[MappingType[K, T], UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[MappingType[K, T]]':
        assert isinstance(value, (MappingType, UnknownValue))
        return cls(value, TYPE_TOKEN_MAPPING, source)

    @classmethod
    def of_builtin_function(
        cls,
        value: Union[Type[min], UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[Type[min]]':
        assert callable(value) or isinstance(value, UnknownValue), repr(value)
        return cls(value, TYPE_TOKEN_BUILTIN, source)

    @classmethod
    def of_def_function(
        cls,
        value: Union[Type[noop], UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[Type[noop]]':
        assert callable(value) or isinstance(value, UnknownValue)
        return cls(value, TYPE_TOKEN_FUNCTION, source)

    @classmethod
    def of_class(
        cls,
        value: Union[Type[T], UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[Type[T]]':
        assert callable(value) or isinstance(value, UnknownValue), repr(value)
        return cls(value, TYPE_TOKEN_CLASS, source)

    @classmethod
    def of_instance(
        cls,
        constructor: Type[T],
        source: Optional[TrackedCode] = None,
    ) -> 'Result[T]':
        assert isinstance(constructor, CLASS_TYPE)
        return cls(UNKNOWN_VALUE, TypeToken(constructor), source)

    @classmethod
    def of_exception(
        cls,
        value: Union[BaseException, UnknownValue] = UNKNOWN_VALUE,
        source: Optional[TrackedCode] = None,
    ) -> 'Result[BaseException]':
        assert isinstance(value, (BaseException, UnknownValue))
        return cls(value, TYPE_TOKEN_EXCEPTION, source)

    def cast_to(self, new_type: TypeToken[T]) -> 'Result[T]':
        return evolve(self, type=new_type)

    def trace_to(self, source: Optional[TrackedCode]) -> 'Result[V]':
        return evolve(self, source=source)

    def get_attr(self, name: str, source: Optional[TrackedCode] = None) -> 'Result[Any]':
        if not self.is_resolved:
            return Result.unknown_value(source=source)
        try:
            return Result.of(getattr(self._value, name), source=source)
        except AttributeError:
            return Result.unknown_value(source=source)

    def get_item(self, key: Any, source: Optional[TrackedCode] = None) -> 'Result[Any]':
        if not self.is_resolved:
            return Result.unknown_value(source=source)
        try:
            return Result.of(self._value[key], source=source)
        except KeyError:
            return Result.unknown_value(source=source)

    def items(self) -> Iterable['Result[Any]']:
        if not self.is_resolved:
            raise ValueError('cannot iterate an unresolved result')
        assert isinstance(self._value, Iterable)
        return IterableWrapper(self._value)

    def call(self, *args: Any, **kwds: Any) -> 'Result[Any]':
        if not self.is_resolved:
            return Result.unknown_value()
        try:
            return Result.of(self._value(*args, **kwds))
        except:
            return Result.unknown_value()

    def __str__(self) -> str:
        return str(self._value)


###############################################################################
# Value Wrappers
###############################################################################


@frozen
class IterableWrapper(Generic[V]):
    value: Iterable[V]

    def __getattr__(self, name: str) -> Any:
        return getattr(self.value, name)

    def __getitem__(self, key: Any) -> Result[V]:
        return Result.of(self.value[key])

    def __iter__(self) -> Iterator[Result[V]]:
        for item in self.value:
            yield Result.of(item)

    def __str__(self) -> str:
        return str(self.value)


@frozen
class MappingWrapper(Generic[K, V]):
    value: Mapping[K, V]

    def __getattr__(self, name: str) -> Any:
        return getattr(self.value, name)

    def __getitem__(self, key: K) -> Result[V]:
        return Result(self.value[key], None)

    def __str__(self) -> str:
        return str(self.value)


###############################################################################
# Partial Results
###############################################################################


@frozen
class UnresolvedString(UnknownValue):
    parts: Iterable[Result[str]]

    def __str__(self) -> str:
        if not self.parts:
            return '$(? str)'
        return ''.join(map(str, self.parts))
