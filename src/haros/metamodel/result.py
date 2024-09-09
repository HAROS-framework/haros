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
import enum

from attrs import asdict, evolve, field, frozen


###############################################################################
# Source Code
###############################################################################


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
    def of(cls, value: V) -> 'TypeToken':
        return cls(type(value))

    @classmethod
    def of_anything(cls) -> 'TypeToken':
        return cls(object)

    @classmethod
    def of_bool(cls) -> 'TypeToken':
        return cls(bool)

    @classmethod
    def of_int(cls) -> 'TypeToken':
        return cls(int)

    @classmethod
    def of_float(cls) -> 'TypeToken':
        return cls(float)

    @classmethod
    def of_complex(cls) -> 'TypeToken':
        return cls(complex)

    @classmethod
    def of_string(cls) -> 'TypeToken':
        return cls(str)

    @classmethod
    def of_builtin_function(cls) -> 'TypeToken':
        return cls(BUILTIN_FUNCTION_TYPE)

    @classmethod
    def of_def_function(cls) -> 'TypeToken':
        return cls(DEF_FUNCTION_TYPE)

    @classmethod
    def of_class(cls) -> 'TypeToken':
        return cls(CLASS_TYPE)

    @classmethod
    def of_exception(cls) -> 'TypeToken':
        return cls(Exception)

    @classmethod
    def of_iterable(cls) -> 'TypeToken':
        return cls(IterableType)

    @classmethod
    def of_list(cls) -> 'TypeToken':
        return cls(list)

    @classmethod
    def of_tuple(cls) -> 'TypeToken':
        return cls(tuple)

    @classmethod
    def of_set(cls) -> 'TypeToken':
        return cls(set)

    @classmethod
    def of_mapping(cls) -> 'TypeToken':
        return cls(MappingType)

    @classmethod
    def of_dict(cls) -> 'TypeToken':
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



class TypeMask(enum.Flag):
    NONE = enum.auto()
    BOOL = enum.auto()
    INT = enum.auto()
    FLOAT = enum.auto()
    COMPLEX = enum.auto()
    STRING = enum.auto()
    FUNCTION = enum.auto()
    CLASS = enum.auto()
    EXCEPTION = enum.auto()
    COLLECTION = enum.auto()
    MAPPING = enum.auto()
    ITERATOR = enum.auto()
    OTHER = enum.auto()
    NUMBER = INT | FLOAT | COMPLEX
    PRIMITIVE = NONE | BOOL | NUMBER | STRING
    DEFINITION = FUNCTION | CLASS
    ITERABLE = STRING | COLLECTION | MAPPING | ITERATOR
    OBJECTS = STRING | DEFINITION | EXCEPTION | ITERABLE | OTHER
    ANY = PRIMITIVE | DEFINITION | OBJECTS

    @classmethod
    def from_value(cls, value: Any) -> 'TypeMask':
        if value is None:
            return cls.NONE
        if isinstance(value, bool):
            return cls.BOOL
        if isinstance(value, int):
            return cls.INT
        if isinstance(value, float):
            return cls.FLOAT
        if isinstance(value, complex):
            return cls.COMPLEX
        if isinstance(value, str):
            return cls.STRING
        if isinstance(value, BaseException):
            return cls.EXCEPTION
        if isinstance(value, type):
            return cls.CLASS
        if isinstance(value, (tuple, list, set)):
            return cls.COLLECTION
        if isinstance(value, dict):
            return cls.MAPPING
        if callable(value):
            return cls.FUNCTION
        d = {}
        g = (type(d.items()), type(d.keys()), type(d.values()))
        if isinstance(value, g):
            return cls.ITERATOR
        return cls.OTHER

    @property
    def can_be_none(self) -> bool:
        return bool(self & TypeMask.NONE)

    @property
    def can_be_bool(self) -> bool:
        return bool(self & TypeMask.BOOL)

    @property
    def can_be_int(self) -> bool:
        return bool(self & TypeMask.INT)

    @property
    def can_be_float(self) -> bool:
        return bool(self & TypeMask.FLOAT)

    @property
    def can_be_complex(self) -> bool:
        return bool(self & TypeMask.COMPLEX)

    @property
    def can_be_number(self) -> bool:
        return bool(self & TypeMask.NUMBER)

    @property
    def can_be_string(self) -> bool:
        return bool(self & TypeMask.STRING)

    @property
    def can_be_primitive(self) -> bool:
        return bool(self & TypeMask.PRIMITIVE)

    @property
    def can_be_function(self) -> bool:
        return bool(self & TypeMask.FUNCTION)

    @property
    def can_be_class(self) -> bool:
        return bool(self & TypeMask.CLASS)

    @property
    def can_be_definition(self) -> bool:
        return bool(self & TypeMask.DEFINITION)

    @property
    def can_be_exception(self) -> bool:
        return bool(self & TypeMask.EXCEPTION)

    @property
    def can_be_collection(self) -> bool:
        return bool(self & TypeMask.COLLECTION)

    @property
    def can_be_mapping(self) -> bool:
        return bool(self & TypeMask.MAPPING)

    @property
    def can_be_iterator(self) -> bool:
        return bool(self & TypeMask.ITERATOR)

    @property
    def can_be_iterable(self) -> bool:
        return bool(self & TypeMask.ITERABLE)

    @property
    def can_be_other_object(self) -> bool:
        return bool(self & TypeMask.OTHER)

    @property
    def can_be_any_object(self) -> bool:
        return bool(self & TypeMask.OBJECTS)

    @property
    def can_be_anything(self) -> bool:
        return bool(self & TypeMask.ANY)

    @property
    def can_have_attributes(self) -> bool:
        return bool(self & ~TypeMask.PRIMITIVE)

    @property
    def can_have_items(self) -> bool:
        mask = TypeMask.STRING | TypeMask.COLLECTION | TypeMask.MAPPING | TypeMask.OTHER
        return bool(self & mask)


###############################################################################
# Unknown Values
###############################################################################


@frozen
class BlackHole:
    def __getattr__(self, _name: str) -> 'BlackHole':
        # could be __getattribute__ instead, if needed
        return self

    def __getitem__(self, _key: Any) -> 'BlackHole':
        return self

    def __call__(self, *args: Any, **kwds: Any) -> 'BlackHole':
        return self

    def __str__(self) -> str:
        return '?'


UNKNOWN_VALUE: Final[BlackHole] = BlackHole()


###############################################################################
# Results
###############################################################################


@frozen
class Result(Generic[V]):
    value: Union[V, TypeMask]
    source: Optional[TrackedCode]

    @property
    def is_resolved(self) -> bool:
        return not isinstance(self.value, TypeMask)

    @property
    def is_traceable(self) -> bool:
        return self.source is not None

    @classmethod
    def unknown_value(cls, source: Optional[TrackedCode] = None) -> 'Result':
        return cls(TypeMask.ANY, source)

    @classmethod
    def unknown_int(cls, source: Optional[TrackedCode] = None) -> 'Result':
        return cls(TypeMask.INT, source)

    @classmethod
    def unknown_float(cls, source: Optional[TrackedCode] = None) -> 'Result':
        return cls(TypeMask.FLOAT, source)

    @classmethod
    def from_complex(cls, value: complex, source: Optional[TrackedCode] = None) -> 'Result':
        return cls(value, source)

    @classmethod
    def from_string(cls, value: str, source: Optional[TrackedCode] = None) -> 'Result':
        return cls(TYPE_TOKEN_STRING, source, value)

    @classmethod
    def from_bool(cls, value: bool, source: Optional[TrackedCode] = None) -> 'Result':
        return cls(TYPE_TOKEN_BOOL, source, value)

    @classmethod
    def from_tuple(cls, value: Tuple[V], source: Optional[TrackedCode] = None) -> 'Result':
        return cls(TYPE_TOKEN_TUPLE, source, value)

    @classmethod
    def from_list(cls, value: List[V], source: Optional[TrackedCode] = None) -> 'Result':
        return cls(TYPE_TOKEN_LIST, source, value)

    @classmethod
    def from_set(cls, value: Set[V], source: Optional[TrackedCode] = None) -> 'Result':
        return cls(TYPE_TOKEN_SET, source, value)

    @classmethod
    def from_dict(cls, value: Dict[K, V], source: Optional[TrackedCode] = None) -> 'Result':
        return cls(TYPE_TOKEN_DICT, source, value)

    @classmethod
    def from_value(cls, value: V, source: Optional[TrackedCode] = None) -> 'Result':
        if value is None:
            return cls.from_none(source=source)
        # if isinstance(value, bool):
        #     return cls.from_bool(value, source=source)
        # if isinstance(value, int):
        #     return cls.from_int(value, source=source)
        # if isinstance(value, float):
        #     return cls.from_float(value, source=source)
        # if isinstance(value, complex):
        #     return cls.from_complex(value, source=source)
        # if isinstance(value, str):
        #     return cls.from_string(value, source=source)
        if isinstance(value, Result):
            raise ValueError('doubly wrapped Result')
        if isinstance(value, BaseException):
            return cls(TYPE_TOKEN_EXCEPTION, source, value)
        if isinstance(value, CLASS_TYPE):
            return cls(TypeToken.of(value), source, value)
        if isinstance(value, BUILTIN_FUNCTION_TYPE):
            return cls(TYPE_TOKEN_FUNCTION, source, value)
        if isinstance(value, DEF_FUNCTION_TYPE):
            return cls(TYPE_TOKEN_FUNCTION, source, value)
        if callable(value):
            return cls(TYPE_TOKEN_FUNCTION, source, value)
        # if isinstance(value, tuple):
        #     return cls.from_tuple(value, source=source)
        # if isinstance(value, list):
        #     return cls.from_list(value, source=source)
        # if isinstance(value, set):
        #     return cls.from_set(value, source=source)
        # if isinstance(value, dict):
        #     return cls.from_dict(value, source=source)
        # d = {}
        # g = (type(d.items()), type(d.keys()), type(d.values()))
        # if isinstance(value, g):
        #     return cls(TYPE_TOKEN_ITERABLE, source, value)
        return cls(TypeToken.of(value), source, value)

    def trace_to(self, source: Optional[TrackedCode]) -> 'Result':
        return evolve(self, source=source)

    def get_attr(self, name: str) -> 'Result':
        try:
            return Result.from_value(getattr(self.value, name))
        except AttributeError:
            return Result.unknown_value()

    def get_item(self, key: Any) -> 'Result':
        try:
            return Result.from_value(self.value[key])
        except KeyError:
            return Result.unknown_value()

    def call(self, *args: Any, **kwds: Any) -> 'Result':
        try:
            return Result.from_value(self.value(*args, **kwds))
        except:
            return Result.unknown_value()

    def __str__(self) -> str:
        return f'$({self.value})'






TupleResult: Final[Type[Result]] = Result[Tuple[Result[V]]]
ListResult: Final[Type[Result]] = Result[List[Result[V]]]


@frozen
class UnresolvedInt(Result[int]):
    min_value: Optional[int] = None
    max_value: Optional[int] = None

    @classmethod
    def unknown_value(
        cls,
        min_value: Optional[int] = None,
        max_value: Optional[int] = None,
        source: Optional[TrackedCode] = None,
    ) -> 'UnresolvedInt':
        return cls(TYPE_TOKEN_INT, source, min_value=min_value, max_value=max_value)

    def __str__(self) -> str:
        return '$(? int)'


@frozen
class UnresolvedFloat(Result[float]):
    min_value: Optional[float] = None
    max_value: Optional[float] = None

    @classmethod
    def unknown_value(
        cls,
        min_value: Optional[float] = None,
        max_value: Optional[float] = None,
        source: Optional[TrackedCode] = None,
    ) -> 'UnresolvedFloat':
        return cls(TYPE_TOKEN_FLOAT, source, min_value=min_value, max_value=max_value)

    def __str__(self) -> str:
        return '$(? float)'


@frozen
class UnresolvedString(Result[str]):
    parts: List[Optional[str]] = field(factory=list)

    @classmethod
    def unknown_value(
        cls,
        parts: Optional[List[Optional[str]]] = None,
        source: Optional[TrackedCode] = None,
    ) -> 'UnresolvedString':
        parts = list(parts) if parts is not None else []
        return cls(TYPE_TOKEN_STRING, source, parts=parts)

    def __str__(self) -> str:
        if not self.parts or (len(self.parts) == 1 and self.parts[0] is None):
            return '$(? str)'
        f = lambda s: '$(?)' if s is None else s
        return ''.join(map(f, self.parts))


@frozen
class UnresolvedIterable(Result[Iterable[V]]):
    parts: List[Result[V]] = field(factory=list)

    @classmethod
    def unknown_value(
        cls,
        parts: Optional[Iterable[Result[V]]] = None,
        source: Optional[TrackedCode] = None,
    ) -> 'UnresolvedIterable':
        parts = list(parts) if parts is not None else []
        return cls(TYPE_TOKEN_ITERABLE, source, parts=parts)

    @classmethod
    def unknown_list(
        cls,
        parts: Optional[Iterable[Result[V]]] = None,
        source: Optional[TrackedCode] = None,
    ) -> 'UnresolvedIterable':
        parts = list(parts) if parts is not None else []
        return cls(TYPE_TOKEN_LIST, source, parts=parts)

    @classmethod
    def unknown_tuple(
        cls,
        parts: Optional[Iterable[Result[V]]] = None,
        source: Optional[TrackedCode] = None,
    ) -> 'UnresolvedIterable':
        parts = list(parts) if parts is not None else []
        return cls(TYPE_TOKEN_TUPLE, source, parts=parts)

    @classmethod
    def unknown_set(
        cls,
        parts: Optional[Iterable[Result[V]]] = None,
        source: Optional[TrackedCode] = None,
    ) -> 'UnresolvedIterable':
        parts = list(parts) if parts is not None else []
        return cls(TYPE_TOKEN_SET, source, parts=parts)

    def __str__(self) -> str:
        if not self.parts:
            return '$(? list)'
        return str(list(map(str, self.parts)))


@frozen
class UnresolvedMapping(Result[Mapping[K, V]]):
    known: Dict[K, Result[V]] = field(factory=dict)

    @classmethod
    def unknown_value(
        cls,
        known: Optional[Mapping[K, Result[V]]] = None,
        source: Optional[TrackedCode] = None,
    ) -> 'UnresolvedMapping':
        known = dict(known) if known is not None else {}
        return cls(TYPE_TOKEN_MAPPING, source, known=known)

    @classmethod
    def unknown_dict(
        cls,
        known: Optional[Mapping[K, Result[V]]] = None,
        source: Optional[TrackedCode] = None,
    ) -> 'UnresolvedMapping':
        known = dict(known) if known is not None else {}
        return cls(TYPE_TOKEN_DICT, source, known=known)

    def __str__(self) -> str:
        return f'$(? map {self.known})'
