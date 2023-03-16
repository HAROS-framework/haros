# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Dict, Final, List, Optional, Self, Set, Tuple, Type

import enum

from attrs import define, evolve, field, frozen

from haros.metamodel.common import (
    BUILTIN_FUNCTION_TYPE,
    CLASS_TYPE,
    DEF_FUNCTION_TYPE,
    IterableType,
    K,
    MappingType,
    Resolved,
    Result,
    TrackedCode,
    TypeToken,
    UnresolvedFloat,
    UnresolvedInt,
    UnresolvedIterable,
    UnresolvedMapping,
    UnresolvedString,
    V,
    VariantData,
)
from haros.metamodel.logic import FALSE, TRUE, LogicValue, LogicVariable
from haros.parsing.python.ast import (
    PythonAssignmentStatement,
    PythonAst,
    PythonBinaryOperator,
    PythonClassDefStatement,
    PythonExpression,
    PythonFunctionCall,
    PythonFunctionDefStatement,
    PythonImportStatement,
    PythonItemAccess,
    PythonLiteral,
    PythonOperator,
    PythonReference,
    PythonUnaryOperator,
)

###############################################################################
# Constants
###############################################################################

BUILTINS_MODULE: Final[str] = '__builtins__'

BUILTIN_FUNCTIONS: Final[List[str]] = [
    'abs',
    'all',
    'any',
    'ascii',
    'bin',
    'breakpoint',
    'callable',
    'chr',
    'classmethod',
    'compile',
    'delattr',
    'dir',
    'divmod',
    'enumerate',
    'eval',
    'exec',
    'filter',
    'format',
    'getattr',
    'globals',
    'hasattr',
    'hash',
    'help',
    'hex',
    'id',
    'input',
    'isinstance',
    'issubclass',
    'iter',
    'len',
    'locals',
    'map',
    'max',
    'min',
    'next',
    'oct',
    'open',
    'ord',
    'pow',
    'print',
    'repr',
    'reversed',
    'round',
    'setattr',
    'sorted',
    'staticmethod',
    'sum',
    'super',
    'vars',
    'zip',
    '__import__',
]

BUILTIN_CLASSES: Final[List[str]] = [
    'bool',
    'bytearray',
    'bytes',
    'complex',
    'dict',
    'float',
    'frozenset',
    'int',
    'list',
    'memoryview',
    'object',
    'property',
    'range',
    'set',
    'slice',
    'str',
    'tuple',
    'type',
]

BUILTIN_EXCEPTIONS: Final[List[str]] = [
    'BaseException',
    'Exception',
    'ArithmeticError',
    'BufferError',
    'LookupError',
    'AssertionError',
    'AttributeError',
    'EOFError',
    'FloatingPointError',
    'GeneratorExit',
    'ImportError',
    'ModuleNotFoundError',
    'IndexError',
    'KeyError',
    'KeyboardInterrupt',
    'MemoryError',
    'NameError',
    'NotImplementedError',
    'OSError',
    'OverflowError',
    'RecursionError',
    'ReferenceError',
    'RuntimeError',
    'StopIteration',
    'StopAsyncIteration',
    'SyntaxError',
    'IndentationError',
    'TabError',
    'SystemError',
    'SystemExit',
    'TypeError',
    'UnboundLocalError',
    'UnicodeError',
    'UnicodeEncodeError',
    'UnicodeDecodeError',
    'UnicodeTranslateError',
    'ValueError',
    'ZeroDivisionError',
    'EnvironmentError',
    'IOError',
    'WindowsError',
    'BlockingIOError',
    'ChildProcessError',
    'ConnectionError',
    'BrokenPipeError',
    'ConnectionAbortedError',
    'ConnectionRefusedError',
    'ConnectionResetError',
    'FileExistsError',
    'FileNotFoundError',
    'InterruptedError',
    'IsADirectoryError',
    'NotADirectoryError',
    'PermissionError',
    'ProcessLookupError',
    'TimeoutError',
    'Warning',
    'UserWarning',
    'DeprecationWarning',
    'PendingDeprecationWarning',
    'SyntaxWarning',
    'RuntimeWarning',
    'FutureWarning',
    'ImportWarning',
    'UnicodeWarning',
    'BytesWarning',
    'ResourceWarning',
]


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
    ITERABLE = enum.auto()
    MAPPING = enum.auto()
    OBJECT = enum.auto()
    NUMBER = INT | FLOAT | COMPLEX
    PRIMITIVE = NONE | BOOL | NUMBER | STRING
    DEFINITIONS = FUNCTION | CLASS | EXCEPTION
    OBJECTS = STRING | ITERABLE | MAPPING | OBJECT
    ANY = PRIMITIVE | DEFINITIONS | OBJECT

    @classmethod
    def from_value(cls, value: Any) -> Self:
        if raw_value is None:
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
        if isinstance(value, FunctionWrapper) or callable(value):
            return cls.FUNCTION
        if isinstance(value, (tuple, list, set)):
            return cls.ITERABLE
        if isinstance(value, dict):
            return cls.MAPPING
        d = {}
        g = (type(d.items()), type(d.keys()), type(d.values()))
        if isinstance(value, g):
            return cls.ITERABLE
        return cls.OBJECT

    @property
    def can_be_bool(self) -> bool:
        return bool(self & PythonType.BOOL)

    @property
    def can_be_int(self) -> bool:
        return bool(self & PythonType.INT)

    @property
    def can_be_float(self) -> bool:
        return bool(self & PythonType.FLOAT)

    @property
    def can_be_complex(self) -> bool:
        return bool(self & PythonType.COMPLEX)

    @property
    def can_be_number(self) -> bool:
        return bool(self & PythonType.NUMBER)

    @property
    def can_be_string(self) -> bool:
        return bool(self & PythonType.STRING)

    @property
    def can_be_primitive(self) -> bool:
        return bool(self & PythonType.PRIMITIVE)

    @property
    def can_be_function(self) -> bool:
        return bool(self & PythonType.FUNCTION)

    @property
    def can_be_class(self) -> bool:
        return bool(self & PythonType.CLASS)

    @property
    def can_be_exception(self) -> bool:
        return bool(self & PythonType.EXCEPTION)

    @property
    def can_be_definitions(self) -> bool:
        return bool(self & PythonType.DEFINITIONS)

    @property
    def can_be_iterable(self) -> bool:
        return bool(self & PythonType.ITERABLE)

    @property
    def can_be_mapping(self) -> bool:
        return bool(self & PythonType.MAPPING)

    @property
    def can_be_object(self) -> bool:
        return bool(self & PythonType.OBJECT)

    @property
    def can_be_any_object(self) -> bool:
        return bool(self & PythonType.OBJECTS)

    @property
    def can_have_attributes(self) -> bool:
        return bool(self & ~PythonType.PRIMITIVE)

    @property
    def can_have_items(self) -> bool:
        mask = PythonType.STRING | PythonType.ITERABLE | PythonType.MAPPING | PythonType.OBJECT
        return bool(self & mask)


###############################################################################
# Data Structures
###############################################################################


@frozen
class PythonTypeToken(TypeToken[V]):
    mask: TypeMask = TypeMask.ANY

    @classmethod
    def of(cls, value: V, mask: TypeMask = TypeMask.ANY) -> Self:
        if mask == TypeMask.ANY:
            mask = TypeMask.from_value(value)
        return cls(type(value), mask=mask)

    @classmethod
    def of_bool(cls, mask: TypeMask = TypeMask.BOOL) -> Self:
        return cls(bool, mask=mask)

    @classmethod
    def of_int(cls, mask: TypeMask = TypeMask.INT) -> Self:
        return cls(int, mask=mask)

    @classmethod
    def of_float(cls, mask: TypeMask = TypeMask.FLOAT) -> Self:
        return cls(float, mask=mask)

    @classmethod
    def of_complex(cls, mask: TypeMask = TypeMask.COMPLEX) -> Self:
        return cls(complex, mask=mask)

    @classmethod
    def of_string(cls, mask: TypeMask = TypeMask.STRING) -> Self:
        return cls(str, mask=mask)

    @classmethod
    def of_builtin_function(cls, mask: TypeMask = TypeMask.FUNCTION) -> Self:
        return cls(BUILTIN_FUNCTION_TYPE, mask=mask)

    @classmethod
    def of_def_function(cls, mask: TypeMask = TypeMask.FUNCTION) -> Self:
        return cls(DEF_FUNCTION_TYPE, mask=mask)

    @classmethod
    def of_class(cls, mask: TypeMask = TypeMask.CLASS) -> Self:
        return cls(CLASS_TYPE, mask=mask)

    @classmethod
    def of_exception(cls, mask: TypeMask = TypeMask.EXCEPTION) -> Self:
        return cls(Exception, mask=mask)

    @classmethod
    def of_iterable(cls, mask: TypeMask = TypeMask.ITERABLE) -> Self:
        return cls(IterableType, mask=mask)

    @classmethod
    def of_list(cls, mask: TypeMask = TypeMask.ITERABLE) -> Self:
        return cls(list, mask=mask)

    @classmethod
    def of_tuple(cls, mask: TypeMask = TypeMask.ITERABLE) -> Self:
        return cls(tuple, mask=mask)

    @classmethod
    def of_set(cls, mask: TypeMask = TypeMask.ITERABLE) -> Self:
        return cls(set, mask=mask)

    @classmethod
    def of_mapping(cls, mask: TypeMask = TypeMask.MAPPING) -> Self:
        return cls(MappingType, mask=mask)

    @classmethod
    def of_dict(cls, mask: TypeMask = TypeMask.MAPPING) -> Self:
        return cls(dict, mask=mask)

    @classmethod
    def of_custom_object(cls, token: Type[V]) -> Self:
        return cls(token, mask=TypeMask.OBJECT)


TYPE_TOKEN_ANYTHING: Final[PythonTypeToken[Any]] = PythonTypeToken(object)
TYPE_TOKEN_BOOL: Final[PythonTypeToken[bool]] = PythonTypeToken.of_bool()
TYPE_TOKEN_INT: Final[PythonTypeToken[int]] = PythonTypeToken.of_int()
TYPE_TOKEN_FLOAT: Final[PythonTypeToken[float]] = PythonTypeToken.of_float()
TYPE_TOKEN_COMPLEX: Final[PythonTypeToken[complex]] = PythonTypeToken.of_complex()
TYPE_TOKEN_STRING: Final[PythonTypeToken[str]] = PythonTypeToken.of_string()
TYPE_TOKEN_ITERABLE: Final[PythonTypeToken[Iterable]] = PythonTypeToken.of_iterable()
TYPE_TOKEN_LIST: Final[PythonTypeToken[list]] = PythonTypeToken.of_list()
TYPE_TOKEN_TUPLE: Final[PythonTypeToken[tuple]] = PythonTypeToken.of_tuple()
TYPE_TOKEN_SET: Final[PythonTypeToken[set]] = PythonTypeToken.of_set()
TYPE_TOKEN_MAPPING: Final[PythonTypeToken[Mapping]] = PythonTypeToken.of_mapping()
TYPE_TOKEN_DICT: Final[PythonTypeToken[dict]] = PythonTypeToken.of_dict()
TYPE_TOKEN_BUILTIN: Final[PythonTypeToken[BUILTIN_FUNCTION_TYPE]] = PythonTypeToken.of_builtin_function()
TYPE_TOKEN_FUNCTION: Final[PythonTypeToken[DEF_FUNCTION_TYPE]] = PythonTypeToken.of_def_function()
TYPE_TOKEN_CLASS: Final[PythonTypeToken[CLASS_TYPE]] = PythonTypeToken.of_class()
TYPE_TOKEN_EXCEPTION: Final[PythonTypeToken[Exception]] = PythonTypeToken.of_exception()
TYPE_TOKEN_OBJECT: Final[PythonTypeToken[Any]] = PythonTypeToken(object, mask=TypeMask.OBJECT)


def unknown_value(
    type: PythonTypeToken[V] = TYPE_TOKEN_ANYTHING,
    source: Optional[TrackedCode] = None,
) -> Result[V]:
    return Result(type, source)


def unknown_int(source: Optional[TrackedCode] = None) -> UnresolvedInt:
    return UnresolvedInt(TYPE_TOKEN_INT, source)


def unknown_float(source: Optional[TrackedCode] = None) -> UnresolvedFloat:
    return UnresolvedFloat(TYPE_TOKEN_FLOAT, source)


def unknown_string(source: Optional[TrackedCode] = None) -> UnresolvedString:
    return UnresolvedString(TYPE_TOKEN_STRING, source)


def unknown_iterable(source: Optional[TrackedCode] = None) -> UnresolvedIterable:
    return UnresolvedIterable(TYPE_TOKEN_ITERABLE, source)


def unknown_list(source: Optional[TrackedCode] = None) -> UnresolvedIterable:
    return UnresolvedIterable(TYPE_TOKEN_LIST, source)


def unknown_tuple(source: Optional[TrackedCode] = None) -> UnresolvedIterable:
    return UnresolvedIterable(TYPE_TOKEN_TUPLE, source)


def unknown_set(source: Optional[TrackedCode] = None) -> UnresolvedIterable:
    return UnresolvedIterable(TYPE_TOKEN_SET, source)


def unknown_mapping(source: Optional[TrackedCode] = None) -> UnresolvedMapping:
    return UnresolvedMapping(TYPE_TOKEN_MAPPING, source)


def unknown_dict(source: Optional[TrackedCode] = None) -> UnresolvedMapping:
    return UnresolvedMapping(TYPE_TOKEN_DICT, source)


def const_int(raw_value: int, source: Optional[TrackedCode] = None) -> Resolved[int]:
    return Resolved(TYPE_TOKEN_INT, source, raw_value)


def const_float(raw_value: float, source: Optional[TrackedCode] = None) -> Resolved[float]:
    return Resolved(TYPE_TOKEN_FLOAT, source, raw_value)


def const_complex(raw_value: complex, source: Optional[TrackedCode] = None) -> Resolved[complex]:
    return Resolved(TYPE_TOKEN_COMPLEX, source, raw_value)


def const_number(raw_value: V, source: Optional[TrackedCode] = None) -> Resolved[V]:
    token = PythonTypeToken.of(raw_value, mask=TypeMask.NUMBER)
    return Resolved(token, source, raw_value)


def const_string(raw_value: str, source: Optional[TrackedCode] = None) -> Resolved[str]:
    return Resolved(TYPE_TOKEN_STRING, source, raw_value)


def const_bool(raw_value: bool, source: Optional[TrackedCode] = None) -> Resolved[bool]:
    return Resolved(TYPE_TOKEN_BOOL, source, raw_value)


def const_tuple(raw_value: Tuple, source: Optional[TrackedCode] = None) -> Resolved[Tuple]:
    return Resolved(TYPE_TOKEN_TUPLE, source, raw_value)


def const_list(raw_value: List[V], source: Optional[TrackedCode] = None) -> Resolved[List[V]]:
    token = PythonTypeToken.of(raw_value, mask=TypeMask.ITERABLE)
    return Resolved(token, source, raw_value)


def const_set(raw_value: Set[V], source: Optional[TrackedCode] = None) -> Resolved[Set[V]]:
    token = PythonTypeToken.of(raw_value, mask=TypeMask.ITERABLE)
    return Resolved(token, source, raw_value)


def const_dict(raw_value: Dict[K, V], source: Optional[TrackedCode] = None) -> Resolved[Dict[K, V]]:
    token = PythonTypeToken.of(raw_value, mask=TypeMask.MAPPING)
    return Resolved(token, source, raw_value)


def const_none(source: Optional[TrackedCode] = None) -> Resolved[Any]:
    token = PythonTypeToken.of(None, mask=TypeMask.NONE)
    return Resolved(token, source, None)


def solved(
    type: PythonTypeToken[V],
    raw_value: V,
    source: Optional[TrackedCode] = None,
) -> Resolved[V]:
    return Resolved(type, source, raw_value)


def solved_from(raw_value: V, source: Optional[TrackedCode] = None) -> Resolved[V]:
    if raw_value is None:
        return const_none(source=source)
    if isinstance(raw_value, bool):
        return const_bool(raw_value, source=source)
    if isinstance(raw_value, int):
        return const_int(raw_value, source=source)
    if isinstance(raw_value, float):
        return const_float(raw_value, source=source)
    if isinstance(raw_value, complex):
        return const_complex(raw_value, source=source)
    if isinstance(raw_value, str):
        return const_string(raw_value, source=source)
    if isinstance(raw_value, BaseException):
        token = PythonTypeToken.of(raw_value, mask=TypeMask.EXCEPTION)
        return Resolved(token, source, raw_value)
    if isinstance(raw_value, type):
        token = PythonTypeToken.of(raw_value, mask=TypeMask.CLASS)
        return Resolved(token, source, raw_value)
    if isinstance(raw_value, FunctionWrapper) or callable(raw_value):
        token = PythonTypeToken.of(raw_value, mask=TypeMask.FUNCTION)
        return Resolved(token, source, raw_value)
    if isinstance(raw_value, tuple):
        return const_tuple(raw_value, source=source)
    if isinstance(raw_value, list):
        return const_list(raw_value, source=source)
    if isinstance(raw_value, set):
        return const_set(raw_value, source=source)
    if isinstance(raw_value, dict):
        return const_dict(raw_value, source=source)
    d = {}
    g = (type(d.items()), type(d.keys()), type(d.values()))
    if isinstance(raw_value, g):
        token = PythonTypeToken.of(raw_value, mask=TypeMask.ITERABLE)
        return Resolved(token, source, raw_value)
    token = PythonTypeToken.of(raw_value, mask=TypeMask.OBJECT)
    return Resolved(token, source, raw_value)


@frozen
class FunctionWrapper:
    function: str
    module: str
    call: Callable


def wrap_normal_function(function: Callable) -> Callable:
    # The purpose of this is just to make return values uniform.
    def wrapper(*args, **kwargs) -> VariantData[Result]:
        for arg in args:
            if not arg.is_resolved:
                print()
                print(f'unresolved call to {function}({args}, {kwargs})')
                return VariantData.with_base_value(unknown_value())
        for arg in kwargs.values():
            if not arg.is_resolved:
                print()
                print(f'unresolved call to {function}({args}, {kwargs})')
                return VariantData.with_base_value(unknown_value())
        raw_args = [arg.value for arg in args]
        raw_kwargs = {key: arg.value for key, arg in kwargs.items()}
        raw_value = function(*raw_args, **raw_kwargs)
        if isinstance(raw_value, list):
            value = [solved_from(v) for v in raw_value]
        elif isinstance(raw_value, tuple):
            value = tuple(solved_from(v) for v in raw_value)
        elif isinstance(raw_value, dict):
            value = {solved_from(k): solved_from(v) for k, v in raw_value.items()}
        else:
            value = solved_from(raw_value)
        return VariantData.with_base_value(value)
    return wrapper


def builtin_function_wrapper(name: str, function: Callable) -> FunctionWrapper:
    return library_function_wrapper(name, '__builtins__', function)


def library_function_wrapper(name: str, module: str, function: Callable) -> FunctionWrapper:
    wrapper = wrap_normal_function(function)
    return FunctionWrapper(name, module, wrapper)


def custom_function_wrapper(name: str, module: str, function: Callable) -> FunctionWrapper:
    def wrapper(*args, **kwargs) -> VariantData[Result]:
        raw_value = function(*args, **kwargs)
        return VariantData.with_base_value(solved_from(raw_value))
    return FunctionWrapper(name, module, wrapper)


@frozen
class Definition:
    value: Result[Any]
    ast: Optional[PythonAst] = None
    import_base: str = ''

    @property
    def line(self) -> int:
        return 0 if self.ast is None else getattr(self.ast, 'line', 0)

    @property
    def column(self) -> int:
        return 0 if self.ast is None else getattr(self.ast, 'column', 0)

    @property
    def is_imported(self) -> bool:
        return bool(self.import_base)

    @property
    def is_builtin(self) -> bool:
        return self.import_base == BUILTINS_MODULE

    @classmethod
    def of_builtin_function(cls, name: str) -> Self:
        #value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert callable(raw_value),  f'expected function, got: {raw_value!r}'
        token = PythonTypeToken(type(raw_value), mask=TypeMask.FUNCTION)
        wrapper = builtin_function_wrapper(name, raw_value)
        value = Resolved(token, None, wrapper)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def of_builtin_class(cls, name: str) -> Self:
        #value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert isinstance(raw_value, type), f'expected class, got: {raw_value!r}'
        token = PythonTypeToken(type(raw_value), mask=TypeMask.CLASS)
        value = Resolved(token, None, raw_value)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def of_builtin_exception(cls, name: str) -> Self:
        #value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert isinstance(raw_value, type), f'expected class, got: {raw_value!r}'
        assert issubclass(raw_value, BaseException), f'expected exception, got: {raw_value!r}'
        token = PythonTypeToken(type(raw_value), mask=TypeMask.EXCEPTION)
        value = Resolved(token, None, raw_value)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def from_value(cls, raw_value: Any, ast: Optional[PythonAst] = None) -> Self:
        return cls(solved_from(raw_value), ast=ast)  # FIXME TrackedCode from ast

    def cast_to(self, type: PythonTypeToken) -> Self:
        if self.value.type == type:
            return self
        new_type_mask = self.value.type.mask & type.mask
        if bool(new_type_mask):
            new_type = evolve(type, mask=new_type_mask)
            return evolve(self, value=self.value.cast_to(new_type))
        raise TypeError(f'unable to cast {self.value.type} to {type}')

    def __str__(self) -> str:
        if self.import_base:
            return f'{self.value} (import from {self.import_base})'
        return f'{self.value}'


###############################################################################
# Interface
###############################################################################


def _default_return_value() -> VariantData[Result[Any]]:
    return VariantData.with_base_value(const_none())


@define
class DataScope:
    variables: Dict[str, VariantData[Definition]] = field(factory=dict)
    return_values: VariantData[Result[Any]] = field(factory=_default_return_value)
    _condition_stack: List[LogicValue] = field(init=False, factory=list)
    _symbols: Dict[str, Any] = field(factory=dict)

    @property
    def condition(self) -> LogicValue:
        if not self._condition_stack:
            return TRUE
        return self._condition_stack[-1]

    @classmethod
    def with_builtins(cls) -> Self:
        variables = {}
        for name in BUILTIN_FUNCTIONS:
            variables[name] = VariantData.with_base_value(Definition.of_builtin_function(name))
        for name in BUILTIN_CLASSES:
            variables[name] = VariantData.with_base_value(Definition.of_builtin_class(name))
        for name in BUILTIN_EXCEPTIONS:
            variables[name] = VariantData.with_base_value(Definition.of_builtin_exception(name))
        return cls(variables=variables)

    def duplicate(self) -> Self:
        variables = {k: v.duplicate() for k, v in self.variables.items()}
        return self.__class__(variables=variables)

    def get(self, name: str) -> VariantData[Definition]:
        return self.variables.get(name, VariantData())

    def set(
        self,
        name: str,
        value: Result[Any],
        ast: Optional[PythonAst] = None,
        import_base: str = '',
    ):
        definition = Definition(value, ast=ast, import_base=import_base)
        self.define(name, definition)

    def set_raw_value(
        self,
        name: str,
        raw_value: Any,
        type_mask: TypeMask = TypeMask.ANY,
        ast: Optional[PythonAst] = None,
        import_base: str = '',
    ):
        # TODO TrackedCode from ast
        source: Optional[TrackedCode] = None
        token = PythonTypeToken.of(raw_value, mask=type_mask)
        value = Resolved(token, source, raw_value)
        return self.set(name, value, ast=ast, import_base=import_base)

    def set_unknown(
        self,
        name: str,
        type_mask: TypeMask = TypeMask.ANY,
        ast: Optional[PythonAst] = None,
        import_base: str = '',
    ):
        token = PythonTypeToken(object, mask=type_mask)
        value = unknown_value(type=token)
        return self.set(name, value, ast=ast, import_base=import_base)

    def define(self, name: str, definition: Definition):
        var = self.variables.get(name)
        if var is None:
            var = VariantData()
            self.variables[name] = var
        var.set(definition, self.condition)

    def push_condition(self, condition: LogicValue):
        self._condition_stack.append(condition)

    def pop_condition(self) -> LogicValue:
        self._condition_stack.pop()

    def add_imported_function(self, name: str, module: str, function: Callable):
        # `function` receives `Result`, returns `Any`
        wrapper = custom_function_wrapper(name, module, function)
        self.add_imported_symbol(name, module, wrapper)

    def add_imported_symbol(self, name: str, module: str, value: Any):
        full_name = f'{module}.{name}' if module else name
        self._symbols[full_name] = value

    def add_import(self, statement: PythonImportStatement):
        assert statement.is_statement and statement.is_import
        for imported_name in statement.names:
            if imported_name.is_wildcard:
                continue  # FIXME TODO
            name = imported_name.alias if imported_name.alias else imported_name.name
            import_base = imported_name.base.dotted_name
            if import_base:
                full_name = f'{import_base}.{imported_name.name}'
                if full_name in self._symbols:
                    raw_value = self._symbols[full_name]
                    value = solved_from(raw_value)
                    self.set(name, value, ast=statement, import_base=import_base)
                else:
                    self.set_unknown(name, ast=statement, import_base=import_base)
            else:
                import_base = imported_name.name
                if import_base in self._symbols:
                    raw_value = self._symbols[import_base]
                    value = solved_from(raw_value)
                    self.set(name, value, ast=statement, import_base=import_base)
                else:
                    self.set_unknown(name, ast=statement, import_base=import_base)

    def add_function_def(self, statement: PythonFunctionDefStatement, fun: FunctionWrapper = None):
        assert statement.is_statement and statement.is_function_def
        if fun is None:
            self.set_unknown(statement.name, type=PythonType.FUNCTION, ast=statement)
        else:
            self.set_raw_value(statement.name, fun, type=PythonType.FUNCTION, ast=statement)

    def add_class_def(self, statement: PythonClassDefStatement):
        assert statement.is_statement and statement.is_class_def
        self.set_unknown(statement.name, type=PythonType.CLASS, ast=statement)

    def add_assignment(self, statement: PythonAssignmentStatement):
        assert statement.is_statement and statement.is_assignment
        if statement.is_packed or statement.is_unpacked:
            return  # FIXME TODO
        if statement.is_augmented:
            return  # FIXME TODO
        variable = statement.variable
        if not variable.is_reference:
            return  # FIXME TODO
        if variable.object is not None:
            return  # FIXME TODO
        name = variable.name
        value = self.value_from_expression(statement.value)
        self.set(name, value, ast=statement)

    def add_return_value(self, expression: Optional[PythonExpression] = None):
        if expression is None:
            value = const_none()
        else:
            value = self.value_from_expression(expression)
        self.return_values.set(value, self.condition)

    def evaluate_condition(self, expression: PythonExpression) -> LogicValue:
        # FIXME this can be improved to actually convert operators into logic
        value = self.value_from_expression(expression)
        if value.is_resolved:
            return TRUE if bool(value.value) else FALSE
        return LogicVariable(data=expression)

    def value_from_expression(self, expression: PythonExpression) -> Result:
        assert expression.is_expression
        if expression.is_literal:
            return self.value_from_literal(expression)
        if expression.is_reference:
            return self.value_from_reference(expression)
        if expression.is_item_access:
            return self.value_from_item_access(expression)
        if expression.is_function_call:
            return self.value_from_function_call(expression)
        if expression.is_star_expression:
            return unknown_value(type=TYPE_TOKEN_OBJECT)
        if expression.is_generator:
            return unknown_value(type=TYPE_TOKEN_OBJECT)
        if expression.is_operator:
            return self.value_from_operator(expression)
        if expression.is_conditional:
            return unknown_value()
        if expression.is_lambda:
            return unknown_value(type=DEF_FUNCTION_TYPE)
        if expression.is_assignment:
            # Python >= 3.8
            return unknown_value()
        if expression.is_yield:
            return unknown_value()
        if expression.is_await:
            return unknown_value()
        return unknown_value()

    def value_from_literal(self, literal: PythonLiteral) -> Result:
        assert literal.is_expression and literal.is_literal
        if literal.is_none:
            return const_none()
        if literal.is_bool:
            return const_bool(literal.value)
        if literal.is_number:
            if literal.is_int:
                return const_int(literal.value)
            if literal.is_float:
                return const_float(literal.value)
            if literal.is_complex:
                return const_complex(literal.value)
        if literal.is_string:
            return const_string(literal.value)
        if literal.is_tuple and not literal.is_comprehension:
            values = tuple(self.value_from_expression(v) for v in literal.values)
            if all(v.is_resolved for v in values):
                values = tuple(v.value for v in values)
                return const_tuple(values)
        if literal.is_list and not literal.is_comprehension:
            values = list(self.value_from_expression(v) for v in literal.values)
            if all(v.is_resolved for v in values):
                # values = list(v.value for v in values)
                return const_list(values)
            else:
                return const_list(values)
        if literal.is_dict and not literal.is_comprehension:
            entries = dict(
                (self.value_from_expression(e.key), self.value_from_expression(e.value))
                for e in literal.entries
            )
            # if not all(key.is_resolved for key in entries.keys()):
            return const_dict(entries)
        # TODO FIXME
        return unknown_value(type=TYPE_TOKEN_OBJECT)

    def value_from_reference(self, reference: PythonReference) -> Result:
        if reference.object is not None:
            obj: Result = self.value_from_expression(reference.object)
            if not obj.type.can_have_attributes:
                return unknown_value()
            if not obj.is_resolved:
                return unknown_value()
            value = getattr(obj.value, reference.name)
            if callable(value):
                value = library_function_wrapper(reference.name, str(type(obj.value)), value)
            return solved_from(value)
        var = self.get(reference.name)
        if reference.name in ('lidar_pkg_dir', 'LDS_LAUNCH_FILE'):
            print()
            print(f'>> reference to {reference.name}:')
            print('  ', var)
        if not var.has_values or not var.is_deterministic:
            return unknown_value()
        assert var.has_base_value
        definition = var.get()
        return definition.value

    def value_from_operator(self, operator: PythonOperator) -> Result:
        assert operator.is_operator
        if operator.is_unary:
            return self.value_from_unary_operator(operator)
        if operator.is_binary:
            return self.value_from_binary_operator(operator)
        return unknown_value()

    def value_from_unary_operator(self, operator: PythonUnaryOperator) -> Result:
        assert operator.is_unary
        value = self.value_from_expression(operator.operand)
        if operator.is_bitwise:
            if not value.type.can_be_int:
                raise DataFlowError.type_check('INT', value.type.name, value)
            if not value.is_resolved:
                return unknown_int()
            r = ~value.value
            return const_int(r)
        if operator.is_arithmetic:
            if not value.type.can_be_number:
                raise DataFlowError.type_check('NUMBER', value.type.name, value)
            if not value.is_resolved:
                token = PythonTypeToken(object, mask=TypeMask.NUMBER)
                return unknown_value(type=token)
            if operator.operator == '+':
                r = +value.value
            elif operator.operator == '-':
                r = -value.value
            else:
                token = PythonTypeToken(object, mask=TypeMask.NUMBER)
                return unknown_value(type=token)
            # TODO FIXME refine return type
            return const_number(r)
        if operator.is_logic:
            if not value.is_resolved:
                return unknown_value(type=TYPE_TOKEN_BOOL)
            r = not value.value
            return const_bool(r)
        return unknown_value()

    def value_from_binary_operator(self, operator: PythonBinaryOperator) -> Result[Any]:
        assert operator.is_binary
        a = self.value_from_expression(operator.operand1)
        b = self.value_from_expression(operator.operand2)
        o = operator.operator
        if not a.is_resolved:
            return unknown_value()
        if not b.is_resolved:
            return unknown_value()
        if operator.is_logic:
            if o == 'and':
                return const_bool(a.value and b.value)
            if o == 'or':
                return const_bool(a.value or b.value)
            return unknown_value(type=TYPE_TOKEN_BOOL)
        if operator.is_comparison:
            if o == '==':
                return const_bool(a.value == b.value)
            if o == '!=':
                return const_bool(a.value != b.value)
            if o == '<':
                return const_bool(a.value < b.value)
            if o == '<=':
                return const_bool(a.value <= b.value)
            if o == '>':
                return const_bool(a.value > b.value)
            if o == '>=':
                return const_bool(a.value >= b.value)
            if o == 'in':
                return const_bool(a.value in b.value)
            if o == 'not in':
                return const_bool(a.value not in b.value)
            if o == 'is':
                return const_bool(a.value is b.value)
            if o == 'is not':
                return const_bool(a.value is not b.value)
            return unknown_value(type=TYPE_TOKEN_BOOL)
        if operator.is_arithmetic:
            if o == '+':
                r = a.value + b.value
                return const_string(r) if isinstance(r, str) else const_number(r)
            if o == '-':
                return const_number(a.value - b.value)
            if o == '*':
                return const_number(a.value * b.value)
            if o == '/':
                return const_number(a.value / b.value)
            if o == '//':
                return const_int(a.value // b.value)
            if o == '**':
                return const_number(a.value ** b.value)
            token = PythonTypeToken(object, mask=TypeMask.NUMBER)
            return unknown_value(type=token)
        return unknown_value()

    def value_from_item_access(self, access: PythonItemAccess) -> Result:
        # object: PythonExpression
        # key: PythonSubscript
        obj: Result = self.value_from_expression(access.object)
        if not obj.is_resolved or not obj.type.can_have_items:
            return unknown_value()
        if access.key.is_slice:
            return unknown_value()
        assert access.key.is_key
        key: Result = self.value_from_expression(access.key.expression)
        if not key.is_resolved:
            return unknown_value()
        try:
            value = obj.value[key.value]
        except KeyError:
            return unknown_value()
        if isinstance(value, Result):
            return value
        return solved_from(value)

    def value_from_function_call(self, call: PythonFunctionCall) -> Result:
        value = self.value_from_expression(call.function)
        if not value.is_resolved:
            return unknown_value()
        if not value.type.can_be_function:
            return unknown_value()
        function = value.value
        assert isinstance(function, FunctionWrapper), f'not function wrapper: {repr(function)}'
        assert callable(function.call), f'not callable: {repr(function.call)}'
        if not call.arguments:
            result: VariantData[Result] = function.call()
            if result.has_values and result.is_deterministic:
                return result.get()
            return unknown_value()
        args = []
        kwargs = {}
        for argument in call.arguments:
            arg = self.value_from_expression(argument.value)
            if argument.is_positional:
                args.append(arg)
            elif argument.is_key_value:
                kwargs[argument.name] = arg
            elif arg.is_resolved:
                if argument.is_star:
                    args.extend(arg)
                elif argument.is_double_star:
                    kwargs.update(arg)
            else:
                if argument.is_star:
                    args.append(arg)
                elif argument.is_double_star:
                    # kwargs.update(arg)
                    return unknown_value()  # FIXME
        result: VariantData[Result] = function.call(*args, **kwargs)
        if result.has_values and result.is_deterministic:
            return result.get()
        return unknown_value()
