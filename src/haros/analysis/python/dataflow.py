# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, List, Optional, Tuple, Union

import enum

from attrs import define, evolve, field, frozen

from haros.metamodel.common import VariantData
from haros.metamodel.logic import TRUE, LogicValue
from haros.parsing.python.ast import (
    PythonAssignmentStatement,
    PythonAst,
    PythonClassDefStatement,
    PythonExpression,
    PythonFunctionDefStatement,
    PythonImportStatement,
    PythonLiteral,
    PythonReference,
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


class PythonType(enum.Flag):
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
    ATOMIC = NONE | BOOL | NUMBER | STRING
    DEFINITIONS = FUNCTION | CLASS | EXCEPTION
    OBJECTS = STRING | ITERABLE | MAPPING | OBJECT
    ANY = ATOMIC | DEFINITIONS | OBJECT

    def can_be_bool(self) -> bool:
        return bool(self & PythonType.BOOL)

    def can_be_int(self) -> bool:
        return bool(self & PythonType.INT)

    def can_be_float(self) -> bool:
        return bool(self & PythonType.FLOAT)

    def can_be_complex(self) -> bool:
        return bool(self & PythonType.COMPLEX)

    def can_be_number(self) -> bool:
        return bool(self & PythonType.NUMBER)

    def can_be_string(self) -> bool:
        return bool(self & PythonType.STRING)

    def can_be_atomic(self) -> bool:
        return bool(self & PythonType.ATOMIC)

    def can_be_function(self) -> bool:
        return bool(self & PythonType.FUNCTION)

    def can_be_class(self) -> bool:
        return bool(self & PythonType.CLASS)

    def can_be_exception(self) -> bool:
        return bool(self & PythonType.EXCEPTION)

    def can_be_definitions(self) -> bool:
        return bool(self & PythonType.DEFINITIONS)

    def can_be_iterable(self) -> bool:
        return bool(self & PythonType.ITERABLE)

    def can_be_mapping(self) -> bool:
        return bool(self & PythonType.MAPPING)

    def can_be_object(self) -> bool:
        return bool(self & PythonType.OBJECT)

    def can_be_any_object(self) -> bool:
        return bool(self & PythonType.OBJECTS)

    def can_have_attributes(self) -> bool:
        return not self.can_be_number() and not self.can_be_bool()


###############################################################################
# Data Structures
###############################################################################


@frozen
class DataFlowValue:
    type: PythonType = field()

    @property
    def is_resolved(self) -> bool:
        return False

    def cast_to(self, type: PythonType) -> 'DataFlowValue':
        return evolve(self, type=type)

    def pretty(self) -> str:
        return f'(?) ({self.type})'


@frozen
class TypedValue(DataFlowValue):
    value: Any

    @property
    def is_resolved(self) -> bool:
        return True

    def pretty(self) -> str:
        return f'{value} ({self.type})'


@frozen
class UnknownValue(DataFlowValue):
    pass


@frozen
class UnknownObject(DataFlowValue):
    attributes: Dict[str, TypedValue] = field(factory=dict)
    keys: Dict[Any, TypedValue] = field(factory=dict)

    def __init__(self, *args, **kwargs):
        self.__attrs_init__(PythonType.OBJECT, *args, **kwargs)


@frozen
class Definition:
    value: DataFlowValue
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

    @property
    def is_bool_type(self) -> bool:
        return bool(self.type & PythonType.BOOL)

    @property
    def is_int_type(self) -> bool:
        return bool(self.type & PythonType.INT)

    @property
    def is_float_type(self) -> bool:
        return bool(self.type & PythonType.FLOAT)

    @property
    def is_complex_type(self) -> bool:
        return bool(self.type & PythonType.COMPLEX)

    @property
    def is_number_type(self) -> bool:
        return bool(self.type & PythonType.NUMBER)

    @property
    def is_string_type(self) -> bool:
        return bool(self.type & PythonType.STRING)

    @property
    def is_function_type(self) -> bool:
        return bool(self.type & PythonType.FUNCTION)

    @property
    def is_class_type(self) -> bool:
        return bool(self.type & PythonType.CLASS)

    @property
    def is_exception_type(self) -> bool:
        return bool(self.type & PythonType.EXCEPTION)

    @property
    def is_object_type(self) -> bool:
        return bool(self.type & PythonType.OBJECT)

    @classmethod
    def of_builtin_function(cls, name: str) -> 'Definition':
        #value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert callable(raw_value),  f'expected function, got: {raw_value!r}'
        value = TypedValue(PythonType.FUNCTION, raw_value)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def of_builtin_class(cls, name: str) -> 'Definition':
        #value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert isinstance(raw_value, type), f'expected class, got: {raw_value!r}'
        value = TypedValue(PythonType.CLASS, raw_value)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def of_builtin_exception(cls, name: str) -> 'Definition':
        #value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert isinstance(raw_value, type), f'expected class, got: {raw_value!r}'
        assert issubclass(raw_value, BaseException), f'expected exception, got: {raw_value!r}'
        value = TypedValue(PythonType.EXCEPTION, raw_value)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def from_value(cls, raw_value: Any, ast: Optional[PythonAst] = None) -> 'Definition':
        if isinstance(raw_value, bool):
            return cls(TypedValue(PythonType.BOOL, raw_value), ast=ast)
        if isinstance(raw_value, int):
            return cls(TypedValue(PythonType.INT, raw_value), ast=ast)
        if isinstance(raw_value, float):
            return cls(TypedValue(PythonType.FLOAT, raw_value), ast=ast)
        if isinstance(raw_value, str):
            return cls(TypedValue(PythonType.STRING, raw_value), ast=ast)
        if isinstance(raw_value, BaseException):
            return cls(TypedValue(PythonType.EXCEPTION, raw_value), ast=ast)
        if isinstance(raw_value, type):
            return cls(TypedValue(PythonType.CLASS, raw_value), ast=ast)
        if callable(raw_value):
            return cls(TypedValue(PythonType.FUNCTION, raw_value), ast=ast)
        return cls(TypedValue(PythonType.OBJECT, raw_value), ast=ast)

    def cast_to(self, type: PythonType) -> 'Definition':
        if self.value.type == type:
            return self
        new_type = self.value.type & type
        if bool(new_type):
            return evolve(self, value=self.value.cast_to(type))
        raise TypeError(f'unable to cast {self.value.type} to {type}')

    def __str__(self) -> str:
        if self.import_base:
            return f'{self.value} (import from {self.import_base})'
        return f'{self.value}'


###############################################################################
# Interface
###############################################################################


@define
class DataScope:
    variables: Dict[str, VariantData[Definition]] = field(factory=dict)
    condition: LogicValue = TRUE

    @classmethod
    def with_builtins(cls) -> 'DataScope':
        variables = {}
        for name in BUILTIN_FUNCTIONS:
            variables[name] = VariantData.with_base_value(Definition.of_builtin_function(name))
        for name in BUILTIN_CLASSES:
            variables[name] = VariantData.with_base_value(Definition.of_builtin_class(name))
        for name in BUILTIN_EXCEPTIONS:
            variables[name] = VariantData.with_base_value(Definition.of_builtin_exception(name))
        return cls(variables=variables)

    def duplicate(self) -> 'DataScope':
        variables = {k: v.duplicate() for k, v in self.variables.items()}
        return self.__class__(variables=variables)

    def get(self, name: str) -> VariantData[Definition]:
        return self.variables.get(name, VariantData())

    def set(
        self,
        name: str,
        value: DataFlowValue,
        ast: Optional[PythonAst] = None,
        import_base: str = '',
    ):
        definition = Definition(value, ast=ast, import_base=import_base)
        self.define(name, definition)

    def set_raw_value(
        self,
        name: str,
        raw_value: Any,
        type: PythonType = PythonType.ANY,
        ast: Optional[PythonAst] = None,
        import_base: str = '',
    ):
        value = TypedValue(type, raw_value)
        return self.set(name, value, ast=ast, import_base=import_base)

    def set_unknown(
        self,
        name: str,
        type: PythonType = PythonType.ANY,
        ast: Optional[PythonAst] = None,
        import_base: str = '',
    ):
        value = UnknownValue(type)
        return self.set(name, value, ast=ast, import_base=import_base)

    def define(self, name: str, definition: Definition):
        var = self.variables.get(name)
        if var is None:
            var = VariantData()
            self.variables[name] = var
        var.set(definition, self.condition)

    def add_import(self, statement: PythonImportStatement):
        assert statement.is_statement and statement.is_import
        for imported_name in statement.names:
            if imported_name.is_wildcard:
                continue  # FIXME TODO
            name = imported_name.alias if imported_name.alias else imported_name.name
            import_base = imported_name.base.dotted_name or imported_name.name
            self.set_unknown(name, ast=statement, import_base=import_base)

    def add_function_def(self, statement: PythonFunctionDefStatement):
        assert statement.is_statement and statement.is_function_def
        self.set_unknown(statement.name, type=PythonType.FUNCTION, ast=statement)

    def add_class_def(self, statement: PythonClassDefStatement):
        assert statement.is_statement and statement.is_class_def
        self.set_unknown(statement.name, type=PythonType.CLASS, ast=statement)

    def add_assignment(self, statement: PythonAssignmentStatement):  # FIXME
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
        self.set(name, value, type=value.type, ast=statement)

    def value_from_expression(self, expression: PythonExpression) -> DataFlowValue:
        assert expression.is_expression
        if expression.is_literal:
            return self.value_from_literal(expression)
        if expression.is_reference:
            return self.value_from_reference(expression)
        if expression.is_item_access:
            return UnknownValue.of_type(PythonType.ANY)
        if expression.is_function_call:
            return UnknownValue.of_type(PythonType.ANY)
        if expression.is_star_expression:
            return UnknownValue.of_type(PythonType.OBJECT)
        if expression.is_generator:
            return UnknownValue.of_type(PythonType.OBJECT)
        if expression.is_operator:
            return UnknownValue.of_type(PythonType.ANY)
        if expression.is_conditional:
            return UnknownValue.of_type(PythonType.ANY)
        if expression.is_lambda:
            return UnknownValue.of_type(PythonType.FUNCTION)
        if expression.is_assignment:
            # Python >= 3.8
            return UnknownValue.of_type(PythonType.ANY)
        if expression.is_yield:
            return UnknownValue.of_type(PythonType.ANY)
        if expression.is_await:
            return UnknownValue.of_type(PythonType.ANY)
        return UnknownValue.of_type(PythonType.ANY)

    def value_from_literal(self, literal: PythonLiteral) -> DataFlowValue:
        assert literal.is_expression and literal.is_literal
        if literal.is_none:
            return TypedValue(None, PythonType.OBJECT)
        if literal.is_bool:
            return TypedValue(literal.value, PythonType.BOOL)
        if literal.is_number:
            if literal.is_int:
                return TypedValue(literal.value, PythonType.INT)
            if literal.is_float:
                return TypedValue(literal.value, PythonType.FLOAT)
            if literal.is_complex:
                return TypedValue(literal.value, PythonType.COMPLEX)
        if literal.is_string:
            return TypedValue(literal.value, PythonType.STRING)
        # TODO FIXME
        return UnknownValue.of_type(PythonType.OBJECT)

    def value_from_reference(self, reference: PythonReference) -> DataFlowValue:
        if reference.object is not None:
            obj, t = self.value_from_expression(reference.object)
            if not t.can_have_attributes():
                return UnknownValue.of_type(PythonType.ANY)
            return UnknownValue.of_type(PythonType.ANY)
        var = self.get(reference.name)
        if not var.has_values or not var.is_deterministic:
            return UnknownValue.of_type(PythonType.ANY)
        assert var.has_base_value
        definition = var.get()
        return definition.value
