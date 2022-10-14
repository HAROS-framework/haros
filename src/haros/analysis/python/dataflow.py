# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, List, Optional, Tuple

import enum

from attrs import define, evolve, field, frozen

from haros.metamodel.common import VariantData
from haros.metamodel.logic import TRUE, LogicValue
from haros.parsing.python.ast import (
    PythonAssignmentStatement,
    PythonAst,
    PythonClassDefStatement,
    PythonFunctionDefStatement,
    PythonImportStatement,
    PythonLiteral,
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

UNKNOWN_VALUE: Final[object] = object()


class PythonType(enum.Flag):
    BOOL = enum.auto()
    INT = enum.auto()
    FLOAT = enum.auto()
    COMPLEX = enum.auto()
    STRING = enum.auto()
    FUNCTION = enum.auto()
    CLASS = enum.auto()
    EXCEPTION = enum.auto()
    OBJECT = enum.auto()
    NUMBER = INT | FLOAT | COMPLEX
    ANY = BOOL | INT | FLOAT | COMPLEX | STRING | FUNCTION | CLASS | EXCEPTION | OBJECT


###############################################################################
# Data Structures
###############################################################################


@frozen
class Definition:
    value: Any
    type: PythonType = PythonType.ANY
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
        value = getattr(__builtins__, name)
        assert callable(value),  f'expected function, got: {value!r}'
        return cls(value, type=PythonType.FUNCTION, import_base=BUILTINS_MODULE)

    @classmethod
    def of_builtin_class(cls, name: str) -> 'Definition':
        value = getattr(__builtins__, name)
        assert isinstance(value, type), f'expected class, got: {value!r}'
        return cls(value, type=PythonType.CLASS, import_base=BUILTINS_MODULE)

    @classmethod
    def of_builtin_exception(cls, name: str) -> 'Definition':
        value = getattr(__builtins__, name)
        assert isinstance(value, BaseException), f'expected exception, got: {value!r}'
        return cls(value, type=PythonType.EXCEPTION, import_base=BUILTINS_MODULE)

    @classmethod
    def from_value(cls, value: Any, ast: Optional[PythonAst] = None) -> 'Definition':
        if value is UNKNOWN_VALUE:
            return cls(value, ast=ast)
        if isinstance(value, bool):
            return cls(value, type=PythonType.BOOL, ast=ast)
        if isinstance(value, int):
            return cls(value, type=PythonType.INT, ast=ast)
        if isinstance(value, float):
            return cls(value, type=PythonType.FLOAT, ast=ast)
        if isinstance(value, str):
            return cls(value, type=PythonType.STRING, ast=ast)
        if isinstance(value, BaseException):
            return cls(value, type=PythonType.EXCEPTION, ast=ast)
        if isinstance(value, type):
            return cls(value, type=PythonType.CLASS, ast=ast)
        if callable(value):
            return cls(value, type=PythonType.FUNCTION, ast=ast)
        return cls(value, type=PythonType.OBJECT, ast=ast)

    def cast_to(self, value_type: PythonType) -> 'Definition':
        if self.type == value_type:
            return self
        new_type = self.type & value_type
        if bool(new_type):
            return evolve(self, type=new_type)
        raise TypeError(f'unable to cast {self.type} to {value_type}')


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
        value: Any,
        type: PythonType = PythonType.ANY,
        ast: Optional[PythonAst] = None,
        import_base: str = '',
    ):
        definition = Definition(value, type=type, ast=ast, import_base=import_base)
        self.define(name, definition)

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
            import_base = imported_name.base.dotted_name
            self.set(name, UNKNOWN_VALUE, ast=statement, import_base=import_base)

    def add_function_def(self, statement: PythonFunctionDefStatement):
        assert statement.is_statement and statement.is_function_def
        self.set(statement.name, UNKNOWN_VALUE, type=PythonType.FUNCTION, ast=statement)

    def add_class_def(self, statement: PythonClassDefStatement):
        assert statement.is_statement and statement.is_class_def
        self.set(statement.name, UNKNOWN_VALUE, type=PythonType.CLASS, ast=statement)

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
        if not statement.value.is_literal:
            return  # FIXME TODO
        value, type = self.value_from_literal(statement.value)
        self.set(name, value, type=type ast=statement)

    def value_from_literal(self, literal: PythonLiteral) -> Tuple[Any, PythonType]:
        assert literal.is_expression and literal.is_literal
        if literal.is_none:
            return None, PythonType.OBJECT
        if literal.is_bool:
            return literal.value, PythonType.BOOL
        if literal.is_number:
            if literal.is_int:
                return literal.value, PythonType.INT
            if literal.is_float:
                return literal.value, PythonType.FLOAT
            if literal.is_complex:
                return literal.value, PythonType.COMPLEX
        if literal.is_string:
            return literal.value, PythonType.STRING
        # TODO FIXME
        return UNKNOWN_VALUE, PythonType.OBJECT