# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, List, Optional

from enum import Enum

from attrs import define, field, frozen

from haros.metamodel.common import VariantData
from haros.metamodel.logic import TRUE, LogicValue
from haros.parsing.python.ast import (
    PythonClassDefStatement,
    PythonFunctionDefStatement,
    PythonImportStatement,
)

###############################################################################
# Constants
###############################################################################

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

###############################################################################
# Data Structures
###############################################################################


class PythonType(Enum):
    BOOL = 'bool'
    INT = 'int'
    FLOAT = 'float'
    STRING = 'string'
    FUNCTION = 'function'
    CLASS = 'class'
    EXCEPTION = 'exception'
    OBJECT = 'object'


@frozen
class Definition:
    name: str
    data: VariantData[Any]
    import_base: str = ''
    data_type: PythonType = PythonType.OBJECT
    line: int = 0
    column: int = 0

    @property
    def is_import(self) -> bool:
        return bool(self.import_base)

    @classmethod
    def of_builtin_function(cls, name: str) -> 'Definition':
        value = getattr(__builtins__, name)
        assert callable(value),  f'expected function, got: {value!r}'
        data = VariantData.with_base_value(value)
        return cls(name, data, import_base='__builtins__', data_type=PythonType.FUNCTION)

    @classmethod
    def of_builtin_class(cls, name: str) -> 'Definition':
        value = getattr(__builtins__, name)
        assert isinstance(value, type), f'expected class, got: {value!r}'
        data = VariantData.with_base_value(value)
        return cls(name, data, import_base='__builtins__', data_type=PythonType.CLASS)

    @classmethod
    def of_builtin_exception(cls, name: str) -> 'Definition':
        value = getattr(__builtins__, name)
        assert isinstance(value, BaseException), f'expected exception, got: {value!r}'
        data = VariantData.with_base_value(value)
        return cls(name, data, import_base='__builtins__', data_type=PythonType.EXCEPTION)


###############################################################################
# Interface
###############################################################################


@define
class DataScope:
    defs: Dict[str, Definition] = field(factory=dict)
    condition: LogicValue = TRUE

    @classmethod
    def with_builtins(cls) -> 'DataScope':
        defs = {}
        for name in BUILTIN_FUNCTIONS:
            defs[name] = Definition.of_builtin_function(name)
        for name in BUILTIN_CLASSES:
            defs[name] = Definition.of_builtin_class(name)
        for name in BUILTIN_EXCEPTIONS:
            defs[name] = Definition.of_builtin_exception(name)
        return cls(defs=defs)

    def duplicate(self) -> 'DataScope':
        defs = {k, v.duplicate() for k, v in self.defs.items()}
        return self.__class__(defs=defs)

    def get(self, name: str) -> Optional[Definition]:
        return self.defs.get(name)

    def set(
        self,
        name: str,
        value: Any,
        import_base: str = '',
        data_type: PythonType = PythonType.OBJECT,
        line: int = 0,
        column: int = 0,
    ):
        definition = self.defs.get(name)
        if definition is None:
            definition = Definition(
                name,
                VariantData(),
                import_base=import_base,
                data_type=data_type,
                line=line,
                column=column,
            )
            self.defs[name] = definition
        definition.data.set(value, self.condition)

    def add_import(self, statement: PythonImportStatement):
        assert statement.is_statement and statement.is_import
        for imported_name in statement.names:
            if imported_name.is_wildcard:
                continue  # FIXME
            name = imported_name.alias if imported_name.alias else imported_name.name
            value = Definition(
                name,
                VariantData(),
                import_base=imported_name.base.dotted_name,
                line=imported_name.line,
                column=imported_name.column,
            )
            self.set(value)

    def add_function_def(self, statement: PythonFunctionDefStatement):
        assert statement.is_statement and statement.is_function_def
        name = statement.name
        data_type = PythonType.FUNCTION
        line = statement.line
        column = statement.column
        self.set(name, statement, data_type=data_type, line=line, column=column)

    def add_class_def(self, statement: PythonClassDefStatement):
        assert statement.is_statement and statement.is_class_def
        name = statement.name
        data_type = PythonType.CLASS
        line = statement.line
        column = statement.column
        self.set(name, statement, data_type=data_type, line=line, column=column)
