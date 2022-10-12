# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final, List

from enum import Enum

from attrs import define, field, frozen

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
    value: Any
    import_base: str = ''
    data_type: PythonType = PythonType.OBJECT

    @property
    def is_import(self) -> bool:
        return bool(self.import_base)

    @classmethod
    def of_builtin_function(cls, name: str) -> 'Definition':
        value = getattr(__builtins__, name)
        assert callable(value),  f'expected function, got: {value!r}'
        return cls(name, value, import_base='__builtins__', data_type=PythonType.FUNCTION)

    @classmethod
    def of_builtin_class(cls, name: str) -> 'Definition':
        value = getattr(__builtins__, name)
        assert isinstance(value, type), f'expected class, got: {value!r}'
        return cls(name, value, import_base='__builtins__', data_type=PythonType.CLASS)

    @classmethod
    def of_builtin_exception(cls, name: str) -> 'Definition':
        value = getattr(__builtins__, name)
        assert isinstance(value, BaseException), f'expected exception, got: {value!r}'
        return cls(name, value, import_base='__builtins__', data_type=PythonType.EXCEPTION)


###############################################################################
# Interface
###############################################################################


@define
class DataScope:
    defs: Dict[str, Definition] = field(factory=dict)

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
        return self.__class__(defs=dict(self.defs))
