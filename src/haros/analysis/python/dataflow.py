# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Dict, Final, List, Optional, Tuple, Union

import enum

from attrs import define, evolve, field, frozen

from haros.metamodel.common import VariantData
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
    def can_be_atomic(self) -> bool:
        return bool(self & PythonType.ATOMIC)

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
        return not self.can_be_number and not self.can_be_bool


###############################################################################
# Data Structures
###############################################################################


@frozen
class UnknownValue:
    def __str__(self) -> str:
        return '(?)'


UNKNOWN_TOKEN: Final[UnknownValue] = UnknownValue()


@frozen
class UnknownObject(UnknownValue):
    attributes: Dict[str, Any] = field(factory=dict)
    keys: Dict[Any, Any] = field(factory=dict)


@frozen
class DataFlowValue:
    type: PythonType = field(default=PythonType.ANY)
    value: Any = field(default=UNKNOWN_TOKEN)

    @property
    def is_resolved(self) -> bool:
        return not isinstance(self.value, UnknownValue)

    @classmethod
    def of(cls, raw_value: Any) -> 'DataFlowValue':
        if raw_value is None:
            return cls(PythonType.NONE, None)
        if isinstance(raw_value, bool):
            return cls(PythonType.BOOL, raw_value)
        if isinstance(raw_value, int):
            return cls(PythonType.INT, raw_value)
        if isinstance(raw_value, float):
            return cls(PythonType.FLOAT, raw_value)
        if isinstance(raw_value, complex):
            return cls(PythonType.COMPLEX, raw_value)
        if isinstance(raw_value, str):
            return cls(PythonType.STRING, raw_value)
        if isinstance(raw_value, BaseException):
            return cls(PythonType.EXCEPTION, raw_value)
        if isinstance(raw_value, type):
            return cls(PythonType.CLASS, raw_value)
        if isinstance(raw_value, FunctionWrapper) or callable(raw_value):
            return cls(PythonType.FUNCTION, raw_value)
        return cls(PythonType.OBJECT, raw_value)

    def cast_to(self, type: PythonType) -> 'DataFlowValue':
        return evolve(self, type=type)

    def pretty(self) -> str:
        return f'{value} ({self.type})'


@frozen
class FunctionWrapper:
    function: str
    module: str
    call: Callable


def builtin_function_wrapper(name: str, function: Callable) -> FunctionWrapper:
    # The purpose of this is just to make return values uniform.
    def wrapper(*args, **kwargs) -> VariantData[DataFlowValue]:
        for arg in args:
            if not arg.is_resolved:
                return DataFlowValue()
        for arg in kwargs.values():
            if not arg.is_resolved:
                return DataFlowValue()
        raw_args = [arg.value for arg in args]
        raw_kwargs = {key: arg.value for key, arg in kwargs.items()}
        raw_value = function(*raw_args, **raw_kwargs)
        return VariantData.with_base_value(DataFlowValue.of(raw_value))
    return FunctionWrapper(name, '__builtins__', wrapper)


def custom_function_wrapper(name: str, module: str, function: Callable) -> FunctionWrapper:
    def wrapper(*args, **kwargs) -> VariantData[DataFlowValue]:
        raw_value = function(*args, **kwargs)
        return VariantData.with_base_value(DataFlowValue.of(raw_value))
    return FunctionWrapper(name, module, wrapper)


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
        wrapper = builtin_function_wrapper(name, raw_value)
        value = DataFlowValue(type=PythonType.FUNCTION, value=wrapper)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def of_builtin_class(cls, name: str) -> 'Definition':
        #value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert isinstance(raw_value, type), f'expected class, got: {raw_value!r}'
        value = DataFlowValue(type=PythonType.CLASS, value=raw_value)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def of_builtin_exception(cls, name: str) -> 'Definition':
        #value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert isinstance(raw_value, type), f'expected class, got: {raw_value!r}'
        assert issubclass(raw_value, BaseException), f'expected exception, got: {raw_value!r}'
        value = DataFlowValue(type=PythonType.EXCEPTION, value=raw_value)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def from_value(cls, raw_value: Any, ast: Optional[PythonAst] = None) -> 'Definition':
        return cls(DataFlowValue.of(raw_value), ast=ast)

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


def _default_return_value() -> VariantData[DataFlowValue]:
    return VariantData.with_base_value(DataFlowValue(type=PythonType.NONE, value=None))


@define
class DataScope:
    variables: Dict[str, VariantData[Definition]] = field(factory=dict)
    return_values: VariantData[DataFlowValue] = field(factory=_default_return_value)
    _condition_stack: List[LogicValue] = field(init=False, factory=list)
    _symbols: Dict[str, Any] = field(factory=dict)

    @property
    def condition(self) -> LogicValue:
        if not self._condition_stack:
            return TRUE
        return self._condition_stack[-1]

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
        value = DataFlowValue(type=type, value=raw_value)
        return self.set(name, value, ast=ast, import_base=import_base)

    def set_unknown(
        self,
        name: str,
        type: PythonType = PythonType.ANY,
        ast: Optional[PythonAst] = None,
        import_base: str = '',
    ):
        value = DataFlowValue(type=type)
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
        # `function` receives `DataFlowValue`, returns `Any`
        wrapper = custom_function_wrapper(name, module, function)
        self.add_imported_symbol(name, module, wrapper)

    def add_imported_symbol(self, name: str, module: str, value: Any):
        full_name = f'{module}.{name}' if module else name
        print('\n\n')
        print(f'Setting symbol "{full_name}" with value: {value}')
        self._symbols[full_name] = value

    def add_import(self, statement: PythonImportStatement):
        assert statement.is_statement and statement.is_import
        for imported_name in statement.names:
            if imported_name.is_wildcard:
                continue  # FIXME TODO
            name = imported_name.alias if imported_name.alias else imported_name.name
            import_base = imported_name.base.dotted_name
            print('\n\n')
            print(f'Import registry: "{name}" from "{import_base}"')
            if import_base:
                full_name = f'{import_base}.{imported_name.name}'
                print(f'Lookup symbol: {full_name}')
                if full_name in self._symbols:
                    raw_value = self._symbols[full_name]
                    print('\n\n\n')
                    print(f'Setting symbol "{name}" with value: {raw_value}')
                    value = DataFlowValue.of(raw_value)
                    self.set(name, value, ast=statement, import_base=import_base)
                else:
                    self.set_unknown(name, ast=statement, import_base=import_base)
            else:
                import_base = imported_name.name
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
            value = DataFlowValue(type=PythonType.NONE, value=None)
        else:
            value = self.value_from_expression(expression)
        self.return_values.set(value, self.condition)

    def evaluate_condition(self, expression: PythonExpression) -> LogicValue:
        # FIXME this can be improved to actually convert operators into logic
        value = self.value_from_expression(expression)
        if value.is_resolved:
            return TRUE if bool(value.value) else FALSE
        return LogicVariable(data=expression)

    def value_from_expression(self, expression: PythonExpression) -> DataFlowValue:
        assert expression.is_expression
        if expression.is_literal:
            return self.value_from_literal(expression)
        if expression.is_reference:
            return self.value_from_reference(expression)
        if expression.is_item_access:
            return DataFlowValue()
        if expression.is_function_call:
            return self.value_from_function_call(expression)
        if expression.is_star_expression:
            return DataFlowValue(type=PythonType.OBJECT)
        if expression.is_generator:
            return DataFlowValue(type=PythonType.OBJECT)
        if expression.is_operator:
            return self.value_from_operator(expression)
        if expression.is_conditional:
            return DataFlowValue()
        if expression.is_lambda:
            return DataFlowValue(type=PythonType.FUNCTION)
        if expression.is_assignment:
            # Python >= 3.8
            return DataFlowValue()
        if expression.is_yield:
            return DataFlowValue()
        if expression.is_await:
            return DataFlowValue()
        return DataFlowValue()

    def value_from_literal(self, literal: PythonLiteral) -> DataFlowValue:
        assert literal.is_expression and literal.is_literal
        if literal.is_none:
            return DataFlowValue(type=PythonType.OBJECT, value=None)
        if literal.is_bool:
            return DataFlowValue(type=PythonType.BOOL, value=literal.value)
        if literal.is_number:
            if literal.is_int:
                return DataFlowValue(type=PythonType.INT, value=literal.value)
            if literal.is_float:
                return DataFlowValue(type=PythonType.FLOAT, value=literal.value)
            if literal.is_complex:
                return DataFlowValue(type=PythonType.COMPLEX, value=literal.value)
        if literal.is_string:
            return DataFlowValue(type=PythonType.STRING, value=literal.value)
        if literal.is_tuple and not literal.is_comprehension:
            values = tuple(self.value_from_expression(v) for v in literal.values)
            if all(v.is_resolved for v in values):
                values = tuple(v.value for v in values)
                return DataFlowValue(type=PythonType.ITERABLE, value=values)
        if literal.is_list and not literal.is_comprehension:
            values = list(self.value_from_expression(v) for v in literal.values)
            if all(v.is_resolved for v in values):
                # values = list(v.value for v in values)
                return DataFlowValue(type=PythonType.ITERABLE, value=values)
            else:
                return DataFlowValue(type=PythonType.ITERABLE, value=values)
        # TODO FIXME
        return DataFlowValue(type=PythonType.OBJECT)

    def value_from_reference(self, reference: PythonReference) -> DataFlowValue:
        if reference.object is not None:
            obj = self.value_from_expression(reference.object)
            if not obj.type.can_have_attributes:
                return DataFlowValue()
            return DataFlowValue()
        var = self.get(reference.name)
        if not var.has_values or not var.is_deterministic:
            return DataFlowValue()
        assert var.has_base_value
        definition = var.get()
        return definition.value

    def value_from_operator(self, operator: PythonOperator) -> DataFlowValue:
        assert operator.is_operator
        if operator.is_unary:
            return self.value_from_unary_operator(operator)
        if operator.is_binary:
            return self.value_from_binary_operator(operator)
        return DataFlowValue()

    def value_from_unary_operator(self, operator: PythonUnaryOperator) -> DataFlowValue:
        assert operator.is_unary
        value = self.value_from_expression(operator.operand)
        if operator.is_bitwise:
            if not value.type.can_be_int:
                raise DataFlowError.type_check('INT', value.type.name, value)
            if not value.is_resolved:
                return DataFlowValue(type=PythonType.INT)
            r = ~value.value
            return DataFlowValue(type=PythonType.INT, value=r)
        if operator.is_arithmetic:
            if not value.type.can_be_number:
                raise DataFlowError.type_check('NUMBER', value.type.name, value)
            if not value.is_resolved:
                return DataFlowValue(type=PythonType.NUMBER)
            if operator.operator == '+':
                r = +value.value
            elif operator.operator == '-':
                r = -value.value
            else:
                return DataFlowValue(type=PythonType.NUMBER)
            # TODO FIXME refine return type
            return DataFlowValue(type=PythonType.NUMBER, value=r)
        if operator.is_logic:
            if not value.is_resolved:
                return DataFlowValue(type=PythonType.BOOL)
            r = not value.value
            return DataFlowValue(type=PythonType.BOOL, value=r)
        return DataFlowValue()

    def value_from_binary_operator(self, operator: PythonBinaryOperator) -> DataFlowValue:
        assert operator.is_binary
        a = self.value_from_expression(operator.operand1)
        b = self.value_from_expression(operator.operand2)
        o = operator.operator
        if not a.is_resolved:
            return DataFlowValue()
        if not b.is_resolved:
            return DataFlowValue()
        if operator.is_logic:
            if o == 'and':
                r = a.value and b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == 'or':
                r = a.value or b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            return DataFlowValue(type=PythonType.BOOL)
        if operator.is_comparison:
            if o == '==':
                r = a.value == b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == '!=':
                r = a.value != b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == '<':
                r = a.value < b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == '<=':
                r = a.value <= b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == '>':
                r = a.value > b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == '>=':
                r = a.value >= b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == 'in':
                r = a.value in b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == 'not in':
                r = a.value not in b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == 'is':
                r = a.value is b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            if o == 'is not':
                r = a.value is not b.value
                return DataFlowValue(type=PythonType.BOOL, value=r)
            return DataFlowValue(type=PythonType.BOOL)
        if operator.is_arithmetic:
            if o == '+':
                r = a.value + b.value
                return DataFlowValue(type=PythonType.NUMBER, value=r)
            if o == '-':
                r = a.value - b.value
                return DataFlowValue(type=PythonType.NUMBER, value=r)
            if o == '*':
                r = a.value * b.value
                return DataFlowValue(type=PythonType.NUMBER, value=r)
            if o == '/':
                r = a.value / b.value
                return DataFlowValue(type=PythonType.NUMBER, value=r)
            if o == '//':
                r = a.value // b.value
                return DataFlowValue(type=PythonType.INT, value=r)
            if o == '**':
                r = a.value ** b.value
                return DataFlowValue(type=PythonType.NUMBER, value=r)
            return DataFlowValue(type=PythonType.NUMBER)
        return DataFlowValue()

    def value_from_function_call(self, call: PythonFunctionCall) -> DataFlowValue:
        value = self.value_from_expression(call.function)
        if not value.is_resolved:
            return DataFlowValue()
        if not value.type.can_be_function:
            return DataFlowValue()
        function = value.value
        assert isinstance(function, FunctionWrapper), f'not function wrapper: {repr(function)}'
        assert callable(function.call), f'not callable: {repr(function.call)}'
        if not call.arguments:
            result: VariantData[DataFlowValue] = function.call()
            if result.has_values and result.is_deterministic:
                return result.get()
            return DataFlowValue()
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
                    return DataFlowValue()  # FIXME
        result: VariantData[DataFlowValue] = function.call(*args, **kwargs)
        if result.has_values and result.is_deterministic:
            return result.get()
        return DataFlowValue()
