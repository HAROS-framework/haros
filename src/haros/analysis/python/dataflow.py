# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Final, Optional, Type

from collections.abc import Mapping, Sequence
import logging

from attrs import define, evolve, field, frozen

from haros.errors import DataFlowError
from haros.metamodel.common import SourceCodeLocation, TrackedCode, VariantData
from haros.metamodel.logic import FALSE, TRUE, LogicValue, LogicVariable
from haros.metamodel.result import Result, T, TypeToken
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

logger: Final[logging.Logger] = logging.getLogger(__name__)

BUILTINS_MODULE: Final[str] = '__builtins__'

BUILTIN_FUNCTIONS: Final[list[str]] = [
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
    # 'open',
    'ord',
    'pow',
    # 'print',
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
    # '__import__',
]

BUILTIN_CLASSES: Final[list[str]] = [
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

BUILTIN_EXCEPTIONS: Final[list[str]] = [
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


@define
class MockObject:
    pass  # marker interface


@frozen
class StrictFunctionCaller(Callable):
    function: Callable
    name: str = ''
    module: str = ''

    def __call__(self, *args: Result[Any], **kwargs: Result[Any]) -> Result[Any]:
        for arg in args:
            if not arg.is_resolved:
                return Result.unknown_value()
        for arg in kwargs.values():
            if not arg.is_resolved:
                return Result.unknown_value()
        raw_args = [arg.value for arg in args]
        raw_kwargs = {key: arg.value for key, arg in kwargs.items()}
        raw_value = self.function(*raw_args, **raw_kwargs)
        if isinstance(raw_value, list):
            value = [Result.of(v) for v in raw_value]
            return Result.of_list(value)
        if isinstance(raw_value, tuple):
            value = tuple(Result.of(v) for v in raw_value)
            return Result.of_tuple(value)
        if isinstance(raw_value, dict):
            value = {Result.of(k): Result.of(v) for k, v in raw_value.items()}
            return Result.of_dict(value)
        return Result.of(raw_value)


def tracked(ast: PythonAst) -> Optional[TrackedCode]:
    location = SourceCodeLocation(
        file=ast.meta.annotations.get('file'),
        package=ast.meta.annotations.get('package'),
        line=ast.line,
        column=ast.column,
        language='Python',
    )
    return TrackedCode(ast, location)


@frozen
class Definition[V]:
    value: Result[V]
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
    def of_builtin_function(cls, name: str) -> 'Definition[Type[min]]':
        # value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert callable(raw_value), f'expected function, got: {raw_value!r}'
        wrapper = StrictFunctionCaller(raw_value, name=name, module=BUILTINS_MODULE)
        value = Result.of_builtin_function(value=wrapper)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def of_builtin_class(cls, name: str) -> 'Definition[Type]':
        # value = getattr(__builtins__, name)
        raw_value = __builtins__.get(name)
        assert isinstance(raw_value, type), f'expected class, got: {raw_value!r}'
        wrapper = StrictFunctionCaller(raw_value, name=name, module=BUILTINS_MODULE)
        value: Result[Type] = Result.of_class(value=wrapper)
        return cls(value, import_base=BUILTINS_MODULE)

    @classmethod
    def from_value(cls, raw_value: V, ast: Optional[PythonAst] = None) -> 'Definition[V]':
        return cls(Result.of(raw_value), ast=ast)  # FIXME TrackedCode from ast

    def cast_to(self, type: TypeToken[T]) -> 'Definition[T]':
        if self.value.type == type:
            return self
        # new_type_mask = self.value.type.mask & type.mask
        # if bool(new_type_mask):
        #     new_type = evolve(type, mask=new_type_mask)
        #     return evolve(self, value=self.value.cast_to(new_type))
        # raise TypeError(f'unable to cast {self.value.type} to {type}')
        return evolve(self, value=self.value.cast_to(type))

    def __str__(self) -> str:
        if self.import_base:
            return f'{self.value} (import from {self.import_base})'
        return f'{self.value}'


###############################################################################
# Interface
###############################################################################


def _default_return_value() -> VariantData[Result[Any]]:
    return VariantData.with_base_value(Result.of_none())


@define
class DataScope:
    variables: Mapping[str, VariantData[Definition]] = field(factory=dict)
    return_values: VariantData[Result[Any]] = field(factory=_default_return_value)
    _condition_stack: Sequence[LogicValue] = field(init=False, factory=list)
    _symbols: Mapping[str, Any] = field(factory=dict)

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
            variables[name] = VariantData.with_base_value(Definition.of_builtin_class(name))
        return cls(variables=variables)

    def duplicate(self) -> 'DataScope':
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
                    value = Result.of(raw_value)
                    self.set(name, value, ast=statement, import_base=import_base)
                else:
                    self.set(name, Result.unknown_value(), ast=statement, import_base=import_base)
            else:
                import_base = imported_name.name
                if import_base in self._symbols:
                    raw_value = self._symbols[import_base]
                    value = Result.of(raw_value)
                    self.set(name, value, ast=statement, import_base=import_base)
                else:
                    self.set(name, Result.unknown_value(), ast=statement, import_base=import_base)

    def add_function_def(
        self,
        statement: PythonFunctionDefStatement,
        fun: Optional[Callable] = None,
    ):
        assert statement.is_statement and statement.is_function_def
        if fun is None:
            self.set(statement.name, Result.of_def_function(), ast=statement)
        else:
            self.set(statement.name, Result.of_def_function(fun), ast=statement)

    def add_class_def(self, statement: PythonClassDefStatement):
        assert statement.is_statement and statement.is_class_def
        self.set(statement.name, Result.of_class(), ast=statement)

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
            value = Result.of_none()
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
            return Result.of_iterable(source=tracked(expression))
        if expression.is_generator:
            return Result.of_iterable(source=tracked(expression))
        if expression.is_operator:
            return self.value_from_operator(expression)
        if expression.is_conditional:
            return Result.unknown_value(source=tracked(expression))
        if expression.is_lambda:
            return Result.of_def_function(source=tracked(expression))
        if expression.is_assignment:
            # Python >= 3.8
            return Result.unknown_value(source=tracked(expression))
        if expression.is_yield:
            return Result.unknown_value(source=tracked(expression))
        if expression.is_await:
            return Result.unknown_value(source=tracked(expression))
        return Result.unknown_value(source=tracked(expression))

    def value_from_literal(self, literal: PythonLiteral) -> Result:
        assert literal.is_expression and literal.is_literal
        if literal.is_none:
            return Result.of_none(source=tracked(literal))
        if literal.is_bool:
            return Result.of_bool(literal.value, source=tracked(literal))
        if literal.is_number:
            if literal.is_int:
                return Result.of_int(literal.value, source=tracked(literal))
            if literal.is_float:
                return Result.of_float(literal.value, source=tracked(literal))
            if literal.is_complex:
                return Result.of_complex(literal.value, source=tracked(literal))
        if literal.is_string:
            return Result.of_string(literal.value, source=tracked(literal))
        if literal.is_tuple and not literal.is_comprehension:
            values = tuple(self.value_from_expression(v) for v in literal.values)
            # if all(v.is_resolved for v in values):
            #     values = tuple(v.value for v in values)
            return Result.of_tuple(values, source=tracked(literal))
        if literal.is_list and not literal.is_comprehension:
            values = list(self.value_from_expression(v) for v in literal.values)
            # if all(v.is_resolved for v in values):
            #     values = list(v.value for v in values)
            return Result.of_list(values, source=tracked(literal))
        if literal.is_dict and not literal.is_comprehension:
            entries = dict(
                (self.value_from_expression(e.key), self.value_from_expression(e.value))
                for e in literal.entries
            )
            # if not all(key.is_resolved for key in entries.keys()):
            return Result.of_dict(entries, source=tracked(literal))
        # TODO FIXME
        return Result.unknown_value(source=tracked(literal))

    def value_from_reference(self, reference: PythonReference) -> Result:
        if reference.object is not None:
            obj: Result = self.value_from_expression(reference.object)
            if not obj.type.has_attributes or not obj.is_resolved:
                logger.warning(f'object without attributes: {obj} # {reference.object}')
                return Result.unknown_value(source=obj.source)
            value = getattr(obj.value, reference.name)
            if callable(value):
                if not isinstance(value, StrictFunctionCaller):
                    if not isinstance(obj.value, MockObject):
                        module = type(obj.value).__name__
                        value = StrictFunctionCaller(value, name=reference.name, module=module)
            return Result.of(value, source=tracked(reference))
        var = self.get(reference.name)
        if not var.has_values or not var.is_deterministic:
            # FIXME to have multiple values associated with different conditions
            # handle `not var.is_deterministic` separately
            # (return `VariantData[Result]` instead of `Result`)
            return Result.unknown_value(source=tracked(reference))
        assert var.has_base_value
        definition = var.get()
        return definition.value

    def value_from_operator(self, operator: PythonOperator) -> Result:
        assert operator.is_operator
        if operator.is_unary:
            return self.value_from_unary_operator(operator)
        if operator.is_binary:
            return self.value_from_binary_operator(operator)
        return Result.unknown_value(source=tracked(operator))

    def value_from_unary_operator(self, operator: PythonUnaryOperator) -> Result:
        assert operator.is_unary
        value = self.value_from_expression(operator.operand)
        if operator.is_bitwise:
            if not value.type.is_int:
                raise DataFlowError.type_check('INT', value.type.name, value)
            if not value.is_resolved:
                return Result.of_int(source=tracked(operator))
            r = ~value.value
            return Result.of_int(r, source=tracked(operator))
        if operator.is_arithmetic:
            if not value.type.is_number:
                raise DataFlowError.type_check('NUMBER', value.type.name, value)
            if not value.is_resolved:
                return Result.of_number(source=tracked(operator))
            if operator.operator == '+':
                r = +value.value
            elif operator.operator == '-':
                r = -value.value
            else:
                return Result.of_number(source=tracked(operator))
            # TODO FIXME refine return type
            return Result.of_number(r, source=tracked(operator))
        if operator.is_logic:
            if not value.is_resolved:
                return Result.of_bool(source=tracked(operator))
            r = not value.value
            return Result.of_bool(r, source=tracked(operator))
        return Result.unknown_value(source=tracked(operator))

    def value_from_binary_operator(self, operator: PythonBinaryOperator) -> Result[Any]:
        assert operator.is_binary
        a = self.value_from_expression(operator.operand1)
        b = self.value_from_expression(operator.operand2)
        o = operator.operator
        if not a.is_resolved:
            return Result.unknown_value(source=tracked(operator))
        if not b.is_resolved:
            return Result.unknown_value(source=tracked(operator))
        if operator.is_logic:
            if o == 'and':
                return Result.of_bool(a.value and b.value, source=tracked(operator))
            if o == 'or':
                return Result.of_bool(a.value or b.value, source=tracked(operator))
            return Result.of_bool(source=tracked(operator))
        if operator.is_comparison:
            if o == '==':
                return Result.of_bool(a.value == b.value, source=tracked(operator))
            if o == '!=':
                return Result.of_bool(a.value != b.value, source=tracked(operator))
            if o == '<':
                return Result.of_bool(a.value < b.value, source=tracked(operator))
            if o == '<=':
                return Result.of_bool(a.value <= b.value, source=tracked(operator))
            if o == '>':
                return Result.of_bool(a.value > b.value, source=tracked(operator))
            if o == '>=':
                return Result.of_bool(a.value >= b.value, source=tracked(operator))
            if o == 'in':
                return Result.of_bool(a.value in b.value, source=tracked(operator))
            if o == 'not in':
                return Result.of_bool(a.value not in b.value, source=tracked(operator))
            if o == 'is':
                return Result.of_bool(a.value is b.value, source=tracked(operator))
            if o == 'is not':
                return Result.of_bool(a.value is not b.value, source=tracked(operator))
            return Result.of_bool(source=tracked(operator))
        if operator.is_arithmetic:
            if o == '+':
                r = a.value + b.value
                if isinstance(r, (int, float, complex)):
                    return Result.of_number(r, source=tracked(operator))
                if isinstance(r, str):
                    return Result.of_string(r, source=tracked(operator))
                if isinstance(r, list):
                    return Result.of_list(r, source=tracked(operator))
                if isinstance(r, tuple):
                    return Result.of_tuple(r, source=tracked(operator))
                return Result.unknown_value(source=tracked(operator))
            if o == '-':
                return Result.of_number(a.value - b.value, source=tracked(operator))
            if o == '*':
                return Result.of_number(a.value * b.value, source=tracked(operator))
            if o == '/':
                return Result.of_number(a.value / b.value, source=tracked(operator))
            if o == '//':
                return Result.of_int(a.value // b.value, source=tracked(operator))
            if o == '**':
                return Result.of_number(a.value**b.value, source=tracked(operator))
            return Result.of_number(source=tracked(operator))
        return Result.unknown_value(source=tracked(operator))

    def value_from_item_access(self, access: PythonItemAccess) -> Result:
        obj: Result = self.value_from_expression(access.object)
        if not obj.is_resolved or not obj.type.has_items:
            return Result.unknown_value(source=tracked(access))
        if access.key.is_slice:
            return Result.unknown_value(source=tracked(access))
        assert access.key.is_key
        key: Result = self.value_from_expression(access.key.expression)
        if not key.is_resolved:
            return Result.unknown_value(source=tracked(access))
        try:
            value = obj.value[key.value]
        except KeyError:
            return Result.unknown_value(source=tracked(access))
        if isinstance(value, Result):
            return value
        return Result.of(value, source=tracked(access))

    def value_from_function_call(self, call: PythonFunctionCall) -> Result:
        function = self.value_from_expression(call.function)
        if not function.is_resolved:
            logger.debug(f'function is not resolved ({function.type}): {call.function}')
            return Result.unknown_value(source=tracked(call))
        # if not function.type.is_function and not function.type.is_class:
        if not function.is_callable:
            logger.warning(f'function is not callable: {function} ({function.type})')
            return Result.unknown_value(source=tracked(call))
        assert callable(function.value), f'not callable: {repr(function.value)}'
        if not call.arguments:
            return function.call().trace_to(tracked(call))
        args: list[Result[Any]] = []
        kwargs: dict[str, Result[Any]] = {}
        for argument in call.arguments:
            arg: Result[Any] = self.value_from_expression(argument.value)
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
                    logger.warning(f'use of double star arguments for {function}')
                    return Result.unknown_value(source=tracked(call))  # FIXME
        return function.call(*args, **kwargs).trace_to(tracked(call))
