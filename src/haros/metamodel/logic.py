# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, Iterable, Mapping, Tuple

from attrs import field, frozen

###############################################################################
# Logic
###############################################################################


@frozen
class LogicValue(object):
    @property
    def is_true(self) -> bool:
        return False

    @property
    def is_false(self) -> bool:
        return False

    @property
    def is_variable(self) -> bool:
        return False

    @property
    def is_atomic(self) -> bool:
        return self.is_true or self.is_false or self.is_variable

    @property
    def is_not(self) -> bool:
        return False

    @property
    def is_and(self) -> bool:
        return False

    @property
    def is_or(self) -> bool:
        return False

    def negate(self) -> 'LogicValue':
        return LogicNot(self)

    def join(self, value: 'LogicValue') -> 'LogicValue':
        if value.is_true:
            return self
        if value.is_false:
            return value
        if value.is_and:
            return value.join(self)
        return LogicAnd((self, value))

    def disjoin(self, value: 'LogicValue') -> 'LogicValue':
        if value.is_true:
            return value
        if value.is_false:
            return self
        if value.is_or:
            return value.disjoin(self)
        return LogicOr((self, value))

    def implies(self, value: 'LogicValue') -> 'LogicValue':
        # (p -> q) == (!p | q)
        if value.is_true:
            return value
        neg = self.negate()
        if value.is_false:
            return neg
        return value.disjoin(neg)

    def simplify(self) -> 'LogicValue':
        return self

    def variables(self) -> None:
        yield from ()

    def replace(self, valuation: Mapping[str, bool]) -> 'LogicValue':
        # valuation: variable name -> bool
        return self

    def serialize(self) -> Any:
        raise NotImplementedError()

    @classmethod
    def deserialize(cls, data: Any) -> 'LogicValue':
        if data is True:
            return LogicValue.T
        if data is False:
            return LogicValue.F
        if isinstance(data, dict):
            return LogicVariable(data['text'], data['data'], name=data['name'])
        if isinstance(data, list):
            if data[0] == 'not':
                return LogicNot(LogicValue.from_json(data[1]))
            args = list(map(LogicValue.from_json, data[1:]))
            if data[0] == 'and':
                return LogicAnd(args)
            if data[0] == 'or':
                return LogicOr(args)
        raise TypeError(f'unexpected data type: {data!r}')


@frozen
class LogicTrue(LogicValue):
    @property
    def is_true(self) -> bool:
        return True

    def negate(self) -> LogicValue:
        return LOGIC_FALSE

    def join(self, value: LogicValue) -> LogicValue:
        return value

    def disjoin(self, value: LogicValue) -> LogicValue:
        return self

    def implies(self, value: LogicValue) -> LogicValue:
        return value

    def serialize(self) -> Any:
        return True

    def __str__(self) -> str:
        return 'True'


TRUE: Final[LogicValue] = LogicTrue()
LogicValue.T = LOGIC_TRUE


@frozen
class LogicFalse(LogicValue):
    @property
    def is_false(self) -> bool:
        return True

    def negate(self) -> LogicValue:
        return LOGIC_TRUE

    def join(self, value: LogicValue) -> LogicValue:
        return self

    def disjoin(self, value: LogicValue) -> LogicValue:
        return value

    def implies(self, value: LogicValue) -> LogicValue:
        return LOGIC_TRUE

    def serialize(self) -> Any:
        return False

    def __str__(self) -> str:
        return 'False'


FALSE: Final[LogicValue] = LogicFalse()
LogicValue.F = LOGIC_FALSE

__id_counter: int = 0


@frozen
class LogicVariable(LogicValue):
    data: Any
    name: str = ''

    def __attrs_post_init__(self):
        if not self.name:
            global __id_counter
            n = __id_counter
            __id_counter += 1
            object.__setattr__(self, 'name', f'@{n}')

    @property
    def is_variable(self) -> bool:
        return True

    def variables(self) -> None:
        yield self

    def replace(self, valuation: Mapping[str, bool]) -> LogicValue:
        # valuation: variable name -> bool
        value = valuation.get(self.name)
        if value is True:
            return LogicValue.T
        if value is False:
            return LogicValue.F
        return self

    def serialize(self) -> Any:
        try:
            data = self.data.serialize()
        except AttributeError:
            data = str(self.data)
        return {
            'name': self.name,
            'data': data,
        }

    def __str__(self) -> str:
        return self.name or self.text


@frozen
class LogicNot(LogicValue):
    operand: LogicValue

    @property
    def is_not(self) -> bool:
        return True

    def negate(self) -> LogicValue:
        return self.operand

    def simplify(self) -> LogicValue:
        if self.operand.is_not:
            return self.operand.operand.simplify()
        operand = self.operand.simplify()
        if operand.is_true:
            return LogicValue.F
        if operand.is_false:
            return LogicValue.T
        if operand.is_and:
            operands = [LogicNot(x).simplify() for x in operand.operands]
            return LogicOr(operands)
        if operand.is_or:
            operands = [LogicNot(x).simplify() for x in operand.operands]
            return LogicAnd(operands)
        return self

    def variables(self) -> None:
        yield from self.operand.variables()

    def replace(self, valuation: Mapping[str, bool]) -> LogicValue:
        # valuation: variable name -> bool
        operand = self.operand.replace(valuation)
        result = LogicNot(operand)
        return result.simplify()

    def serialize(self) -> Any:
        return ['not', self.operand.serialize()]

    def __str__(self) -> str:
        return f'(not {self.operand})'


@frozen
class LogicAnd(LogicValue):
    operands: Tuple[LogicValue] = field(converter=tuple)

    @property
    def is_and(self) -> bool:
        return True

    def join(self, value: LogicValue) -> LogicValue:
        if value.is_true:
            return self
        if value.is_false:
            return value
        operands = list(self.operands)
        if value.is_and:
            operands.extend(value.operands)
        else:
            operands.append(value)
        return LogicAnd(operands)

    def simplify(self) -> LogicValue:
        operands = set()
        for x in self.operands:
            y = x.simplify()
            if y.is_true:
                continue
            if y.is_false:
                return LogicValue.F
            if y.is_and:
                operands.update(y.operands)
            else:
                operands.add(y)
        if not operands:
            return LogicValue.T
        # there are no duplicates, LogicTrue, LogicFalse, or LogicAnd here
        operands = list(operands)
        if self._is_contradiction(operands):
            return LogicValue.F
        self._trim_larger_ors(operands)
        return LogicAnd(operands) if len(operands) > 1 else operands[0]

    def variables(self) -> None:
        for x in self.operands:
            yield from x.variables()

    def replace(self, valuation: Mapping[str, bool]) -> LogicValue:
        # valuation: variable name -> bool
        operands = [op.replace(valuation) for op in self.operands]
        result = LogicAnd(operands)
        return result.simplify()

    def serialize(self) -> Any:
        return ['and'] + [arg.serialize() for arg in self.operands]

    def _is_contradiction(self, operands: Iterable[LogicValue]) -> bool:
        for i in range(len(operands) - 1):
            x = operands[i].negate()
            for j in range(i + 1, len(operands)):
                if x == operands[j]:
                    return True
        return False

    def _trim_larger_ors(self, operands: Iterable[LogicValue]):
        for i in range(len(operands) - 2, -1, -1):
            x = operands[i]
            if not x.is_or:
                continue
            xs = set(x.operands)
            for j in range(len(operands) - 1, i, -1):
                y = operands[j]
                if not y.is_or:
                    continue
                ys = set(y.operands)
                common = xs & ys
                if common == xs:
                    del operands[j]
                elif common == ys:
                    del operands[i]
                    break

    def __str__(self) -> str:
        if not self.operands:
            return 'True'
        if len(self.operands) == 1:
            for x in self.operands:
                return str(x)
        return f"({' and '.join(str(x) for x in self.operands)})"


@frozen
class LogicOr(LogicValue):
    operands: Tuple[LogicValue] = field(converter=tuple)

    @property
    def is_or(self) -> bool:
        return True

    def disjoin(self, value: LogicValue) -> LogicValue:
        if value.is_true:
            return value
        if value.is_false:
            return self
        operands = list(self.operands)
        if value.is_or:
            operands.extend(value.operands)
        else:
            operands.append(value)
        return LogicOr(operands)

    def simplify(self) -> LogicValue:
        operands = set()
        for x in self.operands:
            y = x.simplify()
            if y.is_false:
                continue
            if y.is_true:
                return LogicValue.T
            if y.is_or:
                operands.update(y.operands)
            else:
                operands.add(y)
        if not operands:
            return LogicValue.T
        # there are no duplicates, LogicTrue, LogicFalse, or LogicOr here
        operands = list(operands)
        if self._is_tautology(operands):
            return LogicValue.T
        self._trim_larger_ands(operands)
        return LogicOr(operands) if len(operands) > 1 else operands[0]

    def variables(self) -> None:
        for x in self.operands:
            yield from x.variables()

    def replace(self, valuation: Mapping[str, bool]) -> LogicValue:
        # valuation: variable name -> bool
        operands = [op.replace(valuation) for op in self.operands]
        result = LogicOr(operands)
        return result.simplify()

    def serialize(self) -> Any:
        return ['or'] + [arg.serialize() for arg in self.operands]

    def _is_tautology(self, operands: Iterable[LogicValue]) -> bool:
        for i in range(len(operands) - 1):
            x = operands[i].negate()
            for j in range(i + 1, len(operands)):
                if x == operands[j]:
                    return True
        return False

    def _trim_larger_ands(self, operands: Iterable[LogicValue]):
        for i in range(len(operands) - 2, -1, -1):
            x = operands[i]
            if not x.is_and:
                continue
            xs = set(x.operands)
            for j in range(len(operands) - 1, i, -1):
                y = operands[j]
                if not y.is_and:
                    continue
                ys = set(y.operands)
                common = xs & ys
                if common == xs:
                    del operands[j]
                elif common == ys:
                    del operands[i]
                    break

    def __str__(self) -> str:
        if not self.operands:
            return 'True'
        if len(self.operands) == 1:
            for x in self.operands:
                return str(x)
        return f"({' or '.join(str(x) for x in self.operands)})"
