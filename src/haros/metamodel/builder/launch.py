# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Dict, Final, List, Optional

from enum import Enum
import logging

from attrs import define, field, frozen

from haros.metamodel.launch import (
    LaunchArgument,
    LaunchDescription,
    LaunchInclusion,
    LaunchModel,
    LaunchNode,
    LaunchSubstitution,
    TextSubstitution,
)
from haros.metamodel.ros import RosName

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Interface
###############################################################################


@frozen
class UnknownValue:
    def __str__(self) -> str:
        return '(?)'


UNKNOWN_TOKEN: Final[UnknownValue] = UnknownValue()


class LaunchValueType(Enum):
    BOOL = 'bool'
    INT = 'int'
    DOUBLE = 'double'
    STRING = 'string'
    YAML = 'yaml'
    AUTO = 'auto'
    OBJECT = 'object'


@frozen
class LaunchValue:
    type: LaunchValueType = field(default=LaunchValueType.STRING)
    value: Any = field(default=UNKNOWN_TOKEN)

    @property
    def is_resolved(self) -> bool:
        return not isinstance(self.value, UnknownValue)

    @classmethod
    def type_bool(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.BOOL, value)

    @classmethod
    def type_int(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.INT, value)

    @classmethod
    def type_double(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.DOUBLE, value)

    @classmethod
    def type_string(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.STRING, value)

    @classmethod
    def type_yaml(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.YAML, value)

    @classmethod
    def type_auto(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.AUTO, value)

    @classmethod
    def type_object(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.OBJECT, value)

    def __str__(self) -> str:
        return str(self.value)


@frozen
class AnalysisSystemInterface:
    packages: Dict[str, str] = field(factory=dict)


@frozen
class LaunchScope:
    args: Dict[str, LaunchValue] = field(factory=dict)
    configs: Dict[str, LaunchValue] = field(factory=dict)

    def get(self, name: str) -> LaunchValue:
        value = self.configs.get(name, self.args.get(name))
        if value is None:
            return LaunchValue()  # FIXME maybe raise error
        return value

    def set(self, name: str, value: LaunchValue):
        # if name not in self.configs:
        self.configs[name] = value

    def set_unknown(self, name: str):
        return self.set(name, LaunchValue())

    def set_text(self, name: str, text: str):
        return self.set(name, LaunchValue.type_string(text))

    def resolve(self, sub: Optional[LaunchSubstitution]) -> LaunchValue:
        if sub is None or sub.is_unknown:
            return LaunchValue()
        if sub.is_text:
            return LaunchValue.type_string(sub.value)
        if sub.is_configuration:
            return self.get(sub.name)
        return LaunchValue()

    def duplicate(self) -> 'LaunchScope':
        # LaunchArgument is defined globally
        # LaunchConfiguration is scoped
        return LaunchScope(args=self.args, configs=dict(self.configs))


@define
class LaunchModelBuilder:
    name: str
    system: AnalysisSystemInterface = field(factory=AnalysisSystemInterface)
    nodes: List[LaunchNode] = field(factory=list)
    _scope_stack: List[LaunchScope] = field(factory=lambda: [LaunchScope()])

    @property
    def scope(self) -> LaunchScope:
        return self._scope_stack[-1]

    @property
    def root(self) -> LaunchScope:
        return self._scope_stack[0]

    def build(self) -> LaunchModel:
        return LaunchModel(self.name, nodes=list(self.nodes))

    def enter_group(self):
        self._scope_stack.append(self.scope.duplicate())

    def exit_group(self):
        self._scope_stack.pop()

    def declare_argument(self, name: str, default_value: Optional[LaunchSubstitution] = None):
        # NOTE: probably better to separate LaunchValue from LaunchSubstitution...
        self.root.args[name] = self.scope.resolve(default_value)


def model_from_description(name: str, description: LaunchDescription) -> LaunchModel:
    builder = LaunchModelBuilder(name)
    for entity in description.entities:
        if entity.is_argument:
            builder.declare_argument(entity.name, entity.default_value)
        elif entity.is_inclusion:
            pass
        elif entity.is_node:
            pass
    return builder.build()
