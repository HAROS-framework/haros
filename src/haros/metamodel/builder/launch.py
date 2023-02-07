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

    def get_launch_description(self, path: str) -> LaunchDescription:
        return None


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

    def declare_argument(self, arg: LaunchArgument):
        name: str = arg.name
        default_value: Optional[LaunchSubstitution] = arg.default_value
        self.root.args[name] = self.scope.resolve(default_value)

    def include_launch(self, include: LaunchInclusion):
        namespace: RosName = include.namespace
        arguments: Dict[str, LaunchSubstitution] = include.arguments
        # TODO
        file: LaunchValue = self.scope.resolve(include.file)
        if file.is_resolved:
            description = self.system.get_launch_description(file.value)
            if description is None:
                return  # FIXME
            model = model_from_description(file.value, description)
            # FIXME pass arguments down
        else:
            return  # FIXME

    def launch_node(self, node: LaunchNode):
        name: str = self._get_node_name(node.name)
        package: LaunchValue = self.scope.resolve(node.package)
        executable: LaunchValue = self.scope.resolve(node.executable)
        # namespace: RosName = field(factory=RosName.global_namespace)
        # remaps: Dict[RosName, RosName] = field(factory=dict)
        for key, sub in node.parameters.items():
            value: LaunchValue = self.scope.resolve(sub)
        output: LaunchValue = self.scope.resolve(node.output)
        args: List[LaunchValue] = [self.scope.resolve(arg) for arg in node.arguments]
        params: Dict[str, LaunchValue] = {}
        for key, sub in node.parameters.items():
            params[key] = self.scope.resolve(sub)
        rosnode = self.system.get_ros_node(package, executable)  # FIXME
        instance = RosNodeInstance(name, rosnode, args, params, output)  # FIXME

    def _get_node_name(self, name: Optional[LaunchSubstitution]) -> str:
        if name is None:
            return 'anonymous'  # FIXME
        value: LaunchValue = self.scope.resolve(name)
        return value.value if value.is_resolved else '$(?)'


def model_from_description(name: str, description: LaunchDescription) -> LaunchModel:
    builder = LaunchModelBuilder(name)
    for entity in description.entities:
        if entity.is_argument:
            builder.declare_argument(entity)
        elif entity.is_inclusion:
            builder.include_launch(entity)
        elif entity.is_node:
            builder.launch_node(entity)
    return builder.build()
