# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, Mapping, Optional

from enum import Enum
import logging
from pathlib import Path

from attrs import define, field, frozen

from haros.internal.interface import AnalysisSystemInterface
from haros.metamodel.launch import (
    LaunchArgument,
    LaunchDescription,
    LaunchInclusion,
    LaunchModel,
    LaunchNode,
    LaunchSubstitution,
    TextSubstitution,
)
# from haros.metamodel.ros import RosName

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
        return cls(type=LaunchValueType.BOOL, value=value)

    @classmethod
    def type_int(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.INT, value=value)

    @classmethod
    def type_double(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.DOUBLE, value=value)

    @classmethod
    def type_string(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.STRING, value=value)

    @classmethod
    def type_yaml(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.YAML, value=value)

    @classmethod
    def type_auto(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.AUTO, value=value)

    @classmethod
    def type_object(cls, value: str) -> 'LaunchValue':
        # TODO validate values
        return cls(type=LaunchValueType.OBJECT, value=value)

    def __str__(self) -> str:
        return str(self.value)


@frozen
class LaunchScope:
    file_path: Path
    args: Dict[str, LaunchValue] = field(factory=dict)
    configs: Dict[str, LaunchValue] = field(factory=dict)
    anonymous: Dict[str, str] = field(factory=dict)

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
        if sub.is_anon_name:
            value = self.resolve(sub.name)
            if value.is_resolved:
                name = self.anonymous.get(value.value)
                if name is None:
                    name = self.compute_anon_name(value.value)
                    self.anonymous[value.value] = name
                return LaunchValue.type_string(name)
            return value
        if sub.is_this_file:
            return LaunchValue.type_string(self.get_this_launch_file())
        if sub.is_this_dir:
            return LaunchValue.type_string(self.get_this_launch_file_dir())
        return LaunchValue()

    def duplicate(self) -> 'LaunchScope':
        # LaunchArgument is defined globally
        # LaunchConfiguration is scoped
        return LaunchScope(args=self.args, configs=dict(self.configs))

    def compute_anon_name(self, name: str) -> str:
        # as seen in the official distribution
        import os
        import random
        import socket
        import sys
        name = f'{name}_{socket.gethostname()}_{os.getpid()}_{random.randint(0, sys.maxsize)}'
        name = name.replace('.', '_')
        name = name.replace('-', '_')
        return name.replace(':', '_')

    def get_this_launch_file(self) -> str:
        return self.file_path.as_posix()

    def get_this_launch_file_dir(self) -> str:
        return self.file_path.parent.as_posix()


@define
class LaunchModelBuilder:
    name: str
    system: AnalysisSystemInterface = field(factory=AnalysisSystemInterface)
    nodes: List[LaunchNode] = field(factory=list)
    scope_stack: List[LaunchScope] = field(factory=list)

    @classmethod
    def from_file_path(cls, file_path: Path) -> 'LaunchModelBuilder':
        scopes = [LaunchScope(file_path)]
        return cls(file_path.name, scope_stack=scopes)

    @property
    def scope(self) -> LaunchScope:
        return self.scope_stack[-1]

    @scope.setter
    def scope(self, scope: LaunchScope):
        self.scope_stack[-1] = scope

    @property
    def root(self) -> LaunchScope:
        return self.scope_stack[0]

    def build(self) -> LaunchModel:
        return LaunchModel(self.name, nodes=list(self.nodes))

    def enter_group(self):
        self.scope_stack.append(self.scope.duplicate())

    def exit_group(self):
        self.scope_stack.pop()

    def declare_argument(self, arg: LaunchArgument):
        name: str = arg.name
        default_value: Optional[LaunchSubstitution] = arg.default_value
        self.root.args[name] = self.scope.resolve(default_value)

    def include_launch(self, include: LaunchInclusion):
        if include.namespace is None:
            namespace: LaunchValue = LaunchValue.type_string('/')
        else:
            namespace: LaunchValue = self.scope.resolve(include.namespace)
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
        # namespace: Optional[LaunchSubstitution]
        # remaps: Dict[LaunchSubstitution, LaunchSubstitution]
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


def model_from_description(path: Path, description: LaunchDescription) -> LaunchModel:
    builder = LaunchModelBuilder.from_file_path(path)
    for entity in description.entities:
        if entity.is_argument:
            builder.declare_argument(entity)
        elif entity.is_inclusion:
            builder.include_launch(entity)
        elif entity.is_node:
            builder.launch_node(entity)
    return builder.build()
