# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, Mapping, Optional

import logging
from pathlib import Path

from attrs import define, field, frozen

from haros.errors import AnalysisError, ParseError
from haros.internal.interface import AnalysisSystemInterface
from haros.metamodel.common import SolverResult
from haros.metamodel.launch import (
    LaunchArgument,
    LaunchDescription,
    LaunchInclusion,
    LaunchModel,
    LaunchNode,
    LaunchSubstitution,
    TextSubstitution,
)
from haros.metamodel.ros import (
    const_mapping,
    const_string,
    RosLaunchResult,
    RosNodeModel,
    uid_node,
    unknown_value,
)

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Interface
###############################################################################


@frozen
class LaunchScope:
    file_path: Path
    args: Dict[str, RosLaunchResult] = field(factory=dict)
    configs: Dict[str, RosLaunchResult] = field(factory=dict)
    anonymous: Dict[str, str] = field(factory=dict)

    def get(self, name: str) -> RosLaunchResult:
        value = self.configs.get(name, self.args.get(name))
        if value is None:
            return unknown_value()  # FIXME maybe raise error
        return value

    def set(self, name: str, value: RosLaunchResult):
        # if name not in self.configs:
        self.configs[name] = value

    def set_unknown(self, name: str):
        return self.set(name, unknown_value())

    def set_text(self, name: str, text: str):
        return self.set(name, const_string(text))

    def resolve(self, sub: Optional[LaunchSubstitution]) -> RosLaunchResult:
        if sub is None or sub.is_unknown:
            return unknown_value()
        if sub.is_text:
            return const_string(sub.value)
        if sub.is_configuration:
            name = sub.name
            value = self.configs.get(name, self.args.get(name))
            if value is None:
                value = self.resolve(sub.default_value)
                self.set(name, value)
            return value
        if sub.is_anon_name:
            value = self.resolve(sub.name)
            if value.is_resolved:
                name = self.anonymous.get(value.value)
                if name is None:
                    name = self.compute_anon_name(value.value)
                    self.anonymous[value.value] = name
                return const_string(name)
            return value
        if sub.is_this_file:
            return const_string(self.get_this_launch_file())
        if sub.is_this_dir:
            return const_string(self.get_this_launch_file_dir())
        if sub.is_concatenation:
            parts = []
            for part in sub.parts:
                value = self.resolve(part)
                if not value.is_resolved:
                    return unknown_value()
                parts.append(value)
            return const_string(''.join(map(str, parts)))
        if sub.is_path_join:
            path = Path()
            for part in sub.parts:
                value = self.resolve(part)
                if not value.is_resolved:
                    return unknown_value()
                path = path / str(value)
            return const_string(path.as_posix())
        return unknown_value()

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
    nodes: List[RosNodeModel] = field(factory=list)
    scope_stack: List[LaunchScope] = field(factory=list)

    @classmethod
    def from_file_path(
        cls,
        file_path: Path,
        system: AnalysisSystemInterface,
    ) -> 'LaunchModelBuilder':
        scopes = [LaunchScope(file_path)]
        return cls(file_path.name, system=system, scope_stack=scopes)

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
            namespace: RosLaunchResult = const_string('/')
        else:
            namespace: RosLaunchResult = self.scope.resolve(include.namespace)
        arguments: Dict[str, LaunchSubstitution] = include.arguments
        file: RosLaunchResult = self.scope.resolve(include.file)
        print(f'included launch file: {include.file}')
        if file.is_resolved:
            try:
                description = self.system.get_launch_description(file.value)
                logger.info(f'parsed included launch file: {file.value}')
            except FileNotFoundError as e:
                logger.warning(str(e))
                return
            except (AnalysisError, ParseError) as e:
                logger.warning(str(e))
                return
            path: Path = Path(file.value)
            # FIXME pass arguments down
            model = model_from_description(path, description, self.system)
            self.nodes.extend(model.nodes)
        else:
            logger.warning(f'unknown launch file inclusion')
            return  # FIXME

    def launch_node(self, node: LaunchNode):
        name: str = self._get_node_name(node.name)
        package: RosLaunchResult = self.scope.resolve(node.package)
        executable: RosLaunchResult = self.scope.resolve(node.executable)
        # namespace: Optional[LaunchSubstitution]
        # remaps: Dict[LaunchSubstitution, LaunchSubstitution]
        for key, sub in node.parameters.items():
            value: RosLaunchResult = self.scope.resolve(sub)
        output: RosLaunchResult = self.scope.resolve(node.output)
        args: RosLaunchResult = const_list([
            self.scope.resolve(arg) for arg in node.arguments
        ])
        params: RosLaunchResult = const_mapping({})
        for key, sub in node.parameters.items():
            params.value[key] = self.scope.resolve(sub)
        node_id = uid_node(str(package), str(executable))
        if package.is_resolved and executable.is_resolved:
            fsnode = self.system.get_node_model(package, executable)  # FIXME
            if fsnode is None:
                logger.warning(f'unable to find node: {package}/{executable}')
            # TODO add ROS client library calls
        rosname = const_string(name)
        rosnode = RosNodeModel(
            rosname,
            const_string(node_id),
            arguments=args,
            parameters=params,
            output=output,
        )
        self.nodes.append(rosnode)

    def _get_node_name(self, name: Optional[LaunchSubstitution]) -> str:
        if name is None:
            return 'anonymous'  # FIXME
        value: RosLaunchResult = self.scope.resolve(name)
        return value.value if value.is_resolved else '$(?)'


def model_from_description(
    path: Path,
    description: LaunchDescription,
    system: AnalysisSystemInterface,
) -> LaunchModel:
    logger.info(f'model_from_description({path})')
    builder = LaunchModelBuilder.from_file_path(path, system)
    for entity in description.entities:
        if entity.is_argument:
            builder.declare_argument(entity)
        elif entity.is_inclusion:
            builder.include_launch(entity)
        elif entity.is_node:
            builder.launch_node(entity)
    return builder.build()
