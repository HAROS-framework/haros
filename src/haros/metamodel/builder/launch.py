# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, Optional, Set, Tuple

import logging
from pathlib import Path

from attrs import define, evolve, field, frozen

from haros.analysis.python.dataflow import unknown_value
from haros.errors import AnalysisError, ParseError
from haros.internal.interface import AnalysisSystemInterface
from haros.metamodel.common import Resolved, Result, TrackedCode, UnresolvedMapping
from haros.metamodel.launch import (
    ArgumentFeature,
    FeatureId,
    LaunchArgument,
    LaunchArgumentValueType,
    LaunchDescription,
    LaunchEntity,
    LaunchFileFeature,
    LaunchGroupAction,
    LaunchInclusion,
    LaunchNode,
    LaunchNodeParameterList,
    LaunchSubstitution,
    NodeFeature,
)
from haros.metamodel.ros import RosNodeModel

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

BOOL_VALUES: Final[Tuple[str]] = ('true', 'false')

###############################################################################
# Interface
###############################################################################


@define
class ArgumentFeatureBuilder:
    id: FeatureId
    name: str
    # value of this argument - given via command line or computed from default
    value: Result[str] = field(factory=Result.unknown_value)
    # default value, defined on declaration
    default_value: Optional[Result[str]] = None
    description: Optional[Result[str]] = None
    # type computed from the current value
    inferred_type: LaunchArgumentValueType = LaunchArgumentValueType.STRING
    # affects_cg: bool = False

    def set(self, value: Result[str]):
        # this method can be used to update the argument value, for example
        # to assign it a value given by the user via command line
        self.value = value
        if value.is_resolved:
            v = value.value
            if v.lower() in BOOL_VALUES:
                self.inferred_type = LaunchArgumentValueType.BOOL
                return
            try:
                if v == str(int(v)):
                    self.inferred_type = LaunchArgumentValueType.INT
                    return
            except ValueError:
                pass
            try:
                if v == str(float(v)):
                    self.inferred_type = LaunchArgumentValueType.FLOAT
                    return
            except ValueError:
                pass
            # TODO match path regex
        self.inferred_type = LaunchArgumentValueType.STRING

    def build(self) -> ArgumentFeature:
        known_possible_values = []
        if self.value.is_resolved:
            known_possible_values.append(self.value.value)
        return ArgumentFeature(
            self.id,
            self.name,
            default_value=self.default_value,
            description=self.description,
            known_possible_values=known_possible_values,
            inferred_type=self.inferred_type,
        )


@frozen
class LaunchScope:
    file_path: Path
    args: Dict[str, ArgumentFeatureBuilder] = field(factory=dict)
    configs: Dict[str, Result] = field(factory=dict)
    anonymous: Dict[str, str] = field(factory=dict)

    def get(self, name: str) -> Result:
        value = self.configs.get(name)
        if value is None:
            arg = self.args.get(name)
            value = None if arg is None else arg.value
        if value is None:
            return Result.unknown_value()  # FIXME maybe raise error
        return value

    def set(self, name: str, value: Result):
        # if name not in self.configs:
        self.configs[name] = value

    def set_unknown(self, name: str):
        return self.set(name, Result.unknown_value())

    def set_text(self, name: str, text: str):
        return self.set(name, Resolved.from_string(text))

    def resolve(self, result: Optional[Result[LaunchSubstitution]]) -> Result[str]:
        if result is None:
            return Result.unknown_value()
        if not result.is_resolved:
            return Result.unknown_value(source=result.source)
        return self.substitute(result.value, source=result.source)

    def substitute(
        self,
        sub: LaunchSubstitution,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        if sub.is_text:
            return Resolved.from_string(sub.value, source=source)
        if sub.is_configuration:
            name = sub.name
            value = self.configs.get(name)
            if value is None:
                arg = self.args.get(name)
                value = None if arg is None else arg.value
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
                return Resolved.from_string(name, source=source)
            return value
        if sub.is_this_file:
            return Resolved.from_string(self.get_this_launch_file(), source=source)
        if sub.is_this_dir:
            return Resolved.from_string(self.get_this_launch_file_dir(), source=source)
        if sub.is_concatenation:
            parts = []
            for part in sub.parts:
                value = self.resolve(part)
                if not value.is_resolved:
                    return Result.unknown_value(source=source)
                parts.append(value)
            return Resolved.from_string(''.join(map(str, parts)), source=source)
        if sub.is_path_join:
            path = Path()
            for part in sub.parts:
                value = self.resolve(part)
                if not value.is_resolved:
                    return Result.unknown_value(source=source)
                path = path / str(value)
            return Resolved.from_string(path.as_posix(), source=source)
        return unknown_value(source=source)

    def duplicate(self) -> 'LaunchScope':
        # LaunchArgument is defined globally
        # LaunchConfiguration is scoped
        return LaunchScope(self.file_path, args=self.args, configs=dict(self.configs))

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
class LaunchFeatureModelBuilder:
    file: str
    system: AnalysisSystemInterface = field(factory=AnalysisSystemInterface)
    nodes: List[NodeFeature] = field(factory=list)
    scope_stack: List[LaunchScope] = field(factory=list)
    included_files: Set[FeatureId] = field(factory=set)

    @classmethod
    def from_file_path(
        cls,
        file_path: Path,
        system: AnalysisSystemInterface,
    ) -> 'LaunchFeatureModelBuilder':
        scopes = [LaunchScope(file_path)]
        return cls(str(file_path), system=system, scope_stack=scopes)

    @property
    def scope(self) -> LaunchScope:
        return self.scope_stack[-1]

    @scope.setter
    def scope(self, scope: LaunchScope):
        self.scope_stack[-1] = scope

    @property
    def root(self) -> LaunchScope:
        return self.scope_stack[0]

    def build(self, uid: Optional[FeatureId] = None) -> LaunchFileFeature:
        return LaunchFileFeature(
            uid if uid is not None else FeatureId(f'file:{self.file}'),
            self.file,
            arguments={ a.id: a.build() for a in self.root.args.values() },
            nodes={ n.id: n for n in self.nodes },
            inclusions=set(self.included_files),
        )
        # conflicts: Dict[FeatureId, LogicValue] = field(factory=dict)

    def enter_group(self):
        self.scope_stack.append(self.scope.duplicate())

    def exit_group(self):
        self.scope_stack.pop()

    def declare_argument(self, arg: LaunchArgument):
        name: str = arg.name
        feature = ArgumentFeatureBuilder(
            FeatureId(f'arg:{len(self.root.args)}'),
            name,
            default_value=self.scope.resolve(arg.default_value),
            description=self.scope.resolve(arg.description),
        )
        if arg.default_value is not None:
            feature.set(self.scope.resolve(arg.default_value))
        self.root.args[name] = feature
        # FIXME raise error if argument name already exists?

    def set_argument_value(self, name: str, value: Result[str]):
        self.root.args[name].set(value)

    def include_launch(self, include: LaunchInclusion):
        if include.namespace is None:
            namespace: Result = Resolved.from_string('/')
        else:
            namespace: Result = self.scope.resolve(include.namespace)
        arguments: Dict[str, Result[LaunchSubstitution]] = include.arguments
        file: Result = self.scope.resolve(include.file)
        if file.is_resolved:
            uid = FeatureId(f'file:{file.value}')
            self.included_files.add(uid)
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
            model: LaunchFileFeature = model_from_description(path, description, self.system)
            # must change the node ids, otherwise there will be a clash
            for node in model.nodes.values():
                uid: FeatureId = FeatureId(f'node:{len(self.nodes)}')
                self.nodes.append(evolve(node, id=uid))
        else:
            logger.warning(f'unknown launch file inclusion')
            return  # FIXME

    def launch_node(self, node: LaunchNode):
        logger.debug(f'launch_node({node!r})')
        package: Result = self.scope.resolve(node.package)
        executable: Result = self.scope.resolve(node.executable)
        # node_id = uid_node(str(package), str(executable))
        name: str = self._get_node_name(node.name, package, executable)
        # namespace: Optional[LaunchSubstitution]
        # remaps: Dict[LaunchSubstitution, LaunchSubstitution]
        output: Result = self.scope.resolve(node.output)
        args: Result = Resolved.from_list([
            self.scope.resolve(arg) for arg in node.arguments
        ])
        params: Result = self.parameters_from_list(node.parameters, node=name)
        if package.is_resolved and executable.is_resolved:
            fsnode = self.system.get_node_model(package, executable)  # FIXME
            if fsnode is None:
                logger.warning(f'unable to find node: {package}/{executable}')
            # TODO add ROS client library calls
        rosname = Resolved.from_string(name)  # TODO source if node.name is not None
        rosnode = RosNodeModel(
            rosname,
            package,
            executable,
            # Resolved.from_string(node_id),
            arguments=args,
            parameters=params,
            output=output,
        )
        uid: FeatureId = FeatureId(f'node:{len(self.nodes)}')
        feature = NodeFeature(uid, rosnode)
        self.nodes.append(feature)

    def parameters_from_list(self, parameters: LaunchNodeParameterList, node: Optional[str] = None) -> Result:
        if not parameters.is_resolved:
            return UnresolvedMapping.unknown_dict(source=parameters.source)
        result: Result = Resolved.from_dict({})
        param_dict: Dict[str, Result] = result.value
        for item in parameters.value:
            assert isinstance(item, Result), f'unexpected launch node parameter: {item!r}'
            try:
                new_params = self.process_parameter_item(item, node=node)
            except TypeError as e:
                logger.error(str(e))
                new_params = Result.unknown_value(source=item.source)
            if new_params.is_resolved:
                param_dict.update(new_params.value)
            else:
                result = new_params
                param_dict = result.known
        return result

    def process_parameter_item(self, item: Result, node: Optional[str] = None) -> Result:
        if item.is_resolved:
            if item.type.is_string or issubclass(item.type.token, Path):
                return self._parameters_from_yaml(item.value, node=node)
            elif issubclass(item.type.token, LaunchSubstitution):
                path: Result = self.scope.resolve(item)
                if path.is_resolved:
                    return self._parameters_from_yaml(path.value, node=node)
                else:
                    logger.warning('unable to resolve parameter file path')
                    return UnresolvedMapping.unknown_dict(source=path.source)
            elif item.type.is_mapping:
                result = {}
                param_dict: Dict[Result[Any], Result[Any]] = item.value
                for key, sub in param_dict.items():
                    if key.is_resolved and isinstance(key.value, str):
                        name: Result = Resolved.from_string(key.value, source=key.source)
                    else:
                        name: Result = self.scope.resolve(key)
                    if not name.is_resolved:
                        # break the whole dict analysis
                        logger.warning('unable to resolve parameter name')
                        return UnresolvedMapping.unknown_dict(source=key.source)
                    if not sub.is_resolved:
                        result[name.value] = unknown_value()
                    elif isinstance(sub.value, LaunchSubstitution):
                        result[name.value] = self.scope.resolve(sub)
                    elif sub.type.is_string:
                        result[name.value] = sub
                    elif sub.type.is_iterable:
                        result[name.value] = sub
                    else:
                        # TODO how to handle nested dicts and non-string values?
                        result[name.value] = Resolved.from_string(str(sub.value), source=sub.source)
                return Resolved.from_dict(result, source=item.source)
            else:
                raise TypeError(f'unexpected parameter: {item!r}')
        else:
            logger.warning('unable to resolve parameter list item')
            return UnresolvedMapping.unknown_dict(source=item.source)

    def _parameters_from_yaml(self, path: str, node: Optional[str] = None) -> Result:
        try:
            data = self.system.read_yaml_file(path)
        except OSError as e:
            logger.warning(f'unable to load parameter file: {e}')
            return UnresolvedMapping.unknown_dict(source=path.source)
        except ValueError:
            logger.warning(f'parameter file located in unsafe path: {path}')
            return UnresolvedMapping.unknown_dict(source=path.source)
        # return Resolved.from_dict({'TODO': Resolved.from_string(str(path))})
        params = data
        if node:
            params = {}
            parts = node.split('/')
            parts.reverse()
            current = data
            while parts:
                name = parts.pop()
                current = current.get(name, current.get(f'/{name}', {}))
                if current:
                    params.update(current.get('ros__parameters', {}))
            current = data.get(node, {})
            if current:
                params.update(current.get('ros__parameters', {}))
            if node.startswith('/'):
                current = data.get(node[1:], {})
                if current:
                    params.update(current.get('ros__parameters', {}))
            else:
                current = data.get(f'/{node}', {})
                if current:
                    params.update(current.get('ros__parameters', {}))
            current = data.get('/**', {})  # FIXME there might be other patterns
            if current:
                params.update(current.get('ros__parameters', {}))
        return Resolved.from_dict(params)

    def _get_node_name(
        self,
        name: Optional[Result[LaunchSubstitution]],
        package: Result,
        executable: Result,
    ) -> str:
        if name is None:
            if not package.is_resolved or not executable.is_resolved:
                return 'anonymous'  # FIXME
            node = self.system.get_node_model(package.value, executable.value)
            if node is None:
                return 'anonymous'  # FIXME
            name = node.rosname
            if name is None:
                return executable.value  # FIXME
            return str(name)
        value: Result = self.scope.resolve(name)
        return value.value if value.is_resolved else '$(?)'


def model_from_description(
    path: Path,
    description: LaunchDescription,
    system: AnalysisSystemInterface,
) -> LaunchFileFeature:
    logger.debug(f'model_from_description({path}, {description}, {system})')
    builder = LaunchFeatureModelBuilder.from_file_path(path, system)
    if not description.entities.is_resolved:
        return LaunchFileFeature(path)  # FIXME
    _add_list_of_entities(builder, description.entities.value)
    return builder.build()


def _add_list_of_entities(
    builder: LaunchFeatureModelBuilder,
    entities: Iterable[Result[LaunchEntity]],
):
    for result in entities:
        if not result.is_resolved:
            continue
        entity: LaunchEntity = result.value
        if entity.is_argument:
            builder.declare_argument(entity)
        elif entity.is_inclusion:
            builder.include_launch(entity)
        elif entity.is_node:
            builder.launch_node(entity)
        elif entity.is_group:
            assert isinstance(entity, LaunchGroupAction)
            if not entity.entities.is_resolved:
                continue
            builder.enter_group()
            _add_list_of_entities(builder, entity.entities.value)
            builder.exit_group()
