# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, Mapping, Optional, Set, Tuple

import logging
from pathlib import Path

from attrs import define, evolve, field, frozen

from haros.analysis.python.dataflow import unknown_bool, unknown_value
from haros.errors import AnalysisError, ParseError
from haros.internal.interface import AnalysisSystemInterface
from haros.metamodel.common import TYPE_TOKEN_STRING, Resolved, Result, UnresolvedMapping
from haros.metamodel.launch import (
    ArgumentFeature,
    FeatureId,
    IfCondition,
    LaunchArgument,
    LaunchArgumentValueType,
    LaunchCondition,
    LaunchDescription,
    LaunchEntity,
    LaunchFileFeature,
    LaunchGroupAction,
    LaunchInclusion,
    LaunchNode,
    LaunchNodeParameterList,
    LaunchNodeRemapList,
    LaunchScopeContext,
    LaunchSubstitution,
    NodeFeature,
    UnlessCondition,
    substitute,
    substitute_optional,
)
from haros.metamodel.logic import FALSE, TRUE, LogicValue, LogicVariable
from haros.metamodel.ros import RosNodeModel

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

BOOL_VALUES: Final[Tuple[str]] = ('true', 'false')

TRUE_VALUES: Final[Tuple[str]] = ('true', '1')

FALSE_VALUES: Final[Tuple[str]] = ('false', '0')

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
class LaunchScope(LaunchScopeContext):
    file_path: Path
    condition: LogicValue = field(default=TRUE)
    args: Dict[str, ArgumentFeatureBuilder] = field(factory=dict)
    configs: Dict[str, Result] = field(factory=dict)
    anonymous: Dict[str, str] = field(factory=dict, eq=False)

    def get(self, name: str) -> Optional[Result]:
        value = self.configs.get(name)
        if value is None:
            arg = self.args.get(name)
            value = None if arg is None else arg.value
        return value

    def get_arg(self, name: str) -> Optional[Result]:
        arg = self.args.get(name)
        return None if arg is None else arg.value

    def set(self, name: str, value: Result):
        # if name not in self.configs:
        self.configs[name] = value

    def resolve_condition(self, condition: Optional[Result[LaunchCondition]]) -> Result[bool]:
        if condition is None:
            return Resolved.from_bool(True)
        if condition.is_resolved:
            if condition.value.is_if_condition:
                assert isinstance(condition.value, IfCondition), repr(condition.value)
                result = substitute(condition.value.expression, self, source=condition.source)
                if result.is_resolved:
                    value = result.value.lower()
                    if value in TRUE_VALUES:
                        return Resolved.from_bool(True, source=condition.source)
                    if value in FALSE_VALUES:
                        return Resolved.from_bool(False, source=condition.source)
            elif condition.value.is_unless_condition:
                assert isinstance(condition.value, UnlessCondition), repr(condition.value)
                result = substitute(condition.value.expression, self, source=condition.source)
                if result.is_resolved:
                    value = result.value.lower()
                    if value in TRUE_VALUES:
                        return Resolved.from_bool(False, source=condition.source)
                    if value in FALSE_VALUES:
                        return Resolved.from_bool(True, source=condition.source)
            # TODO LaunchConfigurationEquals
            # TODO LaunchConfigurationNotEquals
            # FIXME https://github.com/ros2/launch/blob/rolling/launch/launch/conditions/launch_configuration_equals.py
        return unknown_bool(source=condition.source)

    def duplicate(self, join_condition: LogicValue = TRUE) -> 'LaunchScope':
        # LaunchArgument is defined globally
        # LaunchConfiguration is scoped
        join_condition = self.condition.join(join_condition)
        return LaunchScope(
            self.file_path,
            condition=join_condition,
            args=self.args,
            configs=dict(self.configs),
        )

    def compute_anon_name(self, name: str) -> str:
        value = self.anonymous.get(name)
        if not value:
            value = super().compute_anon_name(name)
            self.anonymous[name] = value
        return value

    def get_this_launch_file(self) -> str:
        return self.file_path.as_posix()

    def get_this_launch_file_dir(self) -> str:
        return self.file_path.parent.as_posix()


def _empty_args() -> Result[Mapping[str, Result[str]]]:
    return Resolved.from_dict({})


@define
class LaunchFeatureModelBuilder:
    file: str
    system: AnalysisSystemInterface = field(factory=AnalysisSystemInterface)
    nodes: List[NodeFeature] = field(factory=list)
    scope_stack: List[LaunchScope] = field(factory=list)
    included_files: Set[FeatureId] = field(factory=set)
    passed_args: Result[Mapping[str, Result[str]]] = field(factory=_empty_args)

    @classmethod
    def from_file_path(
        cls,
        file_path: Path,
        system: AnalysisSystemInterface,
        args: Optional[Result[Mapping[str, Result[str]]]] = None,
    ) -> 'LaunchFeatureModelBuilder':
        passed_args = args if args is not None else Resolved.from_dict({})
        scopes = [LaunchScope(file_path)]
        return cls(str(file_path), system=system, scope_stack=scopes, passed_args=passed_args)

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

    def enter_group(self, condition: Optional[Result[LaunchCondition]]):
        boolean: Result[bool] = self.scope.resolve_condition(condition)
        phi: LogicValue = _logic_value_from_result(boolean)
        self.scope_stack.append(self.scope.duplicate(join_condition=phi))

    def exit_group(self):
        self.scope_stack.pop()

    def declare_argument(self, arg: LaunchArgument):
        name: str = arg.name
        default_value = substitute_optional(arg.default_value, self.scope)
        feature = ArgumentFeatureBuilder(
            FeatureId(f'arg:{len(self.root.args)}'),
            name,
            default_value=default_value,
            description=substitute(arg.description, self.scope),
        )
        if self.passed_args.is_resolved:
            # discard None
            default_value = default_value or Resolved.from_string('')
            value = self.passed_args.value.get(name, default_value)
        else:
            value = Result.unknown_value(type=TYPE_TOKEN_STRING, source=self.passed_args.source)
        feature.set(value)
        self.root.args[name] = feature
        # FIXME raise error if argument name already exists?

    def set_argument_value(self, name: str, value: Result[str]):
        self.root.args[name].set(value)

    def include_launch(self, include: LaunchInclusion):
        if include.namespace is None:
            namespace: Result[str] = Resolved.from_string('/')
        else:
            namespace: Result[str] = substitute(include.namespace, self.scope)
        file: Result[str] = substitute(include.file, self.scope)
        if not file.is_resolved:
            logger.warning(f'unknown launch file inclusion')
            return  # FIXME
        uid = FeatureId(f'file:{file.value}')
        self.included_files.add(uid)
        try:
            description = self.system.get_launch_description(file.value)
            logger.info(f'parsed included launch file: {file.value}')
        except FileNotFoundError as e:
            logger.warning(f'file not found: {e}')
            return
        except (AnalysisError, ParseError) as e:
            logger.warning(f'analysis error: {e}')
            return
        path: Path = Path(file.value)
        # pass arguments down (order might be unreliable)
        arguments = self._get_include_arguments(include)
        model = model_from_description(path, description, self.system, args=arguments)
        # must change the node ids, otherwise there will be a clash
        for node in model.nodes.values():
            uid: FeatureId = FeatureId(f'node:{len(self.nodes)}')
            self.nodes.append(evolve(node, id=uid))

    def _get_include_arguments(self, include: LaunchInclusion) -> Result[Dict[str, Result[str]]]:
        arguments = {}
        for passed_arg in include.arguments:
            if not passed_arg.is_resolved:
                # reset everything we know
                return UnresolvedMapping.unknown_dict(source=passed_arg.source)
            # the tuple itself is resolved
            key = substitute(passed_arg.value[0], self.scope)
            if not key.is_resolved:
                # reset everything we know
                return UnresolvedMapping.unknown_dict(source=key.source)
            value = substitute(passed_arg.value[1], self.scope)
            arguments[key.value] = value
        return Resolved.from_dict(arguments)

    def launch_node(self, node: LaunchNode):
        logger.debug(f'launch_node({node!r})')
        package: Result = substitute(node.package, self.scope)
        executable: Result = substitute(node.executable, self.scope)
        # node_id = uid_node(str(package), str(executable))
        name: str = self._get_node_name(node.name, package, executable)
        # namespace: Optional[LaunchSubstitution]
        # remaps: Dict[LaunchSubstitution, LaunchSubstitution]
        output: Result = substitute(node.output, self.scope)
        args: Result = Resolved.from_list([
            substitute(arg, self.scope) for arg in node.arguments
        ])
        params: Result = self.parameters_from_list(node.parameters, node=name)
        remaps: Result = self.remappings_from_list(node.remaps)
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
            remappings=remaps,
            output=output,
        )
        uid: FeatureId = FeatureId(f'node:{len(self.nodes)}')
        boolean: Result[bool] = self.scope.resolve_condition(node.condition)
        condition: LogicValue = _logic_value_from_result(boolean)
        condition = self.scope.condition.join(condition)
        feature = NodeFeature(uid, rosnode, condition=condition)
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
                # FIXME what to do with lost information?
                result = new_params
                param_dict = {}
        return result

    def process_parameter_item(self, item: Result, node: Optional[str] = None) -> Result:
        if item.is_resolved:
            if item.type.is_string or issubclass(item.type.token, Path):
                return self._parameters_from_yaml(item.value, node=node)
            elif issubclass(item.type.token, LaunchSubstitution):
                path: Result = substitute(item, self.scope)
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
                        name: Result = substitute(key, self.scope)
                    if not name.is_resolved:
                        # break the whole dict analysis
                        logger.warning('unable to resolve parameter name')
                        return UnresolvedMapping.unknown_dict(source=key.source)
                    if not sub.is_resolved:
                        result[name.value] = unknown_value()
                    elif isinstance(sub.value, LaunchSubstitution):
                        result[name.value] = substitute(sub, self.scope)
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
        params = {key: Resolved.from_value(value) for key, value in params.items()}
        return Resolved.from_dict(params)

    def remappings_from_list(self, remaps: LaunchNodeRemapList) -> Result[Dict[str, Result[str]]]:
        if not remaps.is_resolved:
            return UnresolvedMapping.unknown_dict(source=remaps.source)
        has_unknown: bool = False
        remap_dict: Dict[str, Result[str]] = {}
        for rule in remaps.value:
            assert isinstance(rule, Result), f'unexpected launch remap rule: {rule!r}'
            if not rule.is_resolved:
                has_unknown = True
                continue
            assert isinstance(rule.value, tuple)
            if len(rule.value) != 2:
                logger.error(f'invalid remap rule: {rule.value!r}')
                continue
            from_name, to_name = rule.value
            if issubclass(from_name.type.token, LaunchSubstitution) and from_name.is_resolved:
                from_name = substitute(from_name, self.scope)
            if issubclass(to_name.type.token, LaunchSubstitution) and to_name.is_resolved:
                to_name = substitute(to_name, self.scope)
            if not from_name.type.is_string:
                logger.warning(f'unable to resolve ROS name in remap rule: {from_name!r}')
                continue
            if not to_name.type.is_string:
                logger.warning(f'unable to resolve ROS name in remap rule: {to_name!r}')
                continue
            if from_name.is_resolved:
                remap_dict[from_name.value] = to_name
            else:
                has_unknown = True
        if has_unknown:
            return UnresolvedMapping.unknown_dict(known=remap_dict, source=remaps.source)
        return Resolved.from_dict(remap_dict, source=remaps.source)

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
        value: Result = substitute(name, self.scope)
        return value.value if value.is_resolved else '$(?)'


def model_from_description(
    path: Path,
    description: LaunchDescription,
    system: AnalysisSystemInterface,
    args: Optional[Result[Mapping[str, Result[str]]]] = None,
) -> LaunchFileFeature:
    logger.debug(f'model_from_description({path}, {description}, {system}, args={args})')
    builder = LaunchFeatureModelBuilder.from_file_path(path, system, args=args)
    if not description.entities.is_resolved:
        return LaunchFileFeature(FeatureId(f'file:{path}'), path)
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
            builder.enter_group(entity.condition)
            _add_list_of_entities(builder, entity.entities.value)
            builder.exit_group()


def _logic_value_from_result(condition: Result[bool]) -> LogicValue:
    if condition.is_resolved:
        return TRUE if condition.value else FALSE
    return LogicVariable(data=condition)
