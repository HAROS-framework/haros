# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, Iterable, List, Optional, Tuple, Union

import logging
from pathlib import Path

from attrs import define

from haros.analysis.python.mocks import HarosMockObject
from haros.metamodel.common import VariantData
from haros.metamodel.launch import (
    ConcatenationSubstitution,
    EqualsSubstitution,
    IfCondition,
    LaunchArgument,
    LaunchArgumentKeyValuePair,
    LaunchCondition,
    LaunchConfiguration,
    LaunchDescription,
    LaunchEntity,
    LaunchGroupAction,
    LaunchInclusion,
    LaunchNode,
    LaunchNodeRemapList,
    LaunchSetEnvironment,
    LaunchSubstitution,
    NotEqualsSubstitution,
    ParameterFileDescription,
    PythonExpressionSubstitution,
    ReplaceStringSubstitution,
    RewrittenYamlSubstitution,
    TextSubstitution,
    ThisDirectorySubstitution,
    UnlessCondition,
    const_substitution,
    const_text,
    unknown_parameter_list,
    unknown_remap_list,
    unknown_substitution,
)
from haros.metamodel.result import Result, TypeToken

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

LAUNCH_ENTRY_POINT: Final[str] = 'generate_launch_description'

UNKNOWN_TOKEN: Final[str] = '{?}'

TYPE_LAUNCH_SUBSTITUTION: Final[TypeToken[LaunchSubstitution]] = TypeToken.of(LaunchSubstitution())

###############################################################################
# Mocks
###############################################################################


@define
class LaunchDescriptionMock(HarosMockObject[LaunchDescription]):
    entities: Result[List[Result[LaunchEntity]]]

    def add_action(self, action: Result[LaunchEntity]) -> None:
        if self.entities.is_resolved:
            self.entities.value.append(action)

    def _haros_freeze(self) -> LaunchDescription:
        if self.entities.is_resolved:
            return LaunchDescription(Result.of_tuple(tuple(self.entities.value)))
        return LaunchDescription(Result.of_tuple())


###############################################################################
# Interface
###############################################################################


def if_condition_function(expr: Result[Union[str, LaunchSubstitution]]) -> IfCondition:
    if expr.is_resolved and expr.type.is_string:
        expr = const_text(expr.value, source=expr.source)
    return IfCondition(expr)


def unless_condition_function(expr: Result[Union[str, LaunchSubstitution]]) -> UnlessCondition:
    if expr.is_resolved and expr.type.is_string:
        expr = const_text(expr.value, source=expr.source)
    return UnlessCondition(expr)


def python_launch_description_source_function(arg_list: Result) -> LaunchSubstitution:
    if not arg_list.is_resolved:
        return unknown_substitution(source=arg_list.source)
    if arg_list.type.is_string:
        assert isinstance(arg_list.value, str)
        return TextSubstitution(arg_list.value)
    assert arg_list.type.is_iterable
    parts = []
    for arg in arg_list.items():
        if not arg.is_resolved:
            parts.append(unknown_substitution(source=arg.source))
            continue
        value = arg.value
        assert not isinstance(value, VariantData), repr(value)
        assert not isinstance(value, Result), repr(value)
        if isinstance(value, LaunchSubstitution):
            parts.append(const_substitution(value, source=arg.source))
        else:
            parts.append(const_text(str(value), source=arg.source))  # FIXME
    return ConcatenationSubstitution(tuple(parts))


def launch_description_function(
    arg_list: Optional[Result[List[Result[LaunchEntity]]]] = None,
) -> LaunchDescriptionMock:
    if not arg_list:
        return LaunchDescriptionMock(Result.of_list([]))
    return LaunchDescriptionMock(arg_list)


def declare_launch_argument_function(
    name: Result,
    default_value: Optional[Result] = None,
    description: Optional[Result] = None,
) -> LaunchArgument:
    return LaunchArgument(
        _dataflow_to_string(name),
        default_value=_dataflow_to_launch_substitution(default_value),
        description=_dataflow_to_launch_substitution(description),
    )


def set_environment_variable_function(name: Result, value: Result) -> LaunchSetEnvironment:
    return LaunchSetEnvironment(
        _dataflow_to_launch_substitution(name),
        _dataflow_to_launch_substitution(value),
    )


def launch_configuration_function(
    name: Result,
    default: Optional[Result] = None,
) -> LaunchConfiguration:
    return LaunchConfiguration(
        _dataflow_to_string(name),
        default_value=_dataflow_to_launch_substitution(default),
    )


StringOrSubstitution = Result[Union[str, LaunchSubstitution]]
SubstitutionPair = Tuple[StringOrSubstitution, StringOrSubstitution]
LaunchArgumentKeyValue = Union[SubstitutionPair, Result[SubstitutionPair]]


def include_launch_description_function(
    source: Result,
    launch_arguments: Optional[Result[Iterable[LaunchArgumentKeyValue]]] = None,
    condition: Optional[Result[LaunchCondition]] = None,
) -> LaunchInclusion:
    _source: Result[LaunchSubstitution] = unknown_substitution(source=source.source)
    if source.is_resolved:
        if isinstance(source.value, LaunchSubstitution):
            _source = const_substitution(source.value, source=source.source)
    _arguments: List[LaunchArgumentKeyValuePair] = []
    if launch_arguments is not None and launch_arguments.is_resolved:
        assert launch_arguments.type.is_iterable, repr(launch_arguments)
        for item in launch_arguments.value:
            # handle non-deeply transformed values from dataflow
            if isinstance(item, tuple):
                item = Result.of_tuple(item)
            assert item.type.is_iterable, repr(item)
            if item.is_resolved:
                key = _dataflow_to_launch_substitution(item.value[0])
                value = _dataflow_to_launch_substitution(item.value[1])
                _arguments.append(Result.of_tuple((key, value), source=item.source))
    return LaunchInclusion(file=_source, arguments=_arguments, condition=condition)


def node_function(
    executable: Result,
    package: Optional[Result] = None,
    name: Optional[Result] = None,
    parameters: Optional[Result[Iterable[Result[Any]]]] = None,
    arguments: Optional[Result] = None,
    output: Optional[Result] = None,
    remappings: Optional[Result[Iterable[Result[Any]]]] = None,
    condition: Optional[LaunchCondition] = None,
    respawn: Optional[Any] = None,  # FIXME
    respawn_delay: Optional[Any] = None,  # FIXME
) -> LaunchNode:
    # docs: https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/node.py
    # Node.__init__:
    #   executable: SomeSubstitutionsType
    #   package: Optional[SomeSubstitutionsType]
    #   name: Optional[SomeSubstitutionsType]
    #   namespace: Optional[SomeSubstitutionsType]
    #   exec_name: Optional[SomeSubstitutionsType]
    #   parameters: Optional[SomeParameters]
    #   remappings: Optional[SomeRemapRules]
    #   ros_arguments: Optional[Iterable[SomeSubstitutionsType]]
    #   arguments: Optional[Iterable[SomeSubstitutionsType]]
    #   **kwargs
    if parameters is None:
        params = Result.of_list([])
    elif parameters.is_resolved:
        params = Result.of_list([], source=parameters.source)
        for item in parameters.value:
            if not item.is_resolved:
                params = unknown_parameter_list(source=parameters.source)
                break
            if item.type.is_mapping:
                # dictionary that specifies parameter rules
                # Keys of the dictionary can be strings or an iterable of
                #   Substitutions that will be expanded to a string.
                # Values in the dictionary can be strings, integers, floats, or tuples
                #   of Substitutions that will be expanded to a string.
                # Additionally, values in the dictionary can be lists of the
                #   aforementioned types, or another dictionary with the same properties.
                params.value.append(Result.of_dict(item.value, source=item.source))  # FIXME
            else:
                # yaml file that contains parameter rules
                # (string or pathlib.Path to the full path of the file)
                if item.type.is_string:
                    params.value.append(Result.of_string(item.value, source=item.source))
                elif isinstance(item.value, Path):
                    params.value.append(Result.of(item.value, source=item.source))
                else:
                    params.value.append(Result.of(item.value, source=item.source))
    else:
        params = unknown_parameter_list(source=parameters.source)
    if remappings is None:
        remaps = Result.of_list([])
    elif remappings.is_resolved:
        remaps: LaunchNodeRemapList = Result.of_list([])
        for item in remappings.value:
            if item.is_resolved:
                remaps.value.append(item)
            else:
                remaps = unknown_remap_list(source=remappings.source)
                break
    else:
        remaps = unknown_remap_list(source=remappings.source)
    pkg = _dataflow_to_launch_substitution(package)
    exe = _dataflow_to_launch_substitution(executable)
    return LaunchNode(
        pkg,
        exe,
        name=_dataflow_to_launch_substitution(name),
        parameters=params,
        remaps=remaps,
        output=_dataflow_to_launch_substitution(output, default='log'),
        arguments=_dataflow_to_launch_list(arguments),
        condition=condition,
    )


def group_action_function(
    actions: Optional[Result[Iterable[Result[LaunchEntity]]]] = None,
    condition: Optional[LaunchCondition] = None,
) -> LaunchGroupAction:
    return LaunchGroupAction(actions or Result.of_list([]), condition=condition)


# FIXME return type
def get_package_share_directory_function(package: Result) -> Result[LaunchSubstitution]:
    if not package.is_resolved:
        return Result.unknown_value(of_type=TYPE_LAUNCH_SUBSTITUTION)
        # return unknown_substitution(source=package.source)
    path = str(package.value).replace('\\', '/')
    return f'/usr/share/ros/{path}'
    # return const_text(package.value, source=package.source)


LAUNCH_SYMBOLS = {
    'ament_index_python.packages.get_package_share_directory': get_package_share_directory_function,
    'launch.LaunchDescription': launch_description_function,
    'launch.actions.DeclareLaunchArgument': declare_launch_argument_function,
    'launch.actions.GroupAction': group_action_function,
    'launch.actions.IncludeLaunchDescription': include_launch_description_function,
    'launch.actions.SetEnvironmentVariable': set_environment_variable_function,
    'launch.conditions.IfCondition': if_condition_function,
    'launch.conditions.if_condition.IfCondition': if_condition_function,
    'launch.conditions.UnlessCondition': unless_condition_function,
    'launch.conditions.unless_condition.UnlessCondition': unless_condition_function,
    'launch.launch_description_sources.PythonLaunchDescriptionSource': python_launch_description_source_function,
    'launch.substitutions.EqualsSubstitution': EqualsSubstitution,
    'launch.substitutions.LaunchConfiguration': launch_configuration_function,
    'launch.substitutions.NotEqualsSubstitution': NotEqualsSubstitution,
    'launch.substitutions.PythonExpression': PythonExpressionSubstitution,
    'launch.substitutions.ThisLaunchFileDir': ThisDirectorySubstitution,
    'launch_ros.actions.Node': node_function,
    'launch_ros.descriptions.ParameterFile': ParameterFileDescription.factory,
    'nav2_common.launch.ReplaceString': ReplaceStringSubstitution,
    'nav2_common.launch.RewrittenYaml': RewrittenYamlSubstitution,
}


###############################################################################
# Helper Functions
###############################################################################


def _dataflow_to_string(value: Result) -> str:
    if value.is_resolved:
        return str(value.value)
    return UNKNOWN_TOKEN


def _dataflow_to_launch_substitution(
    result: Optional[Result],
    default: Optional[str] = None,
) -> Optional[Result[LaunchSubstitution]]:
    if result is None:
        return None if default is None else const_text(default)
    if result.is_resolved:
        if isinstance(result.value, LaunchSubstitution):
            return const_substitution(result.value, source=result.source)
        # FIXME this does not preserve primitive types like boolean
        return const_text(str(result.value), source=result.source)
    return unknown_substitution(source=result.source)


def _dataflow_to_launch_list(arg_list: Optional[Result]) -> List[Result[LaunchSubstitution]]:
    values = []
    if arg_list is not None:
        if arg_list.is_resolved:
            for arg in arg_list.value:
                values.append(_dataflow_to_launch_substitution(arg))
        else:
            values.append(unknown_substitution())  # FIXME
    return values
