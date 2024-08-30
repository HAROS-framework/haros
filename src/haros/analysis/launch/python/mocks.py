# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Final, Generic, Iterable, List, Optional, Mapping, Union

import logging
import os
from pathlib import Path
from types import SimpleNamespace

from attrs import define, frozen
from haros.analysis.python.dataflow import (
    TYPE_TOKEN_OBJECT,
    TYPE_TOKEN_STRING,
    MockObject,
    const_list,
    const_none,
    library_function_wrapper,
    solved,
    solved_from,
    unknown_tuple,
    unknown_value,
)
from haros.internal.interface import AnalysisSystemInterface, PathType
from haros.metamodel.common import T, Resolved, Result, UnresolvedString, VariantData
from haros.metamodel.launch import (
    ConcatenationSubstitution,
    EqualsSubstitution,
    IfCondition,
    LaunchCondition,
    LaunchEntity,
    LaunchGroupAction,
    LaunchNodeRemapList,
    NotEqualsSubstitution,
    PythonExpressionSubstitution,
    TextSubstitution,
    UnlessCondition,
    const_substitution,
    const_text,
    LaunchArgument,
    LaunchConfiguration,
    LaunchDescription,
    LaunchInclusion,
    LaunchNode,
    LaunchSubstitution,
    ThisDirectorySubstitution,
    unknown_parameter_list,
    unknown_remap_list,
    unknown_substitution,
)

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

LAUNCH_ENTRY_POINT: Final[str] = 'generate_launch_description'

UNKNOWN_TOKEN: Final[str] = '{?}'

###############################################################################
# Mocks
###############################################################################


@define
class HarosMockObject(MockObject, Generic[T]):
    def _haros_freeze(self) -> T:
        # use a method name with a low probability of name collision
        # with one of the mocked object's methods or attributes
        raise NotImplementedError()


@define
class LaunchDescriptionMock(HarosMockObject[LaunchDescription]):
    entities: Result[List[Result[LaunchEntity]]]

    def add_action(self, action: Result[LaunchEntity]) -> Result[None]:
        if self.entities.is_resolved:
            self.entities.value.append(action)
        return solved_from(None)

    def _haros_freeze(self) -> LaunchDescription:
        if self.entities.is_resolved:
            return LaunchDescription(solved_from(tuple(self.entities.value)))
        return LaunchDescription(unknown_tuple())


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
    for arg in arg_list.value:
        if not arg.is_resolved:
            parts.append(unknown_substitution(source=arg.source))
            continue
        value = arg.value
        assert not isinstance(value, VariantData), repr(value)
        if isinstance(value, Result):
            parts.append(value)
        elif isinstance(value, LaunchSubstitution):
            parts.append(const_substitution(value, source=arg.source))
        else:
            parts.append(const_text(str(value), source=arg.source))  # FIXME
    return ConcatenationSubstitution(tuple(parts))


def launch_description_function(
    arg_list: Optional[Result[List[Result[LaunchEntity]]]] = None,
) -> LaunchDescriptionMock:
    if not arg_list:
        return LaunchDescriptionMock(Resolved.from_list([]))
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


def launch_configuration_function(
    name: Result,
    default: Optional[Result] = None,
) -> LaunchConfiguration:
    return LaunchConfiguration(
        _dataflow_to_string(name),
        default_value=_dataflow_to_launch_substitution(default),
    )


def include_launch_description_function(
    source: Result,
    launch_arguments: Optional[Result] = None,
    condition: Optional[Result[LaunchCondition]] = None,
) -> LaunchInclusion:
    _source: Result[LaunchSubstitution] = unknown_substitution(source=source.source)
    if source.is_resolved:
        if isinstance(source.value, LaunchSubstitution):
            _source = const_substitution(source.value, source=source.source)
    _arguments = {}
    if launch_arguments is not None:
        if launch_arguments.is_resolved:
            assert launch_arguments.type.mask.can_be_iterable
            for item in launch_arguments.value:
                if isinstance(item, tuple):
                    key: Result[LaunchSubstitution] = _dataflow_to_launch_substitution(item[0])
                    value: Result[LaunchSubstitution] = _dataflow_to_launch_substitution(item[1])
                elif item.is_resolved:
                    assert item.type.mask.can_be_iterable
                    key: Result[LaunchSubstitution] = _dataflow_to_launch_substitution(item.value[0])
                    value: Result[LaunchSubstitution] = _dataflow_to_launch_substitution(item.value[1])
                else:
                    continue  # FIXME
                if not key.is_resolved:
                    unknown = _arguments.get(key, [])
                    unknown.append(value)
                    _arguments[key] = unknown
                else:
                    _arguments[key] = value
    return LaunchInclusion(file=_source, arguments=_arguments, condition=condition)


def node_function(
    executable: Result,
    package: Optional[Result] = None,
    name: Optional[Result] = None,
    parameters: Optional[Result] = None,
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
        params = Resolved.from_list([])
    elif parameters.is_resolved:
        params = Resolved.from_list([], source=parameters.source)
        for item in parameters.value:
            if item.is_resolved:
                if item.type.mask.can_be_mapping:
                    # dictionary that specifies parameter rules
                    # Keys of the dictionary can be strings or an iterable of
                    #   Substitutions that will be expanded to a string.
                    # Values in the dictionary can be strings, integers, floats, or tuples
                    #   of Substitutions that will be expanded to a string.
                    # Additionally, values in the dictionary can be lists of the
                    #   aforementioned types, or another dictionary with the same properties.
                    params.value.append(Resolved.from_dict(item.value, source=item.source))  # FIXME
                else:
                    # yaml file that contains parameter rules
                    # (string or pathlib.Path to the full path of the file)
                    if item.type.mask.can_be_string:
                        params.value.append(Resolved.from_string(item.value, source=item.source))
                    elif isinstance(item.value, Path):
                        params.value.append(Resolved.from_value(item.value, source=item.source))
                    else:
                        params.value.append(Resolved.from_value(item.value, source=item.source))
            else:
                params = unknown_parameter_list(source=parameters.source)
                break
    else:
        params = unknown_parameter_list(source=parameters.source)
    if remappings is None:
        remaps = Resolved.from_list([])
    elif remappings.is_resolved:
        remaps: LaunchNodeRemapList = Resolved.from_list([])
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
    logger.warning(f'LaunchNode {pkg}/{exe} condition: {condition}')
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
    return LaunchGroupAction(actions or const_list([]), condition=condition)


def get_package_share_directory_function(package: Result) -> Result[LaunchSubstitution]:
    if not package.is_resolved:
        return UnresolvedString()
        # return unknown_substitution(source=package.source)
    path = str(package.value).replace('\\', '/')
    return f'/usr/share/ros/{path}'
    # return const_text(package.value, source=package.source)


LAUNCH_SYMBOLS = {
    'launch.LaunchDescription': launch_description_function,
    'launch.actions.DeclareLaunchArgument': declare_launch_argument_function,
    'launch.actions.GroupAction': group_action_function,
    'launch.actions.IncludeLaunchDescription': include_launch_description_function,
    'launch.conditions.IfCondition': if_condition_function,
    'launch.conditions.if_condition.IfCondition': if_condition_function,
    'launch.conditions.UnlessCondition': unless_condition_function,
    'launch.conditions.unless_condition.UnlessCondition': unless_condition_function,
    'launch.substitutions.LaunchConfiguration': launch_configuration_function,
    'launch_ros.actions.Node': node_function,
    'ament_index_python.packages.get_package_share_directory': get_package_share_directory_function,
    'launch.launch_description_sources.PythonLaunchDescriptionSource': python_launch_description_source_function,
    'launch.substitutions.ThisLaunchFileDir': ThisDirectorySubstitution,
    'launch.substitutions.EqualsSubstitution': EqualsSubstitution,
    'launch.substitutions.NotEqualsSubstitution': NotEqualsSubstitution,
    'launch.substitutions.PythonExpression': PythonExpressionSubstitution,
}


def prepare_builtin_symbols() -> Mapping[str, Any]:
    symbols = {}
    ns = SimpleNamespace()
    ns.path = SimpleNamespace()
    for key in dir(os.path):
        if key.startswith('_'):
            continue
        value = getattr(os.path, key)
        if key == 'join':
            value = _os_path_wrapper
        if callable(value):
            value = library_function_wrapper(key, 'os.path', value)
        setattr(ns.path, key, value)
    ns.environ = {
        'TURTLEBOT3_MODEL': 'burger',
        'LDS_MODEL': 'LDS-01',
    }
    symbols['os'] = ns
    return symbols


def _os_path_wrapper(*args) -> str:
    return Path(*args).as_posix()


@frozen
class LazyFileHandle(MockObject):
    path: Result[PathType]
    system: AnalysisSystemInterface

    def read(self, encoding: Optional[Result[str]] = None) -> Result[str]:
        try:
            if self.path.is_resolved:
                encoding = encoding or const_none()
                text = self.system.read_text_file(self.path.value, encoding=encoding.value)
                return solved_from(text)
        except ValueError:
            pass
        return unknown_value(type=TYPE_TOKEN_STRING)

    def __str__(self) -> str:
        return f'{self.__class__.__name__}(path={self.path})'


def builtin_open(
    system: AnalysisSystemInterface
) -> Callable[[Result[str], Optional[Result[str]]], Result[LazyFileHandle]]:
    def wrapper(path: Result[str], mode: Optional[Result[str]] = None) -> Result[LazyFileHandle]:
        if not path.is_resolved:
            return unknown_value()
        if mode is None:
            mode = solved_from('r')
        if not mode.is_resolved or mode.value != 'r':
            return unknown_value()
        return solved(TYPE_TOKEN_OBJECT, LazyFileHandle(path, system))
    return wrapper


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

