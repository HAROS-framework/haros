# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, List, Optional, Mapping

import logging
import os
from pathlib import Path
from types import SimpleNamespace

from attrs import frozen

#from haros.analysis.python import query
from haros.analysis.python.dataflow import (
    library_function_wrapper,
    PythonResult,
    UnresolvedPythonString,
)
from haros.analysis.python.graph import from_ast
from haros.errors import WrongFileTypeError
from haros.metamodel.common import VariantData
from haros.metamodel.launch import (
    ConcatenationSubstitution,
    const_substitution,
    const_text,
    LaunchArgument,
    LaunchConfiguration,
    LaunchDescription,
    LaunchEntity,
    LaunchInclusion,
    LaunchNode,
    LaunchSubstitution,
    LaunchSubstitutionResult,
    ListSubstitution,
    PathJoinSubstitution,
    TextSubstitution,
    ThisDirectorySubstitution,
    UNKNOWN_SUBSTITUTION,
    unknown_substitution,
)
from haros.parsing.python import parse

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

LAUNCH_ENTRY_POINT: Final[str] = 'generate_launch_description'

UNKNOWN_TOKEN: Final[str] = '{?}'

###############################################################################
# Interface
###############################################################################


def python_launch_description_source_function(arg_list: PythonResult) -> LaunchSubstitution:
    if not arg_list.is_resolved:
        return UNKNOWN_SUBSTITUTION
    parts = []
    for arg in arg_list.value:
        if not arg.is_resolved:
            parts.append(UNKNOWN_SUBSTITUTION)
            continue
        value = arg.value
        assert not isinstance(value, VariantData), repr(value)
        if isinstance(value, LaunchSubstitution):
            parts.append(value)
        else:
            parts.append(TextSubstitution(str(value)))  # FIXME
    return ConcatenationSubstitution(tuple(parts))


def launch_description_function(arg_list: PythonResult) -> LaunchDescription:
    values = []
    if arg_list.is_resolved:
        for arg in arg_list.value:
            values.append(arg.value if arg.is_resolved else LaunchEntity())
    return LaunchDescription(values)


def declare_launch_argument_function(
    name: PythonResult,
    default_value: Optional[PythonResult] = None,
    description: Optional[PythonResult] = None,
) -> LaunchArgument:
    return LaunchArgument(
        _dataflow_to_string(name),
        default_value=_dataflow_to_launch_substitution(default_value),
        description=_dataflow_to_launch_substitution(description),
    )


def launch_configuration_function(
    name: PythonResult,
    default: Optional[PythonResult] = None,
) -> LaunchConfiguration:
    return LaunchConfiguration(
        _dataflow_to_string(name),
        default_value=_dataflow_to_launch_substitution(default),
    )


def include_launch_description_function(
    source: PythonResult,
    launch_arguments: Optional[PythonResult] = None,
) -> LaunchInclusion:
    _source: LaunchSubstitutionResult = unknown_substitution(source=source.source)
    if source.is_resolved:
        if isinstance(source.value, LaunchSubstitution):
            _source = const_substitution(source.value, source=source.source)
    _arguments = {}
    if launch_arguments is not None:
        if launch_arguments.is_resolved:
            assert launch_arguments.type.can_be_iterable
            for item in launch_arguments.value:
                if isinstance(item, tuple):
                    key: LaunchSubstitutionResult = _dataflow_to_launch_substitution(item[0])
                    value: LaunchSubstitutionResult = _dataflow_to_launch_substitution(item[1])
                elif item.is_resolved:
                    assert item.type.can_be_iterable
                    key: LaunchSubstitutionResult = _dataflow_to_launch_substitution(item.value[0])
                    value: LaunchSubstitutionResult = _dataflow_to_launch_substitution(item.value[1])
                else:
                    continue  # FIXME
                if not key.is_resolved:
                    unknown = _arguments.get(key, [])
                    unknown.append(value)
                    _arguments[key] = unknown
                else:
                    _arguments[key] = value
    return LaunchInclusion(file=_source, arguments=_arguments)


def node_function(
    executable: PythonResult,
    package: Optional[PythonResult] = None,
    name: Optional[PythonResult] = None,
    parameters: Optional[PythonResult] = None,
    arguments: Optional[PythonResult] = None,
    output: Optional[PythonResult] = None,
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
    return LaunchNode(
        _dataflow_to_launch_substitution(package),
        _dataflow_to_launch_substitution(executable),
        name=_dataflow_to_launch_substitution(name),
        # parameters=launch_params,
        output=_dataflow_to_launch_substitution(output, default='log'),
        arguments=_dataflow_to_launch_list(arguments),
    )


def get_package_share_directory_function(package: PythonResult):
    if not package.is_resolved:
        return UnresolvedPythonString()
    return f'/usr/share/ros/{package.value}'


LAUNCH_SYMBOLS = {
    'launch.LaunchDescription': launch_description_function,
    'launch.actions.DeclareLaunchArgument': declare_launch_argument_function,
    'launch.actions.IncludeLaunchDescription': include_launch_description_function,
    'launch.substitutions.LaunchConfiguration': launch_configuration_function,
    'launch_ros.actions.Node': node_function,
    'ament_index_python.packages.get_package_share_directory': get_package_share_directory_function,
    'launch.launch_description_sources.PythonLaunchDescriptionSource': python_launch_description_source_function,
    'launch.substitutions.ThisLaunchFileDir': ThisDirectorySubstitution,
}


def _prepare_builtin_symbols() -> Mapping[str, Any]:
    symbols = {}
    ns = SimpleNamespace()
    ns.path = SimpleNamespace()
    for key in dir(os.path):
        if key.startswith('_'):
            continue
        value = getattr(os.path, key)
        if callable(value):
            value = library_function_wrapper(key, 'os.path', value)
        setattr(ns.path, key, value)
    ns.environ = {
        'TURTLEBOT3_MODEL': 'hamburger',
        'LDS_MODEL': 'LDS-01',
    }
    symbols['os'] = ns
    return symbols


def _dataflow_to_string(value: PythonResult) -> str:
    if value.is_resolved:
        return str(value.value)
    return UNKNOWN_TOKEN


def _dataflow_to_launch_substitution(
    result: Optional[PythonResult],
    default: Optional[str] = None,
) -> Optional[LaunchSubstitutionResult]:
    if result is None:
        return None if default is None else const_text(default)
    if result.is_resolved:
        if isinstance(result.value, LaunchSubstitution):
            return const_substitution(result.value, source=result.source)
        return const_text(str(result.value), source=result.source)
    return unknown_substitution(source=result.source)


def _dataflow_to_launch_list(arg_list: Optional[PythonResult]) -> List[LaunchSubstitutionResult]:
    values = []
    if arg_list is not None:
        if arg_list.is_resolved:
            for arg in arg_list.value:
                values.append(_dataflow_to_launch_substitution(arg))
        else:
            values.append(unknown_substitution())  # FIXME
    return values


def get_python_launch_description(path: Path) -> LaunchDescription:
    if not path.is_file():
        raise FileNotFoundError(f'not a file: {path}')
    ext = path.suffix.lower()
    if ext != '.py':
        raise WrongFileTypeError(f'not a valid launch file: {path}')
    code = path.read_text(encoding='utf-8')
    ast = parse(code)

    # system = PythonLaunchSystemInterface(path, None)

    symbols = {
        'mymodule.MY_CONSTANT': 44,
        'mymodule.my_division': lambda a, b: (a.value // b.value) if a.is_resolved and b.is_resolved else None,
    }
    symbols.update(_prepare_builtin_symbols())
    symbols.update(LAUNCH_SYMBOLS)

    # TODO include launch arguments
    # TODO node parameters
    # TODO node remaps

    builder = from_ast(ast, symbols=symbols)
    return launch_description_from_program_graph(builder)


def launch_description_from_program_graph(graph: Any) -> LaunchDescription:
    subgraph, data = graph.subgraph_builder(LAUNCH_ENTRY_POINT).build()  # !!
    for variant_value in data.return_values.possible_values():
        # variant_value: VariantData[PythonResult]
        if not variant_value.condition.is_true:
            logger.error('variant_value is not true')
            continue  # FIXME
        if not variant_value.value.is_resolved:
            logger.error('variant_value is not resolved')
            continue  # FIXME
        launch_description = variant_value.value.value
        if not isinstance(launch_description, LaunchDescription):
            logger.error(f'variant_value is not a LaunchDescription: {repr(launch_description)}')
            continue  # FIXME
        return launch_description
    logger.error('unable to return a complete LaunchDescription')
    return LaunchDescription()  # FIXME
