# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, List, Optional, Mapping

import os
from pathlib import Path
from types import SimpleNamespace

from attrs import frozen

#from haros.analysis.python import query
from haros.analysis.python.dataflow import DataFlowValue, library_function_wrapper, UnknownValue
from haros.analysis.python.graph import from_ast
from haros.errors import ParseError
from haros.metamodel.builder.launch import AnalysisSystemInterface, LaunchModelBuilder
from haros.metamodel.common import VariantData
from haros.metamodel.launch import (
    LaunchArgument,
    LaunchConfiguration,
    LaunchDescription,
    LaunchEntity,
    LaunchInclusion,
    LaunchModel,
    LaunchNode,
    LaunchValue,
    TextSubstitution,
)
from haros.parsing.python import parse

###############################################################################
# Constants
###############################################################################

LAUNCH_ENTRY_POINT: Final[str] = 'generate_launch_description'

UNKNOWN_TOKEN: Final[str] = '{?}'

###############################################################################
# Interface
###############################################################################


@frozen
class PythonLaunchSystemInterface:
    file_path: Path
    system: AnalysisSystemInterface

    @property
    def environment(self) -> Mapping[str, str]:
        return self.system.environment

    def get_this_launch_file_dir(self) -> str:
        return self.file_path.parent.as_posix()


def python_launch_description_source_function(arg_list: DataFlowValue) -> LaunchValue:
    if not arg_list.is_resolved:
        return LaunchValue()
    parts = []
    for arg in arg_list.value:
        if not arg.is_resolved:
            return LaunchValue()
        value = arg.value
        assert not isinstance(value, VariantData), repr(value)
        if isinstance(value, LaunchConfiguration):
            parts.append(f'$(var {value.name})')
        else:
            parts.append(value)
    return TextSubstitution('/'.join(parts))  # FIXME


def launch_description_function(arg_list: DataFlowValue) -> LaunchDescription:
    values = []
    if arg_list.is_resolved:
        for arg in arg_list.value:
            values.append(arg.value if arg.is_resolved else LaunchEntity())
    return LaunchDescription(values)


def declare_launch_argument_function(
    name: DataFlowValue,
    default_value: DataFlowValue = None,
    description: DataFlowValue = None,
) -> LaunchArgument:
    return LaunchArgument(
        _dataflow_to_string(name),
        default_value=_dataflow_to_launch_value(default_value),
        description=_dataflow_to_launch_value(description),
    )


def launch_configuration_function(
    name: DataFlowValue,
    default: Optional[DataFlowValue] = None,
) -> LaunchConfiguration:
    return LaunchConfiguration(
        _dataflow_to_string(name),
        default_value=_dataflow_to_launch_value(default),
    )


def include_launch_description_function(
    source: DataFlowValue,
    launch_arguments: Optional[DataFlowValue] = None,
) -> LaunchInclusion:
    if source.is_resolved:
        if isinstance(source.value, LaunchValue):
            return LaunchInclusion(file=source.value)
    return LaunchInclusion()


def node_function(
    package: DataFlowValue,
    executable: DataFlowValue,
    name: Optional[DataFlowValue] = None,
    parameters: Optional[DataFlowValue] = None,
    arguments: Optional[DataFlowValue] = None,
    output: Optional[DataFlowValue] = None,
) -> LaunchNode:
    # docs: https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/node.py
    return LaunchNode(
        name=_dataflow_to_launch_value(name),
        package=_dataflow_to_launch_value(package),
        executable=_dataflow_to_launch_value(executable),
        output=_dataflow_to_launch_value(output, default='log'),
    )


def get_package_share_directory_function(package: DataFlowValue):
    if not package.is_resolved:
        return UnknownValue()
    return f'/usr/share/ros/{package.value}'


LAUNCH_SYMBOLS = {
    'launch.LaunchDescription': launch_description_function,
    'launch.actions.DeclareLaunchArgument': declare_launch_argument_function,
    'launch.actions.IncludeLaunchDescription': include_launch_description_function,
    'launch.substitutions.LaunchConfiguration': launch_configuration_function,
    'launch_ros.actions.Node': node_function,
    'ament_index_python.packages.get_package_share_directory': get_package_share_directory_function,
    'launch.launch_description_sources.PythonLaunchDescriptionSource': python_launch_description_source_function,
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


def _dataflow_to_string(value: DataFlowValue) -> str:
    if value.is_resolved:
        return str(value.value)
    return UNKNOWN_TOKEN


def _dataflow_to_launch_value(
    value: Optional[DataFlowValue],
    default: Optional[str] = None,
) -> Optional[LaunchValue]:
    if value is None:
        if default is None:
            return None
        return TextSubstitution(default)
    if value.is_resolved:
        return TextSubstitution(str(value.value))
    return LaunchValue()


def get_python_launch_model(path: Path) -> LaunchModel:
    if not path.is_file():
        raise ValueError(f'not a file: {path}')
    ext = path.suffix.lower()
    if ext != '.py':
        raise ValueError(f'not a valid launch file: {path}')
    code = path.read_text(encoding='utf-8')
    ast = parse(code)

    system = PythonLaunchSystemInterface(path, None)

    symbols = {
        'mymodule.MY_CONSTANT': 44,
        'mymodule.my_division': lambda a, b: (a.value // b.value) if a.is_resolved and b.is_resolved else None,
    }
    symbols.update(_prepare_builtin_symbols())
    symbols.update(LAUNCH_SYMBOLS)
    symbols['launch.substitutions.ThisLaunchFileDir'] = system.get_this_launch_file_dir

    # TODO include launch arguments
    # TODO node name if not present
    # TODO node parameters
    # TODO node remaps
    # TODO node main arguments

    graph = from_ast(ast, symbols=symbols)
    return graph
    # return launch_model_from_program_graph(path.name, graph)  # FIXME


def launch_model_from_program_graph(name: str, graph: Any) -> LaunchModel:
    subgraph, data = graph.subgraph_builder(LAUNCH_ENTRY_POINT).build()
    # FIXME possible KeyError from `subgraph_builder`
    for variant_value in data.return_values.possible_values():
        if not variant_value.condition.is_true:
            continue  # FIXME
        if not variant_value.value.is_resolved:
            continue  # FIXME
        builder = LaunchModelBuilder(name)
        builder.scope.set_unknown()
        return builder.build()
    return LaunchModel(name)  # FIXME
