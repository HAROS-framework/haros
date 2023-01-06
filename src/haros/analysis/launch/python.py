# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, Optional

from pathlib import Path

from attrs import frozen

#from haros.analysis.python import query
from haros.analysis.python.dataflow import DataFlowValue, UnknownValue
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
        if isinstance(value, VariantData):
            if value.is_deterministic:
                value = value.get()
                if value.is_resolved:
                    value = value.value
                else:
                    return LaunchValue()
            else:
                return LaunchValue()
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
    _name = name.value if name.is_resolved else UNKNOWN_TOKEN
    if default_value is not None and default_value.is_resolved:
        value = default_value.value
    else:
        value = LaunchValue()
    if description is not None and description.is_resolved:
        _description = TextSubstitution(description.value)
    else:
        _description = TextSubstitution('')
    return LaunchArgument(_name, default_value=value, description=_description)


def launch_configuration_function(
    name: DataFlowValue,
    default: Optional[DataFlowValue] = None,
) -> LaunchConfiguration:
    _name = name.value if name.is_resolved else UNKNOWN_TOKEN
    value = None
    if default is not None:
        value = TextSubstitution(default.value) if default.is_resolved else LaunchValue()
    return LaunchConfiguration(_name, default_value=value)


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
    output: DataFlowValue = None,
) -> LaunchNode:
    _name = name.value if name and name.is_resolved else LaunchValue()
    _package = package.value if package.is_resolved else LaunchValue()
    _executable = executable.value if executable.is_resolved else LaunchValue()
    return LaunchNode(name=_name, package=_package, executable=_executable)


def get_package_share_directory_function(package: DataFlowValue):
    if not package.is_resolved:
        return UnknownValue()
    return f'/usr/share/ros/{package.value}'


def os_path_join_function(**args):
    print(f'os.path.join({args})')
    parts = []
    for arg in args:
        if arg.is_resolved:
            parts.append(arg.value)
        else:
            parts.append(UNKNOWN_TOKEN)
    return '/'.join(parts)


LAUNCH_SYMBOLS = {
    'launch.LaunchDescription': launch_description_function,
    'launch.actions.DeclareLaunchArgument': declare_launch_argument_function,
    'launch.actions.IncludeLaunchDescription': include_launch_description_function,
    'launch.substitutions.LaunchConfiguration': launch_configuration_function,
    'launch_ros.actions.Node': node_function,
    'ament_index_python.packages.get_package_share_directory': get_package_share_directory_function,
    'launch.launch_description_sources.PythonLaunchDescriptionSource': python_launch_description_source_function,
    'os.path.join': os_path_join_function,
}


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
    symbols.update(LAUNCH_SYMBOLS)
    symbols['launch.substitutions.ThisLaunchFileDir'] = system.get_this_launch_file_dir
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
