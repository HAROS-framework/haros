# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""
Module that contains the command line sub-program.
"""

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, Mapping

import argparse
import logging
from pathlib import Path

from haros.analysis.launch import get_launch_description
from haros.analysis.python.dataflow import BUILTINS_MODULE
from haros.internal.interface import AnalysisSystemInterface
from haros.internal.settings import Settings
from haros.metamodel.builder.launch import model_from_description
from haros.metamodel.common import Result
from haros.metamodel.launch import LaunchFileFeature, NodeFeature
from haros.metamodel.ros import FileModel, NodeModel, PackageModel, ProjectModel

from haros.analysis.python.graph import ProgramGraphBuilder

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Entry Point
###############################################################################


def subprogram(argv: List[str], settings: Settings) -> int:
    args = parse_arguments(argv)
    return run(args, settings)


###############################################################################
# Command-specific Functions
###############################################################################


def run(args: Dict[str, Any], settings: Settings) -> int:
    try:
        path: Path = args['path'].resolve(strict=True)
    except FileNotFoundError:
        logger.error(f'debug: the file "{args["path"]}" does not exist')
        return 1
    if not path.is_file():
        logger.error(f'debug: not a file: "{path}"')
        return 1

    workspace = Path.cwd() / 'tests' / 'ws1'
    repo = workspace / 'src' / 'repo'
    system = AnalysisSystemInterface(
        workspace=str(workspace),
        packages={
            'turtlebot3': (repo / 'turtlebot3').as_posix(),
            'turtlebot3_bringup': (repo / 'turtlebot3_bringup').as_posix(),
            'turtlebot3_cartographer': (repo / 'turtlebot3_cartographer').as_posix(),
            'turtlebot3_description': (repo / 'turtlebot3_description').as_posix(),
            'turtlebot3_example': (repo / 'turtlebot3_example').as_posix(),
            'turtlebot3_navigation2': (repo / 'turtlebot3_navigation2').as_posix(),
            'turtlebot3_node': (repo / 'turtlebot3_node').as_posix(),
            'turtlebot3_teleop': (repo / 'turtlebot3_teleop').as_posix(),
            'nav2_bringup': (repo / 'nav2_bringup').as_posix(),
        },
        model=debugging_model(),
        parse_launch_description=get_launch_description,
    )

    launch_description = system.get_launch_description(path)
    model = model_from_description(path, launch_description, system)
    print('Launch Model:')
    print_launch_model(model)

    # code = path.read_text(encoding='utf-8')
    # ast = parse(code, path=path.as_posix())
    # symbols: Dict[str, Any] = _prepare_builtin_symbols()
    # symbols.update({
    #     f'{BUILTINS_MODULE}.__file__': str(path),
    #     f'{BUILTINS_MODULE}.open': _builtin_open(system),
    # })
    # symbols.update(LAUNCH_SYMBOLS)
    # builder = from_ast(ast, symbols=symbols)
    # print_subgraphs(builder)

    # graph, data = builder.subgraph_builder('main').build()

    # builder = get_launch_description(path)
    # graph, data = builder.build()
    # print(graph.pretty())
    # print('')
    # print('Variables:')
    # print_variables(data.variables)
    #
    # print_subgraphs(builder)

    # qualified_name = 'launch.LaunchDescription'
    # print(f'Find calls to: `{qualified_name}`')
    # calls = find_qualified_function_call(graph, qualified_name)
    # for call in calls:
    #     print(f'  > {call.name} (line {call.meta.line}, column {call.meta.column})')
    # if not calls:
    #     print('-- no function calls were found --')

    return 0


###############################################################################
# Helper Functions
###############################################################################


def debugging_model() -> ProjectModel:
    project = ProjectModel('debug')

    package = PackageModel('turtlebot3_bringup')
    project.packages[package.uid] = package
    file = FileModel('turtlebot3_bringup', 'launch/robot.launch.py')
    project.files[file.uid] = file
    package.files.append(file.uid)
    file = FileModel('turtlebot3_bringup', 'launch/turtlebot3_state_publisher.launch.py')
    project.files[file.uid] = file
    package.files.append(file.uid)

    package = PackageModel('turtlebot3_node')
    project.packages[package.uid] = package
    node = NodeModel('turtlebot3_node', 'turtlebot3_ros', rosname='turtlebot3_node')
    project.nodes[node.uid] = node
    return project


def print_launch_model(model: LaunchFileFeature):
    print('LaunchFileFeature:', model.id)
    for node in model.nodes.values():
        print_ros_node(node)
    if not model.nodes:
        print('<there are no nodes>')
    for fid in model.inclusions:
        print('Include:', fid)
    if not model.inclusions:
        print('<there are no inclusions>')


def print_ros_node(node: NodeFeature):
    print('ROS node:', node.rosname)
    print('  node:', node.node)
    print('  output:', node.output)
    print_dynamic_collection('arguments', node.arguments)
    print_dynamic_collection('parameters', node.parameters)
    print_dynamic_collection('remappings', node.remappings)


def print_dynamic_collection(name: str, result: Result[Any]):
    if result.is_resolved:
        if result.type.is_mapping:
            print(f'  {name}:')
            print_mapping(result.value, indent=4)
        elif result.type.is_iterable:
            print(f'  {name}:')
            print_list_of_values(result.value, indent=4)
        else:
            print(f'  {name}: {result.value}')
    else:
        print(f'  {name}: {result}')


def print_list_of_values(values: Iterable[Result[Any]], indent: int = 0):
    ws = ' ' * indent
    if values:
        for value in values:
            print(f'{ws}{value}')
    else:
        print(f'{ws}[]')


def print_mapping(mapping: Mapping[Result[Any], Result[Any]], indent: int = 0):
    ws = ' ' * indent
    if mapping:
        for key, value in mapping.items():
            text_value: str = str(value)
            try:
                if value.is_resolved:
                    if value.type.is_mapping:
                        text_value = str({str(k): str(v) for k, v in value.value.items()})
                    elif value.type.is_iterable:
                        text_value = str(list(map(str, value.value)))
                print(f'{ws}{key}: {text_value}')
            except AttributeError:
                logger.error(f'expected Result value for {key!r}: {value!r}')
    else:
        print(f'{ws}{{}}')


def print_subgraphs(builder: ProgramGraphBuilder):
    for name, statement in builder.nested_graphs.items():
        # full_name = f'{builder.name}/{name}'
        print('')
        print(f'>> {name}')

        subbuilder = builder.subgraph_builder(name)
        subgraph, data = subbuilder.build()
        print(subgraph.pretty())

        print(f'\nReachable:')
        for node in subgraph.nodes.values():
            if node.id == subgraph.root_id or not node.is_unreachable:
                print('')
                print(node.pretty())

        print(f'\nUnreachable:')
        for node in subgraph.nodes.values():
            if node.id != subgraph.root_id and node.is_unreachable:
                print('')
                print(node.pretty())

        print('')
        print('Variables:')
        print_variables(data.variables)
        print('')
        print('Return values:')
        for r in data.return_values.possible_values():
            print(' $', r)

        print_subgraphs(subbuilder)


def print_variables(variables):
    for name, var in variables.items():
        if var.has_values:
            for variant in var.possible_values():
                if variant.value.import_base == BUILTINS_MODULE:
                    continue
                if variant.value.import_base:
                    continue  # skip imported names
                print(f' #{name} = {variant}')
        else:
            print(f' #{name} has no definitions')


###############################################################################
# Argument Parsing
###############################################################################


def parse_arguments(argv: List[str]) -> Dict[str, Any]:
    parser = argparse.ArgumentParser(
        prog='haros debug',
        description='Manual tests and debugging',
    )

    parser.add_argument(
        'path',
        type=Path,
        help='path to the input file',
    )

    args = parser.parse_args(args=argv)
    return vars(args)
