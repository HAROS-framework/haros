# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""
Module that contains the command line sub-program.
"""

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Dict, Final, List, Optional

import argparse
import logging
from pathlib import Path

from attrs import frozen
from haros.analysis.launch import get_launch_description
from haros.analysis.launch.python import _prepare_builtin_symbols
from haros.analysis.python.dataflow import (
    BUILTINS_MODULE,
    TYPE_TOKEN_OBJECT,
    TYPE_TOKEN_STRING,
    MockObject,
    solved,
    solved_from,
    unknown_value,
)
from haros.internal.interface import AnalysisSystemInterface, PathType
from haros.internal.settings import Settings
from haros.metamodel.builder.launch import model_from_description
from haros.metamodel.common import Resolved, Result
from haros.metamodel.launch import LaunchFileFeature, NodeFeature
from haros.metamodel.ros import FileModel, NodeModel, PackageModel, ProjectModel

from haros.analysis.python.graph import ProgramGraphBuilder, from_ast
from haros.parsing.python import parse

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
        path = args['path'].resolve(strict=True)
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
        },
        model=debugging_model(),
        parse_launch_description=get_launch_description,
    )

    launch_description = system.get_launch_description(path)
    model = model_from_description(path, launch_description, system)
    print('Launch Model:')
    print_launch_model(model)

    # code = path.read_text(encoding='utf-8')
    # ast = parse(code)
    # symbols: Dict[str, Any] = _prepare_builtin_symbols()
    # symbols.update({
    #     f'{BUILTINS_MODULE}.__file__': str(path),
    #     f'{BUILTINS_MODULE}.open': _builtin_open(system),
    #     'mymodule.MY_CONSTANT': 44,
    #     'mymodule.my_division': lambda a, b: (a.value // b.value) if a.is_resolved and b.is_resolved else None,
    # })
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


# @frozen
# class LazyFileHandle(MockObject):
#     path: Result[PathType]
#     system: AnalysisSystemInterface
# 
#     def read(self, encoding: Optional[str] = None) -> Result[str]:
#         try:
#             if self.path.is_resolved:
#                 resolved_path: Resolved[str] = self.path
#                 text = self.system.read_text_file(resolved_path.value, encoding=encoding)
#                 return solved_from(text)
#         except ValueError:
#             pass
#         return unknown_value(type=TYPE_TOKEN_STRING)
# 
#     def __str__(self) -> str:
#         return f'{self.__class__.__name__}(path={self.path})'
# 
# 
# def _builtin_open(
#     system: AnalysisSystemInterface
# ) -> Callable[[Result[str], Optional[Result[str]]], Result[LazyFileHandle]]:
#     def wrapper(path: Result[str], mode: Optional[Result[str]] = None) -> Result[LazyFileHandle]:
#         if not path.is_resolved:
#             return unknown_value()
#         if mode is None:
#             mode = solved_from('r')
#         if not mode.is_resolved or mode.value != 'r':
#             return unknown_value()
#         return solved(TYPE_TOKEN_OBJECT, LazyFileHandle(path, system))
#     return wrapper


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


def print_dynamic_collection(name, result):
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


def print_list_of_values(values, indent=0):
    ws = ' ' * indent
    if values:
        for value in values:
            print(f'{ws}{value}')
    else:
        print(f'{ws}[]')


def print_mapping(mapping, indent=0):
    ws = ' ' * indent
    if mapping:
        for key, value in mapping.items():
            print(f'{ws}{key}: {value}')
    else:
        print(f'{ws}{{}}')


def print_subgraphs(builder: ProgramGraphBuilder):
    for name, statement in builder.nested_graphs.items():
        # full_name = f'{builder.name}/{name}'
        print('')
        print(f'>> {name}')

        subbuilder = builder.subgraph_builder(name)
        print('test 1')
        subgraph, data = subbuilder.build()
        print('test 2')
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
