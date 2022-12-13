# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""
Module that contains the command line sub-program.
"""

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, List

import argparse
import logging
from pathlib import Path

from haros.analysis.launch import get_launch_model
from haros.analysis.python.graph import find_qualified_function_call
from haros.internal.settings import Settings

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
        logger.error(f'debug: the file "{path}" does not exist')
        return 1
    if not path.is_file():
        logger.error(f'debug: not a file: "{path}"')
        return 1

    graph, variables = get_launch_model(path)
    print(graph.pretty())
    print('\nVariables:')
    print_variables(variables)

    print_subgraphs(graph.name, graph)

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


def print_subgraphs(name, graph):
    for subgraph, variables in graph.nested_graphs.values():
        full_name = f'{name}/{subgraph.name}'
        print('')
        print(f'>> {name}')
        print(subgraph.pretty())

        for node in subgraph.nodes.values():
            if node.id != subgraph.root_id and node.is_unreachable:
                print('')
                print(node.pretty())

        print('')
        print('Variables')
        print_variables(variables)

        print_subgraphs(full_name, subgraph)


def print_variables(variables):
    for name, var in variables.items():
        if var.has_values:
            for variant in var.possible_values():
                if variant.value.import_base == '__builtins__':
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
