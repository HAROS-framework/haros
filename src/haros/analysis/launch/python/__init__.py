# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final

import logging
from pathlib import Path

from haros.analysis.launch.python.mocks import LAUNCH_SYMBOLS, LaunchDescriptionMock
from haros.analysis.python.dataflow import BUILTINS_MODULE
from haros.analysis.python.graph import ProgramGraphBuilder, from_ast
from haros.analysis.python.mocks import standard_symbols
from haros.errors import WrongFileTypeError
from haros.internal.interface import AnalysisSystemInterface
from haros.metamodel.launch import LaunchDescription
from haros.metamodel.result import Result
from haros.parsing.python import parse

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

LAUNCH_ENTRY_POINT: Final[str] = 'generate_launch_description'

###############################################################################
# Interface
###############################################################################


def get_python_launch_description(path: Path, system: AnalysisSystemInterface) -> LaunchDescription:
    if not path.is_file():
        raise FileNotFoundError(f'not a file: {path}')
    ext = path.suffix.lower()
    if ext != '.py':
        raise WrongFileTypeError(f'not a valid launch file: {path}')
    code = path.read_text(encoding='utf-8')
    ast = parse(code, path=path.as_posix())

    env = {
        'TURTLEBOT3_MODEL': 'burger',
        'LDS_MODEL': 'LDS-01',
    }

    symbols = standard_symbols(system)
    symbols.update(LAUNCH_SYMBOLS)
    symbols['os'].environ.update(env)
    symbols[f'{BUILTINS_MODULE}.__file__'] = path.as_posix()

    # TODO include launch arguments
    # TODO node parameters
    # TODO node remaps

    builder: ProgramGraphBuilder = from_ast(ast, symbols=symbols)
    return launch_description_from_program_graph(builder)


def launch_description_from_program_graph(graph: ProgramGraphBuilder) -> LaunchDescription:
    subgraph, data = graph.subgraph_builder(LAUNCH_ENTRY_POINT).build()  # !!
    for variant_value in data.return_values.possible_values():
        if not variant_value.condition.is_true:
            logger.error('variant_value is not true')
            continue  # FIXME
        if not variant_value.value.is_resolved:
            logger.error('variant_value is not resolved')
            continue  # FIXME
        launch_description = variant_value.value.value
        if not isinstance(launch_description, LaunchDescriptionMock):
            logger.error(f'variant_value is not a LaunchDescription: {repr(launch_description)}')
            continue  # FIXME
        return launch_description._haros_freeze()
    logger.error('unable to return a complete LaunchDescription')
    return LaunchDescription(Result.of_tuple())  # FIXME
