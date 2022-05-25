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

from haros.analysis.cmake import get_targets_from_cmake
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
    targets = get_targets_from_cmake(path)
    print()
    print('targets:')
    if not targets:
        print('<there are no targets>')
    else:
        for target in targets.values():
            t = '[exe]' if target.is_executable else '[lib]'
            print(f' > {t} {target.name}: {target.sources}')
            print('    <deps>', target.dependencies)
    return 0


###############################################################################
# Helper Functions
###############################################################################


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
