# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""Module that contains the command line sub-program."""

###############################################################################
# Imports
###############################################################################

from typing import Any

import argparse
from collections.abc import Mapping
from pathlib import Path

from haros.internal import home
from haros.internal.settings import Settings

###############################################################################
# Entry Point
###############################################################################


def subprogram(argv: list[str], settings: Settings) -> int:
    args = parse_arguments(argv)
    return run(args, settings)


###############################################################################
# Command-specific Functions
###############################################################################


def run(args: Mapping[str, Any], settings: Settings) -> int:
    path = args['path'].resolve()
    print('Initializing HAROS home at: ' + str(path))
    home.make_at(path, overwrite=True)
    return 0  # success


###############################################################################
# Argument Parsing
###############################################################################


def parse_arguments(argv: list[str]) -> dict[str, Any]:
    msg = 'haros init'
    parser = argparse.ArgumentParser(prog='haros init', description=msg)

    parser.add_argument(
        'path',
        nargs='?',
        default=home.DEFAULT_PATH,
        type=Path,
        help=f'HAROS home directory path. Defaults to {home.DEFAULT_PATH}.',
    )

    args = parser.parse_args(args=argv)
    return vars(args)
