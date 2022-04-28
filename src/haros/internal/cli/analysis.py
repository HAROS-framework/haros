# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""
Module that contains the command line sub-program.
"""

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, List, Tuple

import argparse
import logging
from pathlib import Path
import re

from haros.internal.fsutil import is_ros_package, is_workspace

###############################################################################
# Constants
###############################################################################

DEFAULT_PROJECT: Final[str] = 'my-ros-project'

RE_PACKAGE_NAME: Final[re.Pattern] = re.compile(r'([a-zA-Z][a-zA-Z0-9_]*)')

DEFAULT_FILE_NAME: Final[str] = 'project.haros.yaml'
DEFAULT_PATH: Final[Path] = Path.cwd() / DEFAULT_FILE_NAME

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Entry Point
###############################################################################


def subprogram(argv: List[str], settings: Dict[str, Any]) -> int:
    args = parse_arguments(argv)
    return run(args, settings)


###############################################################################
# Command-specific Functions
###############################################################################


def run(args: Dict[str, Any], settings: Dict[str, Any]) -> int:
    paths = args['paths']
    workspaces, packages, to_find = process_paths(paths)
    if not workspaces and not packages:
        if not to_find:
            logger.error('analysis: need at least one workspace or package')
            return 1
    logger.error(f'project: {action}')
    return 1


###############################################################################
# Argument Parsing
###############################################################################


def parse_arguments(argv: List[str]) -> Dict[str, Any]:
    msg = 'Run analyses over ROS workspaces and packages'
    parser = argparse.ArgumentParser(prog='haros analysis', description=msg)

    parser.add_argument(
        'paths',
        nargs='*',
        default=[Path.cwd()],
        type=Path,
        help='paths to workspaces or packages [default: "."]'
    )

    parser.add_argument(
        '-p',
        '--project'
        default=DEFAULT_PROJECT,
        help=f'paths to workspaces or packages [default: {DEFAULT_PROJECT}]'
    )

    args = parser.parse_args(args=argv)
    return vars(args)


###############################################################################
# Argument Validation and Processing
###############################################################################


def process_paths(paths: List[Path]) -> Tuple[List[Path], List[Path], List[str]]:
    workspaces = []
    packages = []
    to_find = []
    for path in paths:
        if is_ros_package(path):
            packages.append(path)
        elif is_workspace(path):
            workspaces.append(path)
        else:
            name = path.name
            if RE_PACKAGE_NAME.fullmatch(name):
                to_find.append(name)
            else:
                logger.error(f'analysis: neither a package nor a workspace: {str(path)}')
    for name in to_find:

    return workspaces, packages, to_find


###############################################################################
# Helper Functions
###############################################################################
