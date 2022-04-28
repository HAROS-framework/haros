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
    action = args['action']
    if action == 'new':
        return action_new(args)
    if action == 'build':
        return action_build(args)
    logger.error(f'project: {action}')
    return 1


def action_new(args: Dict[str, Any]) -> int:
    name = args['name']
    path = args['path']
    logger.info(f'project: new ({name}, {path})')
    try:
        path = path.resolve(strict=True)
        if path.is_dir():
            path = path / DEFAULT_FILE_NAME
        if path.exists():
            logger.error(f'project: "{path}" already exists')
            return 1
    except FileNotFoundError:
        pass  # this is ok
    try:
        path.write_text(f'project: {name}', encoding='utf-8')
    except FileNotFoundError:
        logger.error(f'project: unable to write to "{path}"')
        return 1
    return 0  # success


def action_build(args: Dict[str, Any]) -> int:
    path = args['path']
    logger.info(f'project: build ({path})')
    try:
        path = path.resolve(strict=True)
        if path.is_dir():
            path = path / DEFAULT_FILE_NAME
            path = path.resolve(strict=True)
            if not path.is_file():
                logger.error(f'project: "{path}" is not a file')
                return 1
        text = path.read_text(encoding='utf-8')
        logger.info(f'project: contents:\n{text}')
    except FileNotFoundError:
        logger.error(f'project: "{path}" is not a file')
        return 1
    return 0  # success


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
    return workspaces, packages, to_find
