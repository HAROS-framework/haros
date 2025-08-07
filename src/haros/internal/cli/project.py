# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""Module that contains the command line sub-program."""

###############################################################################
# Imports
###############################################################################

from typing import Any, Final

import argparse
from collections.abc import Mapping
import logging
from pathlib import Path

from haros.internal.settings import Settings

###############################################################################
# Constants
###############################################################################

DEFAULT_FILE_NAME: Final[str] = 'project.haros.yaml'
DEFAULT_PATH: Final[Path] = Path.cwd() / DEFAULT_FILE_NAME

logger: Final[logging.Logger] = logging.getLogger(__name__)

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
    action = args['action']
    if action == 'new':
        return action_new(args)
    if action == 'build':
        return action_build(args)
    logger.error(f'project: {action}')
    return 1


def action_new(args: Mapping[str, Any]) -> int:
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


def action_build(args: Mapping[str, Any]) -> int:
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


def parse_arguments(argv: list[str]) -> dict[str, Any]:
    msg = 'haros project'
    parser = argparse.ArgumentParser(prog='haros project', description=msg)

    subparsers = parser.add_subparsers(dest='action', help='desired action')
    _parser_new(subparsers)
    _parser_build(subparsers)

    args = parser.parse_args(args=argv)
    return vars(args)


def _parser_new(subparsers):
    parser = subparsers.add_parser('new')

    parser.add_argument('name', help='name of the new project')

    parser.add_argument(
        'path',
        nargs='?',
        default=DEFAULT_PATH,
        type=Path,
        help=f'path of the generated file [{DEFAULT_PATH}]',
    )


def _parser_build(subparsers):
    parser = subparsers.add_parser('build')

    # default_path = Path.cwd() / 'project.haros.yaml'
    parser.add_argument(
        'path',
        nargs='?',
        default=DEFAULT_PATH,
        type=Path,
        help=f'path to the project file [{DEFAULT_PATH}]',
    )
