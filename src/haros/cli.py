# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""
Module that contains the command line program.

Why does this file exist, and why not put this in __main__?

  In some cases, it is possible to import `__main__.py` twice.
  This approach avoids that. Also see:
  https://click.palletsprojects.com/en/5.x/setuptools/#setuptools-integration

Some of the structure of this file came from this StackExchange question:
  https://softwareengineering.stackexchange.com/q/418600
"""

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, List, Optional

import argparse
import logging
import logging.config
from pathlib import Path
import sys

from haros import __version__ as current_version
from haros.internal import home
from haros.internal.cli import init, project
from haros.internal.plugins import load as load_plugins
from haros.internal.settings import load as load_settings, defaults as default_settings

###############################################################################
# Entry Point
###############################################################################


logger: Final[logging.Logger] = logging.getLogger(__name__)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_arguments(argv)
    cmd = args['cmd']

    logger.info('test info without config')
    logger.warning('test warning without config')
    logger.error('test error without config')

    # shortcut commands ------------------------------------
    if cmd == 'init':
        return init.subprogram(args.get('args'), default_settings())

    # setup phase ------------------------------------------
    try:
        homepath = home.find()
        settings = load_settings(homepath)
        _setup_logging(homepath, settings)
        logger.info('Logging configured.')
    except KeyboardInterrupt:
        print('Aborted manually.', file=sys.stderr)
        return 1
    except Exception as err:
        print('Unhandled exception during setup.', err, file=sys.stderr)
        return 1

    logger.info('Setup phase finished.')
    logger.info(f'Running {cmd} command.')
    # main phase -------------------------------------------
    rcode = 0
    try:
        logger.warning('Test warning message.')
        if cmd == 'echo-args':
            print(f'Arguments: {args}')
            print(f'Settings: {settings}')
        elif cmd == 'config':
            pass
        elif cmd == 'cache':
            pass
        elif cmd == 'project':
            logger.info(f'running subprogram: project')
            rcode = project.subprogram(args['args'], settings)
        elif cmd == 'analysis':
            plugins = load_plugins()
            logger.info(f'Running analysis with plugins {plugins}')
    except KeyboardInterrupt:
        logger.error('Aborted manually.')
        return 1
    except Exception as err:
        logger.exception('An unhandled exception crashed the application!', err)
        return 1

    if rcode == 0:  # success
        print(f'[HAROS] Command {cmd} executed successfully.')
    return rcode


###############################################################################
# Argument Parser
###############################################################################


def parse_arguments(argv: Optional[List[str]]) -> Dict[str, Any]:
    msg = 'The High-Assurance ROS Framework.'
    parser = argparse.ArgumentParser(description=msg)

    parser.add_argument(
        '--version',
        action='version',
        version=f'{current_version}',
        help='Prints the program version.'
    )

    parser.add_argument(
        'cmd',
        metavar='CMD',
        choices=[
            'init',
            'config',
            'project',
            'cache',
            'analysis',
            'echo-args',
        ],
        help='A concrete HAROS command to run.',
    )

    parser.add_argument(
        'args',
        metavar='ARG',
        nargs=argparse.ZERO_OR_MORE,
        help='Arguments for the HAROS command.',
    )

    args = parser.parse_args(args=argv)
    return vars(args)


###############################################################################
# Logging Configuration
###############################################################################

def _setup_logging(homepath: Path, settings: Dict[str, Any]) -> None:
    config = settings['logging']
    config['disable_existing_loggers'] = False
    filename = config['handlers']['logfile']['filename']
    path = homepath / 'logs' / filename
    path = path.resolve()
    config['handlers']['logfile']['filename'] = str(path)
    logging.config.dictConfig(config)
