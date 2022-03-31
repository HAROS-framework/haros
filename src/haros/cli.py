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

from typing import Any, Dict, List, Optional

import argparse
import sys

from haros import __version__ as current_version
from haros.internal import home
from haros.internal.cli import init
from haros.internal.settings import load as load_settings, defaults as default_settings

###############################################################################
# Entry Point
###############################################################################


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_arguments(argv)
    cmd = args['cmd']

    # shortcut commands ------------------------------------
    if cmd == 'init':
        return init.subprogram(args.get('args'), default_settings())

    # setup phase ------------------------------------------
    try:
        homepath = home.find()
        settings = load_settings(homepath)
        # TODO logging; see
        # https://stackoverflow.com/questions/13733552/logger-configuration-to-log-to-file-and-print-to-stdout?rq=1
    except KeyboardInterrupt:
        print('Aborted manually.', file=sys.stderr)
        return 1
    except Exception as err:
        print('Unhandled exception during setup.', err)
        return 1

    # main phase -------------------------------------------
    try:
        if cmd == 'echo-args':
            print(f'Arguments: {args}')
            print(f'Settings: {settings}')
        elif cmd == 'config':
            pass
        elif cmd == 'cache':
            pass
        elif cmd == 'project':
            pass
        elif cmd == 'analysis':
            pass
    except KeyboardInterrupt:
        print('Aborted manually.', file=sys.stderr)
        return 1
    except Exception as err:
        # In real code the `except` would probably be less broad.
        # Turn exceptions into appropriate logs and/or console output.

        print('An unhandled exception crashed the application!', err)

        # Non-zero return code to signal error.
        # It can, of course, be more fine-grained than this general code.
        return 1

    print(f'[HAROS] Command {cmd} executed successfully.')
    return 0  # success


###############################################################################
# Argument Parsing
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
