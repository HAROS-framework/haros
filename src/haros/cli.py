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
from haros.internal.config import load as load_configs
from haros.internal.init import cli as cli_init

###############################################################################
# Argument Parsing
###############################################################################


def parse_arguments(argv: Optional[List[str]]) -> Dict[str, Any]:
    msg = 'A short description of the project.'
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
        choices=['init', 'analysis', 'config', 'echo-args'],
        help='A concrete HAROS command to run.',
    )

    parser.add_argument(
        'args', metavar='ARG', nargs=argparse.ZERO_OR_MORE, help='Arguments for the command.'
    )

    args = parser.parse_args(args=argv)
    return vars(args)


###############################################################################
# Commands
###############################################################################


def cmd_switch(args: Dict[str, Any], configs: Dict[str, Any]) -> None:
    cmd = args['cmd']
    if cmd == 'echo-args':
        print(f'Arguments: {args}')
        print(f'Configurations: {configs}')
    elif cmd == 'init':
        cli_init.main(args.get('args'), configs)
    elif cmd == 'config':
        pass
    return 0  # success


###############################################################################
# Entry Point
###############################################################################


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_arguments(argv)

    try:
        # Load additional config files here, e.g., from a path given via args.
        # Alternatively, set sane defaults if configuration is missing.
        # settings = load_configs(args)
        # loading settings will depend on the commands
        settings = {}
        return cmd_switch(args, settings)

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
