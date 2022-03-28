# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""
Module that contains the command line sub-program.
"""

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, List

import argparse
import sys

###############################################################################
# Argument Parsing
###############################################################################


def parse_arguments(argv: List[str]) -> Dict[str, Any]:
    msg = 'haros init'
    parser = argparse.ArgumentParser(description=msg)

    parser.add_argument(
        'args', metavar='ARG', nargs=argparse.ZERO_OR_MORE, help='Arguments for the command.'
    )

    args = parser.parse_args(args=argv)
    return vars(args)


###############################################################################
# Command-specific Functions
###############################################################################


def run(args: Dict[str, Any], configs: Dict[str, Any]) -> None:
    print('Command init executed successfully.')
    print(f'init arguments: {args}')


###############################################################################
# Entry Point
###############################################################################


def main(argv: List[str], configs: Dict[str, Any]) -> int:
    args = parse_arguments(argv)
    try:
        run(args, configs)
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
    return 0  # success
