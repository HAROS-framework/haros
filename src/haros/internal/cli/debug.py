# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

"""
Module that contains the command line sub-program.
"""

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List

import argparse
import logging
from pathlib import Path
import re

import attr

from haros.internal.settings import Settings
from haros.parsing.cmake import CMakeArgument
from haros.parsing.cmake import parser as cmake_parser

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
    logger.info(f'debug: cmake parser on: {path}')
    parser = cmake_parser()
    text = path.read_text()
    tree = parser.parse(text)
    # print(tree.pretty())
    executables, libraries = get_nodes_from_cmake(tree, parser)
    print()
    print('executables:')
    if not executables:
        print('<there are no executables>')
    else:
        for key, value in executables.items():
            print(' >', key, ':', value)
    print()
    print('libraries:')
    if not libraries:
        print('<there are no libraries>')
    else:
        for key, value in libraries.items():
            print(' >', key, ':', value)
    return 0


###############################################################################
# Helper Functions
###############################################################################


def get_nodes_from_cmake(cmake, parser):
    context = CMakeContext(parser)
    for cmd in cmake.commands:
        args = context.process_arguments(cmd.arguments)
        if cmd.name == 'set':
            context.cmake_set(args)
        elif cmd.name == 'unset':
            context.cmake_unset(args)
        elif cmd.name == 'add_executable':
            context.cmake_add_executable(args)
        elif cmd.name == 'add_library':
            context.cmake_add_library(args)
        elif cmd.name == 'project':
            context.cmake_set(('PROJECT_NAME', args[0]))
    return context.executables, context.libraries


@attr.s(auto_attribs=True, slots=True, frozen=True)
class CMakeContext:
    parser: Any
    parent: Any = None
    variables: Dict[str, str] = attr.Factory(dict)
    environment: Dict[str, str] = attr.Factory(dict)
    cache: Dict[str, str] = attr.Factory(dict)
    executables: Dict[str, List[str]] = attr.Factory(dict)
    libraries: Dict[str, List[str]] = attr.Factory(dict)

    def process_arguments(self, arguments: List[CMakeArgument]) -> List[str]:
        if not arguments:
            return []
        # first pass: variable substitution
        args: List[str] = self.interpret_all(arg.value for arg in arguments)
        # second pass: join text and break it down again in separate arguments
        text: str = ' '.join(args)
        arguments = self.parser.parse_arguments(text)
        # there should be no further variable references at this point
        return [arg.value for arg in arguments]

    _RE_VARIABLE = re.compile(r'\${(\w+)}')

    def interpret(self, value: str) -> str:
        # TODO: $ENV{variable}
        # TODO: $CACHE{variable}
        match = self._RE_VARIABLE.search(value)
        if not match:
            return value
        i = 0
        parts = []
        while match:
            parts.append(value[i:match.start()])
            key = match.group(1)
            replacement = self.variables.get(key, self.cache.get(key, ''))
            parts.append(replacement)
            i = match.end()
            match = self._RE_VARIABLE.search(value, i)
        # completed a pass through the string
        if i < len(value):
            parts.append(value[i:])
        # must take nested variables into account
        return self.interpret(''.join(parts))

    def interpret_all(self, values: Iterable[str]) -> List[str]:
        return [self.interpret(v) for v in values]

    _RE_ENV_VAR = re.compile(r'ENV{(.+)}')

    def cmake_set(self, args: Iterable[str]):
        if len(args) < 1:
            raise ValueError('set() expects at least one argument')
        name = args[0]
        match = self._RE_ENV_VAR.fullmatch(name)
        if match:
            # environment variable
            name = match.group(1)
            if len(args) == 1:
                # clear variable from environment
                if name in self.environment:
                    del self.environment[name]
            else:
                # set environment variable
                self.environment[name] = args[1]
        else:
            if len(args) < 2:
                raise ValueError('set() expects a name and a value')
            values = []
            on_parent = False
            for i in range(1, len(args)):
                if args[i] == 'CACHE':
                    break
                elif args[i] == 'PARENT_SCOPE':
                    on_parent = True
                    break
                values.append(args[i])
            value = ' '.join(values)
            if args[i] == 'CACHE':
                var_type = args[i+1]
                docstring = args[i+2]
                force = i+3 < len(args) and args[i+3] == 'FORCE'
                if force or name not in cache:
                    self.cache[name] = (value, var_type, docstring)
            else:
                if on_parent:
                    if self.parent is None:
                        raise ValueError('no parent context')
                    self.parent.variables[name] = value
                else:
                    self.variables[name] = value

    def cmake_unset(self, args: Iterable[str]):
        if len(args) < 1:
            raise ValueError('unset() expects at least one argument')
        name = args[0]
        mapping = self.variables
        match = self._RE_ENV_VAR.fullmatch(name)
        if match:  # environment variable
            if len(args) > 1:
                raise ValueError(f'too many arguments: unset({", ".join(args)})')
            name = match.group(1)
            mapping = self.environment
        else:
            if len(args) > 2:
                raise ValueError(f'too many arguments: unset({", ".join(args)})')
            if len(args) == 2:
                if args[1] == 'CACHE':
                    mapping = self.cache
                elif args[1] == 'PARENT_SCOPE':
                    if not self.parent:
                        raise ValueError(f'no parent scope: unset({", ".join(args)})')
                    mapping = self.parent.variables
                else:
                    raise ValueError(f'unknown unset() argument: {args[1]}')
        if name in mapping:
            del mapping[name]

    _ADD_EXECUTABLE_OPTIONS = ('WIN32', 'MACOSX_BUNDLE', 'EXCLUDE_FROM_ALL')

    def cmake_add_executable(self, args: Iterable[str]):
        if len(args) < 1:
            raise ValueError('expected one or more arguments: add_executable()')

        name = args[0]
        if name in self.executables:
            raise ValueError('duplicate executable name:', name)

        sources = []
        self.executables[name] = sources
        for i in range(1, len(args)):
            if args[i] not in self._ADD_EXECUTABLE_OPTIONS:
                break
        if i >= len(args):
            # New in version 3.11: The source files can be omitted
            # if they are added later using target_sources().
            raise ValueError(f'no sources: add_executable({", ".join(args)})')
        for i in range(i, len(args)):
            sources.append(args[i])

    def cmake_add_library(self, args: Iterable[str]):
        if len(args) < 1:
            raise ValueError('expected one or more arguments: add_library()')

        # TODO Object Libraries
        if 'OBJECT' in args:
            return
        # TODO Interface Libraries
        if 'INTERFACE' in args:
            return
        # TODO Imported Libraries
        if 'IMPORTED' in args:
            return
        # TODO Alias Libraries
        if 'ALIAS' in args:
            return

        name = args[0]
        if name in self.libraries:
            raise ValueError('duplicate library name:', name)

        sources = []
        self.libraries[name] = sources

        i = 1
        if args[i] in ('STATIC', 'SHARED', 'MODULE'):
            i += 1
        if args[i] == 'EXCLUDE_FROM_ALL':
            i += 1

        if i >= len(args):
            # New in version 3.11: The source files can be omitted
            # if they are added later using target_sources().
            raise ValueError(f'no sources: add_library({", ".join(args)})')
        for i in range(i, len(args)):
            sources.append(args[i])


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
