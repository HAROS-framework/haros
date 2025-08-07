# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, Iterable

from collections.abc import Mapping
import logging
from pathlib import Path
import re

from attrs import field, frozen

from haros.parsing.cmake import CMakeArgument, parser as cmake_parser

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Data Structures
###############################################################################


@frozen
class CMakeTarget:
    name: str
    is_executable: bool = True
    sources: list[str] = field(factory=list)
    dependencies: list[str] = field(factory=list)

    @property
    def is_library(self) -> bool:
        return not self.is_executable


@frozen
class CMakeContext:
    parser: Any
    parent: Any = None
    variables: Mapping[str, str] = field(factory=dict)
    environment: Mapping[str, str] = field(factory=dict)
    cache: Mapping[str, str] = field(factory=dict)
    targets: Mapping[str, CMakeTarget] = field(factory=dict)

    def process_arguments(self, arguments: list[CMakeArgument]) -> list[str]:
        if not arguments:
            return []
        # first pass: variable substitution
        args: list[str] = self.interpret_all(arg.value for arg in arguments)
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
            parts.append(value[i : match.start()])
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

    def interpret_all(self, values: Iterable[str]) -> list[str]:
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
                var_type = args[i + 1]
                docstring = args[i + 2]
                force = i + 3 < len(args) and args[i + 3] == 'FORCE'
                if force or name not in self.cache:
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
        if name in self.targets:
            raise ValueError('duplicate executable name:', name)

        target = CMakeTarget(name, is_executable=True)
        self.targets[name] = target
        for i in range(1, len(args)):
            if args[i] not in self._ADD_EXECUTABLE_OPTIONS:
                break
        if i >= len(args):
            # New in version 3.11: The source files can be omitted
            # if they are added later using target_sources().
            raise ValueError(f'no sources: add_executable({", ".join(args)})')
        for j in range(i, len(args)):
            target.sources.append(args[j])

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
        if name in self.targets:
            raise ValueError('duplicate library name:', name)

        target = CMakeTarget(name, is_executable=False)
        self.targets[name] = target

        i = 1
        if args[i] in ('STATIC', 'SHARED', 'MODULE'):
            i += 1
        if args[i] == 'EXCLUDE_FROM_ALL':
            i += 1

        if i >= len(args):
            # New in version 3.11: The source files can be omitted
            # if they are added later using target_sources().
            raise ValueError(f'no sources: add_library({", ".join(args)})')
        for j in range(i, len(args)):
            target.sources.append(args[j])

    def cmake_ament_target_dependencies(self, args: Iterable[str]):
        msg = '{}: ament_target_dependencies({})'
        if len(args) < 2:
            raise ValueError(msg.format('too few arguments', ', '.join(args)))

        target = self.targets.get(args[0])
        if target is None:
            raise ValueError(msg.format('invalid target', ', '.join(args)))

        target.dependencies.extend(args[1:])


###############################################################################
# Interface
###############################################################################


def get_targets_from_cmake(path: Path) -> list[CMakeTarget]:
    logger.debug(f'get_targets_from_cmake("{path}")')
    parser = cmake_parser()
    text = path.read_text()
    cmake = parser.parse(text)
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
        elif cmd.name == 'ament_target_dependencies':
            context.cmake_ament_target_dependencies(args)
        elif cmd.name == 'project':
            context.cmake_set(('PROJECT_NAME', args[0]))
    return context.targets
