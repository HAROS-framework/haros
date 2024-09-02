# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, NewType, Optional, Set, Tuple, Union

import importlib
import logging
from pathlib import Path
from types import ModuleType

from attrs import field, frozen
from enum import Enum, unique

from haros.metamodel.common import (
    TYPE_TOKEN_ANYTHING,
    TYPE_TOKEN_LIST,
    TYPE_TOKEN_STRING,
    IterableType,
    Resolved,
    Result,
    TrackedCode,
    UnresolvedIterable,
    UnresolvedString,
)
from haros.metamodel.logic import TRUE, LogicValue
from haros.metamodel.ros import RosName, RosNodeModel

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# ROS Launch Substitutions
###############################################################################


@frozen
class LaunchScopeContext:
    def get(self, name: str) -> Optional[Result]:
        logger.debug(f'{self.__class__.__name__}.get({name!r})')
        raise NotImplementedError()

    def get_arg(self, name: str) -> Optional[Result]:
        logger.debug(f'{self.__class__.__name__}.get_arg({name!r})')
        raise NotImplementedError()

    def set(self, name: str, value: Result):
        logger.debug(f'{self.__class__.__name__}.set({name!r}, {value!r})')
        raise NotImplementedError()

    def set_unknown(self, name: str, source: Optional[TrackedCode] = None):
        return self.set(name, Result.unknown_value(source=source))

    def set_text(self, name: str, text: str, source: Optional[TrackedCode] = None):
        return self.set(name, Resolved.from_string(text, source=source))

    def compute_anon_name(self, name: str) -> str:
        # as seen in the official distribution
        import os
        import random
        import socket
        import sys
        name = f'{name}_{socket.gethostname()}_{os.getpid()}_{random.randint(0, sys.maxsize)}'
        name = name.replace('.', '_')
        name = name.replace('-', '_')
        return name.replace(':', '_')

    def get_this_launch_file(self) -> str:
        logger.debug(f'{self.__class__.__name__}.get_this_launch_file()')
        raise NotImplementedError()

    def get_this_launch_file_dir(self) -> str:
        logger.debug(f'{self.__class__.__name__}.get_this_launch_file_dir()')
        raise NotImplementedError()


# For a full list of substitutions see
# https://github.com/ros2/launch/tree/galactic/launch/launch/substitutions


@frozen
class LaunchSubstitution:
    @property
    def is_text(self) -> bool:
        return False

    @property
    def is_python_expression(self) -> bool:
        return False

    @property
    def is_configuration(self) -> bool:
        return False

    @property
    def is_package_share(self) -> bool:
        return False

    @property
    def is_argument(self) -> bool:
        return False

    @property
    def is_environment(self) -> bool:
        return False

    @property
    def is_find_executable(self) -> bool:
        return False

    @property
    def is_local(self) -> bool:
        return False

    @property
    def is_command(self) -> bool:
        return False

    @property
    def is_anon_name(self) -> bool:
        return False

    @property
    def is_this_file(self) -> bool:
        return False

    @property
    def is_this_dir(self) -> bool:
        return False

    @property
    def is_concatenation(self) -> bool:
        return False

    @property
    def is_path_join(self) -> bool:
        return False

    @property
    def is_equals(self) -> bool:
        return False

    @property
    def is_not_equals(self) -> bool:
        return False

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        logger.debug(f'{self.__class__.__name__}.resolve({ctx!r}, {source!r})')
        raise NotImplementedError

    def __str__(self) -> str:
        return '$(?)'


def unknown_substitution(source: Optional[TrackedCode] = None) -> Result[LaunchSubstitution]:
    return Result(LaunchSubstitution, source)


def const_substitution(
    sub: LaunchSubstitution,
    source: Optional[TrackedCode] = None,
) -> Result[LaunchSubstitution]:
    return Resolved(type(sub), source, sub)


def substitute(
    substitution: Optional[Result[LaunchSubstitution]],
    context: LaunchScopeContext,
    source: Optional[TrackedCode] = None,
) -> Result[str]:
    if substitution is None:
        return Result.unknown_value(type=TYPE_TOKEN_STRING, source=source)
    if substitution.is_resolved:
        return substitution.value.resolve(context, source=(source or substitution.source))
    return substitution.cast_to(TYPE_TOKEN_STRING)


def substitute_optional(
    substitution: Optional[Result[LaunchSubstitution]],
    context: LaunchScopeContext,
    source: Optional[TrackedCode] = None,
) -> Result[str]:
    if substitution is None:
        return None
    if substitution.is_resolved:
        return substitution.value.resolve(context, source=(source or substitution.source))
    return substitution.cast_to(TYPE_TOKEN_STRING)


def _to_sub(arg: Result[Union[None, str, LaunchSubstitution]]) -> Result[LaunchSubstitution]:
    if not arg.is_resolved:
        return arg.cast_to(TYPE_TOKEN_ANYTHING)
    if arg.value is None:
        return Resolved.from_value(TextSubstitution(''))
    if isinstance(arg.value, LaunchSubstitution):
        return arg
    if isinstance(arg.value, str):
        return Resolved.from_value(TextSubstitution(arg.value), source=arg.source)
    return Result.unknown_value(type=TYPE_TOKEN_ANYTHING, source=arg.source)


def _to_sub_list(
    arg: Result[Union[None, str, LaunchSubstitution, Iterable[Result[Union[None, str, LaunchSubstitution]]]]],
) -> Result[Iterable[Result[LaunchSubstitution]]]:
    if not arg.is_resolved:
        return arg.cast_to(TYPE_TOKEN_LIST)
    if arg.value is None:
        return Resolved.from_value(TextSubstitution(''))
    if isinstance(arg.value, str):
        arg = Resolved.from_value(TextSubstitution(arg.value), source=arg.source)
    if isinstance(arg.value, IterableType):
        values = [_to_sub(item) for item in arg.value]
        return Resolved.from_list(values, source=arg.source)
    if isinstance(arg.value, LaunchSubstitution):
        return Resolved.from_list([arg], source=arg.source)
    return Result.unknown_value(type=TYPE_TOKEN_LIST, source=arg.source)


@frozen
class TextSubstitution(LaunchSubstitution):
    value: str

    @property
    def is_text(self) -> bool:
        return True

    def resolve(
        self,
        _ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return Resolved.from_string(self.value, source=source)

    def __str__(self) -> str:
        return self.value


def const_text(text: str, source: Optional[TrackedCode] = None) -> Result[LaunchSubstitution]:
    return const_substitution(TextSubstitution(text), source=source)


@frozen
class LaunchConfiguration(LaunchSubstitution):
    name: str
    default_value: Optional[Result[LaunchSubstitution]] = None

    @property
    def is_configuration(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        value = ctx.get(self.name)
        if value is not None:
            return value
        value = substitute(self.default_value, ctx, source=source)
        ctx.set(self.name, value)
        return value

    def __str__(self) -> str:
        return f'$(var {self.name} {self.default_value})'


@frozen
class PackageShareDirectorySubstitution(LaunchSubstitution):
    """Custom substitution to evaluate share directory lazily."""

    package: Result[LaunchSubstitution]

    @property
    def is_package_share(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return super().resolve(ctx, source)

    def __str__(self) -> str:
        return f'$(share {self.package})'


@frozen
class LaunchArgumentSubstitution(LaunchSubstitution):
    """
    This substitution gets the value of a launch description argument,
    as a string, by name.
    """

    name: str

    @property
    def is_argument(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        _source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return ctx.get_arg(self.name)

    def __str__(self) -> str:
        return f'$(arg {self.name})'


def _default_python_modules() -> Result[Iterable[Result[LaunchSubstitution]]]:
    math = Resolved.from_value(TextSubstitution('math'))
    return Resolved.from_tuple((math,))


@frozen
class PythonExpressionSubstitution(LaunchSubstitution):
    """
    Substitution that can access contextual local variables.

    The expression may contain Substitutions, but must return something that can
    be converted to a string with `str()`.
    It also may contain math symbols and functions.
    """

    # https://github.com/ros2/launch/blob/rolling/launch/launch/substitutions/python_expression.py
    expression: Result[Iterable[Result[LaunchSubstitution]]] = field(converter=_to_sub_list)
    modules: Result[Iterable[Result[LaunchSubstitution]]] = field(
        converter=_to_sub_list,
        factory=_default_python_modules,
    )

    @property
    def is_python_expression(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        if not self.expression.is_resolved or not self.modules.is_resolved:
            return Result.unknown_value(type=TYPE_TOKEN_STRING, source=source)
        known_parts: List[str] = []
        all_parts: List[Optional[str]] = []
        for part in self.expression.value:
            value = substitute(part, ctx, source=source)
            if value.is_resolved:
                known_parts.append(value.value)
                all_parts.append(value.value)
            else:
                all_parts.append(None)
        if len(known_parts) != len(all_parts):
            return UnresolvedString.unknown_value(parts=all_parts, source=self.expression.source)
        expression = ''.join(known_parts)
        # FIXME avoid eval if possible
        expression_locals: Dict[str, Any] = {}
        for item in self.modules.value:
            value = substitute(item, ctx, source=source)
            if not value.is_resolved:
                return value
            name: str = value.value
            module: ModuleType = importlib.import_module(name)
            expression_locals[module.__name__] = module
            if module.__name__ == 'math':
                expression_locals.update(vars(module))
        try:
            end_result = str(eval(expression, {}, expression_locals))
            return Resolved.from_string(end_result, source=source)
        except:
            pass
        return Result.unknown_value(type=TYPE_TOKEN_STRING, source=source)

    def __str__(self) -> str:
        return f'$(python {self.expression})'


@frozen
class EnvironmentSubstitution(LaunchSubstitution):
    """
    This substitution gets an environment variable value,
    as a string, by name.
    """

    name: str

    @property
    def is_environment(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return super().resolve(ctx, source)

    def __str__(self) -> str:
        return f'$(env {self.name})'


@frozen
class FindExecutableSubstitution(LaunchSubstitution):
    """
    This substitution locates the full path to an executable
    on the PATH if it exists.
    """

    name: str

    @property
    def is_find_executable(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return super().resolve(ctx, source)

    def __str__(self) -> str:
        return f'$(find {self.name})'


@frozen
class LocalSubstitution(LaunchSubstitution):
    """
    This substitution gets a "local" variable out of the context.
    This is a mechanism that allows a "parent" action to pass
    information to sub actions.
    """

    expression: str

    @property
    def is_local(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return super().resolve(ctx, source)

    def __str__(self) -> str:
        return f'$(local {self.expression})'


@frozen
class CommandSubstitution(LaunchSubstitution):
    """
    Substitution that gets the output of a command as a string.
    If the command is not found or fails a `SubstitutionFailure` error is raised.
    Behavior on stderr output is configurable, see constructor.
    """

    """
    command: command to be executed. The substitutions will be performed, and
        `shlex.split` will be used on the result.
    """
    command: Result[LaunchSubstitution]

    """
    :on_stderr: specifies what to do when there is stderr output.
    Can be one of:
    - 'fail': raises `SubstitutionFailere` when stderr output is detected.
    - 'ignore': `stderr` output is ignored.
    - 'warn': The `stderr` output is ignored, but a warning is logged if detected.
    - 'capture': The `stderr` output will be captured, together with stdout.
    It can also be a substitution, that results in one of those four options.
    """
    # on_stderr: LaunchSubstitution  # TODO

    @property
    def is_command(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return super().resolve(ctx, source)

    def __str__(self) -> str:
        return f'$(cmd {self.command})'


@frozen
class AnonymousNameSubstitution(LaunchSubstitution):
    """
    Generates an anonymous id based on name.
    Name itself is a unique identifier: multiple uses of anon with
    the same parameter name will create the same "anonymized" name.
    """

    name: Result[LaunchSubstitution]

    @property
    def is_anon_name(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        value = substitute(self.name, ctx, source=source)
        if value.is_resolved:
            name = ctx.compute_anon_name(value.value)
            value = Resolved.from_string(name, source=source)
        return value

    def __str__(self) -> str:
        return f'$(anon {self.name})'


@frozen
class ThisLaunchFileSubstitution(LaunchSubstitution):
    """Substitution that returns the absolute path to the current launch file."""

    @property
    def is_this_file(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return Resolved.from_string(ctx.get_this_launch_file(), source=source)

    def __str__(self) -> str:
        return '$(this-launch-file)'


@frozen
class ThisDirectorySubstitution(LaunchSubstitution):
    """
    Substitution that returns the absolute path to the current launch file's
    containing directory.
    """

    @property
    def is_this_dir(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return Resolved.from_string(ctx.get_this_launch_file_dir(), source=source)

    def __str__(self) -> str:
        return '$(this-launch-file-dir)'


@frozen
class ConcatenationSubstitution(LaunchSubstitution):
    parts: Iterable[Result[LaunchSubstitution]]

    @property
    def is_concatenation(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        known_parts: List[str] = []
        all_parts: List[Optional[str]] = []
        for part in self.parts:
            value = substitute(part, ctx, source=source)
            if value.is_resolved:
                known_parts.append(value.value)
                all_parts.append(value.value)
            else:
                all_parts.append(None)
        if len(known_parts) != len(all_parts):
            return UnresolvedString.unknown_value(all_parts, source=source)
        return Resolved.from_string(''.join(known_parts), source=source)

    def __str__(self) -> str:
        return ''.join(map(str, self.parts))


@frozen
class PathJoinSubstitution(LaunchSubstitution):
    parts: Iterable[Result[LaunchSubstitution]]

    @property
    def is_path_join(self) -> bool:
        return True

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        path = Path()
        is_unknown: bool = False
        all_parts: List[Optional[str]] = []
        for part in self.parts:
            value = substitute(part, ctx, source=source)
            if value.is_resolved:
                path = path / str(value)
                all_parts.append(value.value)
            else:
                is_unknown = True
                all_parts.append(None)
        if is_unknown:
            return UnresolvedString.unknown_value(parts=all_parts, source=source)
        return Resolved.from_string(path.as_posix(), source=source)

    def __str__(self) -> str:
        return f'$(join {" ".join(map(str, self.parts))})'


@frozen
class EqualsSubstitution(LaunchSubstitution):
    # https://github.com/ros2/launch/blob/rolling/launch/launch/substitutions/equals_substitution.py
    argument1: Result[LaunchSubstitution] = field(converter=_to_sub)
    argument2: Result[LaunchSubstitution] = field(converter=_to_sub)

    @property
    def is_equals(self) -> bool:
        return True

    @property
    def a(self) -> Result[LaunchSubstitution]:
        return self.argument1

    @property
    def b(self) -> Result[LaunchSubstitution]:
        return self.argument2

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        a = substitute(self.argument1, ctx, source=source)
        b = substitute(self.argument2, ctx, source=source)
        if not a.is_resolved:
            return Result.unknown_value(type=TYPE_TOKEN_STRING, source=a.source)
        if not b.is_resolved:
            return Result.unknown_value(type=TYPE_TOKEN_STRING, source=b.source)
        if a.value == b.value:  # FIXME
            return Resolved.from_string('true', source=source)
        return Resolved.from_string('false', source=source)

    def __str__(self) -> str:
        return f'$(eq {self.argument1} {self.argument2})'


@frozen
class NotEqualsSubstitution(LaunchSubstitution):
    # https://github.com/ros2/launch/blob/rolling/launch/launch/substitutions/not_equals_substitution.py
    argument1: Result[LaunchSubstitution] = field(converter=_to_sub)
    argument2: Result[LaunchSubstitution] = field(converter=_to_sub)

    @property
    def is_not_equals(self) -> bool:
        return True

    @property
    def a(self) -> Result[LaunchSubstitution]:
        return self.argument1

    @property
    def b(self) -> Result[LaunchSubstitution]:
        return self.argument2

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        a = substitute(self.argument1, ctx, source=source)
        b = substitute(self.argument2, ctx, source=source)
        if not a.is_resolved:
            return Result.unknown_value(type=TYPE_TOKEN_STRING, source=a.source)
        if not b.is_resolved:
            return Result.unknown_value(type=TYPE_TOKEN_STRING, source=b.source)
        if a.value != b.value:  # FIXME
            return Resolved.from_string('true', source=source)
        return Resolved.from_string('false', source=source)

    def __str__(self) -> str:
        return f'$(neq {self.argument1} {self.argument2})'


###############################################################################
# ROS Launch Conditions
###############################################################################


@frozen
class LaunchCondition:
    pass

    @property
    def is_if_condition(self) -> bool:
        return False

    @property
    def is_unless_condition(self) -> bool:
        return False


@frozen
class IfCondition(LaunchCondition):
    expression: Result[LaunchSubstitution]

    @property
    def is_if_condition(self) -> bool:
        return True

    def __str__(self) -> str:
        return f'{self.__class__.__name__}({self.expression})'


@frozen
class UnlessCondition(LaunchCondition):
    expression: Result[LaunchSubstitution]

    @property
    def is_unless_condition(self) -> bool:
        return True

    def __str__(self) -> str:
        return f'{self.__class__.__name__}({self.expression})'


###############################################################################
# ROS Launch Entities
###############################################################################


@frozen
class LaunchEntity:
    @property
    def is_argument(self) -> bool:
        return False

    @property
    def is_inclusion(self) -> bool:
        return False

    @property
    def is_node(self) -> bool:
        return False

    @property
    def is_group(self) -> bool:
        return False


@frozen
class LaunchArgument(LaunchEntity):
    name: str
    default_value: Optional[Result[LaunchSubstitution]] = None
    description: Optional[Result[LaunchSubstitution]] = None
    condition: Optional[Result[LaunchCondition]] = None

    @property
    def is_argument(self) -> bool:
        return True

    def to_xml(self) -> str:
        parts = [f'name="{self.name}"']
        if self.default_value is not None:
            parts.append(f'default="{self.default_value}"')
        if self.description is not None:
            parts.append(f'description="{self.description}"')
        return f'<arg {" ".join(parts)} />'


LaunchArgumentKeyValuePair = Result[Tuple[Result[LaunchSubstitution], Result[LaunchSubstitution]]]


@frozen
class LaunchInclusion(LaunchEntity):
    file: Result[LaunchSubstitution]
    namespace: Optional[Result[LaunchSubstitution]] = None
    arguments: Iterable[LaunchArgumentKeyValuePair] = field(factory=list)
    condition: Optional[Result[LaunchCondition]] = None

    @property
    def is_inclusion(self) -> bool:
        return True


LaunchNodeParameterDict = Result[Dict[Result[LaunchSubstitution], Result[LaunchSubstitution]]]
LaunchNodeParameterItem = Union[Result[LaunchSubstitution], LaunchNodeParameterDict]
LaunchNodeParameterList = Result[List[LaunchNodeParameterItem]]


def unknown_parameter_list(source: Optional[TrackedCode] = None) -> LaunchNodeParameterList:
    return UnresolvedIterable.unknown_list(source=source)


def empty_parameter_list(source: Optional[TrackedCode] = None) -> LaunchNodeParameterList:
    return Resolved.from_list([], source=source)


LaunchNodeRemapName = Result[Union[str, LaunchSubstitution]]
LaunchNodeRemapItem = Result[Tuple[LaunchNodeRemapName, LaunchNodeRemapName]]
LaunchNodeRemapList = Result[List[LaunchNodeRemapItem]]


def unknown_remap_list(source: Optional[TrackedCode] = None) -> LaunchNodeRemapList:
    return UnresolvedIterable.unknown_list(source=source)


def empty_remap_list(source: Optional[TrackedCode] = None) -> LaunchNodeRemapList:
    return Resolved.from_list([], source=source)


@frozen
class LaunchNode(LaunchEntity):
    """
    From `launch_ros.actions.Node` documentation:

    «If the name is not given (or is None) then no name is passed to
    the node on creation. The default name specified within the
    code of the node is used instead.»

    «The namespace can either be absolute (i.e. starts with /) or relative.
    If absolute, then nothing else is considered and this is passed
    directly to the node to set the namespace.
    If relative, the namespace in the 'ros_namespace' LaunchConfiguration
    will be prepended to the given relative node namespace.
    If no namespace is given, then the default namespace `/` is assumed.»

    «The parameters are passed as a list, with each element either a yaml
    file that contains parameter rules (string or pathlib.Path to the full
    path of the file), or a dictionary that specifies parameter rules.
    Keys of the dictionary can be strings or an iterable of Substitutions
    that will be expanded to a string.
    Values in the dictionary can be strings, integers, floats, or tuples
    of Substitutions that will be expanded to a string.
    Additionally, values in the dictionary can be lists of the
    aforementioned types, or another dictionary with the same properties.
    A yaml file with the resulting parameters from the dictionary will be
    written to a temporary file, the path to which will be passed to the node.»

    «Multiple parameter dictionaries/files can be passed: each file path
    will be passed in, in order, to the node (where the last definition of
    a parameter takes effect). However, fully qualified node names override
    wildcards even when specified earlier.
    If `namespace` is not specified, dictionaries are prefixed by a
    wildcard namespace (`/**`) and other specific parameter declarations
    may overwrite it.»

    «Using `ros_arguments` is equivalent to using `arguments` with a
    prepended '--ros-args' item.»

    «`remappings` is an ordered list of ('from', 'to') string pairs to be
    passed to the node as ROS remapping rules.»
    """

    package: Result[LaunchSubstitution]
    executable: Result[LaunchSubstitution]
    name: Optional[Result[LaunchSubstitution]] = None
    namespace: Optional[Result[LaunchSubstitution]] = None
    parameters: LaunchNodeParameterList = field(factory=empty_parameter_list)
    remaps: LaunchNodeRemapList = field(factory=empty_remap_list)
    output: Result[LaunchSubstitution] = const_text('log')
    arguments: Iterable[Result[LaunchSubstitution]] = field(factory=list)
    condition: Optional[Result[LaunchCondition]] = None

    @property
    def is_node(self) -> bool:
        return True


@frozen
class LaunchGroupAction(LaunchEntity):
    entities: Result[Iterable[Result[LaunchEntity]]]
    condition: Optional[Result[LaunchCondition]] = None

    @property
    def is_group(self) -> bool:
        return True


@frozen
class LaunchDescription:
    entities: Result[Tuple[Result[LaunchEntity]]]


###############################################################################
# ROS Launch Runtime
###############################################################################


FeatureId = NewType('FeatureId', str)


# @frozen
# class LaunchFeatureMetadata:
#     name: FeatureId
#     dependencies: Set[FeatureId] = field(factory=set)


@frozen
class LaunchFeature:
    # meta: LaunchFeatureMetadata
    id: FeatureId

    @property
    def is_argument(self) -> bool:
        return False

    @property
    def is_node(self) -> bool:
        return False

    @property
    def is_launch_file(self) -> bool:
        return False


@unique
class LaunchArgumentValueType(Enum):
    STRING = 'string'
    INT = 'int'
    FLOAT = 'float'
    PATH = 'path'
    BOOL = 'bool'


@frozen
class ArgumentFeature(LaunchFeature):
    name: str
    default_value: Optional[Result[str]] = None
    description: Optional[Result[str]] = None
    # known_possible_values: List[Result[str]] = field(factory=list)
    known_possible_values: List[str] = field(factory=list)
    inferred_type: LaunchArgumentValueType = LaunchArgumentValueType.STRING
    # affects_cg: bool = False
    # decision_points: int = 0

    @property
    def is_argument(self) -> bool:
        return True


@frozen
class NodeFeature(LaunchFeature):
    rosnode: RosNodeModel
    condition: LogicValue = field(default=TRUE)
    # argument_dependencies: Set[FeatureId] = field(factory=set)

    @property
    def is_node(self) -> bool:
        return True

    @property
    def rosname(self) -> Result[RosName]:
        return self.rosnode.rosname

    @property
    def package(self) -> Result[str]:
        return self.rosnode.package

    @property
    def executable(self) -> Result[str]:
        return self.rosnode.executable

    @property
    def arguments(self) -> Result[List[str]]:
        return self.rosnode.arguments

    @property
    def parameters(self) -> Result:
        return self.rosnode.parameters

    @property
    def remappings(self) -> Result:
        return self.rosnode.remappings

    @property
    def output(self) -> Result[str]:
        return self.rosnode.output

    @property
    def node(self) -> Result[str]:
        return self.rosnode.node


@frozen
class LaunchFileFeature(LaunchFeature):
    file: str
    arguments: Dict[FeatureId, ArgumentFeature] = field(factory=dict)
    nodes: Dict[FeatureId, NodeFeature] = field(factory=dict)
    inclusions: Set[FeatureId] = field(factory=set)
    conflicts: Dict[FeatureId, LogicValue] = field(factory=dict)

    @property
    def is_launch_file(self) -> bool:
        return True

    def features(self) -> Dict[FeatureId, LaunchFeature]:
        features = dict(self.arguments)
        features.update(self.nodes)
        return features


@frozen
class LaunchModel:
    name: str
    files: List[LaunchFileFeature] = field(factory=list)
