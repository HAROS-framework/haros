# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from types import ModuleType
from typing import (
    Any,
    Final,
    Iterator,
    NewType,
    Optional,
    Tuple,
    Union,
)

from collections.abc import Iterable, Mapping, Sequence, Set
from enum import Enum, unique
import importlib
import logging
from pathlib import Path
import tempfile

from attrs import asdict, define, field, frozen
import yaml

from haros.metamodel.common import TrackedCode
from haros.metamodel.logic import TRUE, LogicValue
from haros.metamodel.result import IterableType, Result, UnresolvedString
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
        return self.set(name, Result.of_string(text, source=source))

    def resolve_condition(self, condition: Optional[Result['LaunchCondition']]) -> Result[bool]:
        logger.debug(f'{self.__class__.__name__}.resolve_condition({condition!r})')
        raise NotImplementedError()

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

    def read_text_file(self, path: str) -> str:
        logger.debug(f'{self.__class__.__name__}.read_text_file({path!r})')
        raise NotImplementedError()

    def read_yaml_file(self, path: str) -> dict[Any, Any]:
        logger.debug(f'{self.__class__.__name__}.read_yaml_file({path!r})')
        raise NotImplementedError()


# For a full list of substitutions see
# https://github.com/ros2/launch/tree/galactic/launch/launch/substitutions


@frozen
class LaunchSubstitution:
    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        logger.debug(f'{self.__class__.__name__}.resolve({ctx!r}, {source!r})')
        raise NotImplementedError()

    def __str__(self) -> str:
        return '$(?)'


def unknown_substitution(source: Optional[TrackedCode] = None) -> Result[LaunchSubstitution]:
    return Result.of_instance(LaunchSubstitution, source=source)


def const_substitution(
    sub: LaunchSubstitution,
    source: Optional[TrackedCode] = None,
) -> Result[LaunchSubstitution]:
    return Result.of(sub, source=source)


def substitute(
    substitution: Optional[Result[Union[LaunchSubstitution, Sequence[Result[LaunchSubstitution]]]]],
    context: LaunchScopeContext,
    source: Optional[TrackedCode] = None,
) -> Result[str]:
    if substitution is None:
        return Result.of_string(source=source)
    source = source or substitution.source
    if substitution.is_resolved:
        if isinstance(substitution.value, LaunchSubstitution):
            return substitution.value.resolve(context, source=source)
        if substitution.type.is_iterable:
            parts: list[str] = []
            for sub in substitution.value:
                part = sub.value.resolve(context, source=source)
                if not part.is_resolved:
                    return Result.of_string(source=source)
                parts.append(part.value)
            return Result.of_string(''.join(parts))
    return Result.of_string(source=source)


def substitute_optional(
    substitution: Optional[Result[LaunchSubstitution]],
    context: LaunchScopeContext,
    source: Optional[TrackedCode] = None,
) -> Result[str]:
    if substitution is None:
        return None
    if substitution.is_resolved:
        return substitution.value.resolve(context, source=(source or substitution.source))
    return Result.of_string(source=substitution.source)


def _to_sub(arg: Result[Union[None, str, LaunchSubstitution]]) -> Result[LaunchSubstitution]:
    if not arg.is_resolved:
        return Result.unknown_value(source=arg.source)
    if arg.value is None:
        return Result.of(TextSubstitution(''))
    if isinstance(arg.value, LaunchSubstitution):
        return arg
    if isinstance(arg.value, str):
        return Result.of(TextSubstitution(arg.value), source=arg.source)
    return Result.unknown_value(source=arg.source)


def _to_sub_list(
    arg: Result[
        Union[None, str, LaunchSubstitution, Iterable[Result[Union[None, str, LaunchSubstitution]]]]
    ],
) -> Result[list[Result[LaunchSubstitution]]]:
    if not arg.is_resolved:
        return Result.of_list(source=arg.source)
    if arg.value is None:
        arg = Result.of(TextSubstitution(''), source=arg.source)
    elif isinstance(arg.value, str):
        arg = Result.of(TextSubstitution(arg.value), source=arg.source)
    elif isinstance(arg.value, IterableType):
        values = [_to_sub(item) for item in arg.value]
        return Result.of_list(values, source=arg.source)
    if isinstance(arg.value, LaunchSubstitution):
        return Result.of_list([arg], source=arg.source)
    return Result.of_list(source=arg.source)


def _to_sub_list_or_none(
    arg: Optional[
        Result[
            Union[
                None,
                str,
                LaunchSubstitution,
                Iterable[Result[Union[None, str, LaunchSubstitution]]],
            ]
        ]
    ],
) -> Optional[Result[list[Result[LaunchSubstitution]]]]:
    if arg is None:
        return None
    return _to_sub_list(arg)


@frozen
class TextSubstitution(LaunchSubstitution):
    value: str

    def resolve(
        self,
        _ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return Result.of_string(self.value, source=source)

    def __str__(self) -> str:
        return self.value


def const_text(text: str, source: Optional[TrackedCode] = None) -> Result[LaunchSubstitution]:
    return const_substitution(TextSubstitution(text), source=source)


@frozen
class LaunchConfiguration(LaunchSubstitution):
    name: str
    default_value: Optional[Result[LaunchSubstitution]] = None

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
    """Gets the value of a launch description argument, as a string, by name."""

    name: str

    def resolve(
        self,
        ctx: LaunchScopeContext,
        _source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return ctx.get_arg(self.name)

    def __str__(self) -> str:
        return f'$(arg {self.name})'


def _default_python_modules() -> Result[Iterable[Result[LaunchSubstitution]]]:
    math = Result.of(TextSubstitution('math'))
    return Result.of_tuple((math,))


@frozen
class PythonExpressionSubstitution(LaunchSubstitution):
    """
    Substitution that can access contextual local variables.

    The expression may contain Substitutions, but must return something that can
    be converted to a string with `str()`.
    It also may contain math symbols and functions.
    """

    # https://github.com/ros2/launch/blob/rolling/launch/launch/substitutions/python_expression.py
    expression: Result[Sequence[Result[LaunchSubstitution]]] = field(converter=_to_sub_list)
    modules: Result[Sequence[Result[LaunchSubstitution]]] = field(
        converter=_to_sub_list,
        factory=_default_python_modules,
    )

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        if not self.expression.is_resolved or not self.modules.is_resolved:
            return Result.of_string(source=source)
        known_parts: list[str] = []
        all_parts: list[Result[str]] = []
        for part in self.expression.value:
            value = substitute(part, ctx, source=source)
            all_parts.append(value)
            if value.is_resolved:
                known_parts.append(value.value)
        if len(known_parts) != len(all_parts):
            return Result.of_string(UnresolvedString(all_parts), source=self.expression.source)
        expression = ''.join(known_parts)
        # FIXME avoid eval if possible
        expression_locals: dict[str, Any] = {}
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
            return Result.of_string(end_result, source=source)
        except Exception:
            pass
        return Result.of_string(source=source)

    def __str__(self) -> str:
        return f'$(python {self.expression})'


@frozen
class EnvironmentSubstitution(LaunchSubstitution):
    """Gets an environment variable value, as a string, by name."""

    name: str

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
    """Locates the full path to an executable on the PATH if it exists."""

    name: str

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

    This is a mechanism that allows a "parent" action to pass information to sub actions.
    """

    expression: str

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
    - 'fail': raises `SubstitutionFailure` when stderr output is detected.
    - 'ignore': `stderr` output is ignored.
    - 'warn': The `stderr` output is ignored, but a warning is logged if detected.
    - 'capture': The `stderr` output will be captured, together with stdout.
    It can also be a substitution, that results in one of those four options.
    """
    # on_stderr: LaunchSubstitution  # TODO

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

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        value = substitute(self.name, ctx, source=source)
        if value.is_resolved:
            name = ctx.compute_anon_name(value.value)
            value = Result.of_string(name, source=source)
        return value

    def __str__(self) -> str:
        return f'$(anon {self.name})'


@frozen
class ThisLaunchFileSubstitution(LaunchSubstitution):
    """Substitution that returns the absolute path to the current launch file."""

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return Result.of_string(ctx.get_this_launch_file(), source=source)

    def __str__(self) -> str:
        return '$(this-launch-file)'


@frozen
class ThisDirectorySubstitution(LaunchSubstitution):
    """Returns the absolute path to the current launch file's containing directory."""

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        return Result.of_string(ctx.get_this_launch_file_dir(), source=source)

    def __str__(self) -> str:
        return '$(this-launch-file-dir)'


@frozen
class ConcatenationSubstitution(LaunchSubstitution):
    parts: Iterable[Result[LaunchSubstitution]]

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        known_parts: list[str] = []
        all_parts: list[Result[str]] = []
        for part in self.parts:
            value = substitute(part, ctx, source=source)
            all_parts.append(value)
            if value.is_resolved:
                known_parts.append(value.value)
        if len(known_parts) != len(all_parts):
            return Result.of_string(value=UnresolvedString(all_parts), source=source)
        return Result.of_string(value=''.join(known_parts), source=source)

    def __str__(self) -> str:
        return ''.join(map(str, self.parts))


@frozen
class PathJoinSubstitution(LaunchSubstitution):
    parts: Iterable[Result[LaunchSubstitution]]

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        path = Path()
        is_unknown: bool = False
        all_parts: list[Result[str]] = []
        for part in self.parts:
            value = substitute(part, ctx, source=source)
            all_parts.append(value)
            if value.is_resolved:
                path = path / str(value)
            else:
                is_unknown = True
        if is_unknown:
            return Result.of_string(value=UnresolvedString(all_parts), source=source)
        return Result.of_string(value=path.as_posix(), source=source)

    def __str__(self) -> str:
        return f'$(join {" ".join(map(str, self.parts))})'


@frozen
class EqualsSubstitution(LaunchSubstitution):
    # https://github.com/ros2/launch/blob/rolling/launch/launch/substitutions/equals_substitution.py
    argument1: Result[LaunchSubstitution] = field(converter=_to_sub)
    argument2: Result[LaunchSubstitution] = field(converter=_to_sub)

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
            return Result.of_string(source=a.source)
        if not b.is_resolved:
            return Result.of_string(source=b.source)
        if a.value == b.value:  # FIXME
            return Result.of_string('true', source=source)
        return Result.of_string('false', source=source)

    def __str__(self) -> str:
        return f'$(eq {self.argument1} {self.argument2})'


@frozen
class NotEqualsSubstitution(LaunchSubstitution):
    # https://github.com/ros2/launch/blob/rolling/launch/launch/substitutions/not_equals_substitution.py
    argument1: Result[LaunchSubstitution] = field(converter=_to_sub)
    argument2: Result[LaunchSubstitution] = field(converter=_to_sub)

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
            return Result.of_string(source=a.source)
        if not b.is_resolved:
            return Result.of_string(source=b.source)
        if a.value != b.value:  # FIXME
            return Result.of_string('true', source=source)
        return Result.of_string('false', source=source)

    def __str__(self) -> str:
        return f'$(neq {self.argument1} {self.argument2})'


###############################################################################
# ROS Launch Conditions
###############################################################################


@frozen
class LaunchCondition:
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
# Third-Party ROS Launch Substitutions
###############################################################################


def _values_to_sub_list(
    d: Result[Mapping[Result[str], Result[Union[None, str, LaunchSubstitution]]]],
) -> Result[dict[str, Result[Sequence[Result[LaunchSubstitution]]]]]:
    if not d.is_resolved:
        return Result.of_dict(source=d.source)
    o: dict[str, Result[Sequence[Result[LaunchSubstitution]]]] = {}
    for key, value in d.value.items():
        if not key.is_resolved:
            return Result.of_dict(source=d.source)
        o[key.value] = _to_sub_list(value)
    return Result.of_dict(o)


@frozen
class ReplaceStringSubstitution(LaunchSubstitution):
    # nav2_common/launch/replace_string.py
    source_file: Result[Sequence[Result[LaunchSubstitution]]] = field(converter=_to_sub_list)
    replacements: Result[Mapping[str, Result[Sequence[Result[LaunchSubstitution]]]]] = field(
        converter=_values_to_sub_list
    )
    condition: Optional[Result[LaunchCondition]] = None

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        if not self.replacements.is_resolved:
            return Result.of_string(source=source)
        yaml_filename: Result[str] = substitute(self.source_file, ctx, source=source)
        if not yaml_filename.is_resolved:
            return yaml_filename

        phi: Result[bool] = Result.of_bool(source=source)
        if self.condition is None:
            phi = Result.of_bool(True, source=source)
        elif self.condition.is_resolved:
            phi = ctx.resolve_condition(self.condition)
        if not phi.is_resolved:
            return Result.of_string(source=source)

        if phi.value:
            replacements = self._resolve_replacements(ctx, source)
            if not replacements.is_resolved:
                return Result.of_string(source=source)
            output_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
            try:
                input_file = open(yaml_filename, 'r')
                self._replace(input_file, output_file, replacements)
            finally:
                input_file.close()
                output_file.close()
            return Result.of_string(output_file.name, source=source)
        else:
            return yaml_filename

    def _resolve_replacements(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode],
    ) -> Result[dict[str, str]]:
        assert self.replacements.is_resolved
        resolved_replacements: dict[str, Result[str]] = {}
        for key, replacement in self.replacements.value.items():
            value = substitute(replacement, ctx, source=source)
            if not value.is_resolved:
                return Result.of_dict(source=source)
            resolved_replacements[key] = value
        return Result.of_dict(resolved_replacements)

    def _replace(self, input_file, output_file, replacements: dict[str, str]):
        for line in input_file:
            for key, value in replacements.items():
                if isinstance(key, str) and isinstance(value, str):
                    if key in line:
                        line = line.replace(key, value)
                else:
                    raise TypeError(
                        'A provided replacement pair is not a string. '
                        'Both key and value should be strings.'
                    )
            output_file.write(line)

    def __str__(self) -> str:
        return (
            '$(nav2_common/replace-string '
            f'{self.source_file} {self.replacements} {self.condition})'
        )


@frozen
class RewrittenYamlSubstitution(LaunchSubstitution):
    # nav2_common/launch/rewritten_yaml.py
    source_file: Result[Sequence[Result[LaunchSubstitution]]] = field(converter=_to_sub_list)
    param_rewrites: Result[Mapping[str, Result[Sequence[Result[LaunchSubstitution]]]]] = field(
        converter=_values_to_sub_list,
    )
    root_key: Optional[Result[Sequence[Result[LaunchSubstitution]]]] = field(
        default=None,
        converter=_to_sub_list_or_none,
    )
    key_rewrites: Result[Mapping[str, Result[Sequence[Result[LaunchSubstitution]]]]] = field(
        default=None,
        converter=lambda d: Result.of_dict({}) if d is None else _values_to_sub_list(d),
    )
    convert_types: Result[bool] = field(factory=lambda: Result.of_bool(False))

    def resolve(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[str]:
        if not self.param_rewrites.is_resolved:
            return Result.of_string(source=source)
        if not self.key_rewrites.is_resolved:
            return Result.of_string(source=source)

        yaml_filename: Result[str] = substitute(self.source_file, ctx, source=source)
        if not yaml_filename.is_resolved:
            return yaml_filename

        param_rewrites, keys_rewrites = self._resolve_rewrites(ctx)
        yaml_data = ctx.read_yaml_file(yaml_filename.value)
        data: Result[dict[Any, Any]] = self._substitute_params(yaml_data, param_rewrites)
        if not data.is_resolved:
            return yaml_filename

        data = self._add_params(data.value, param_rewrites)
        if not data.is_resolved:
            return yaml_filename

        data = self._substitute_keys(data.value, keys_rewrites)
        if not data.is_resolved:
            return yaml_filename

        if self.root_key is not None:
            root_key = substitute(self.root_key, ctx)
            if not root_key.is_resolved:
                return yaml_filename
            if root_key.value:
                data = Result.of_dict({root_key.value: data.value})

        assert data.is_resolved
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as rewritten_yaml:
            yaml.dump(data.value, rewritten_yaml)
        return Result.of_string(rewritten_yaml.name)

    def _resolve_rewrites(
        self, context: LaunchScopeContext
    ) -> Tuple[dict[str, Result[str]], dict[str, Result[str]]]:
        resolved_params: dict[str, Result[str]] = {}
        for key, value in self.param_rewrites.value.items():
            resolved_params[key] = substitute(value, context)
        resolved_keys: dict[str, Result[str]] = {}
        for key, value in self.key_rewrites.value.items():
            resolved_keys[key] = substitute(value, context)
        return resolved_params, resolved_keys

    def _substitute_params(
        self, data: dict[Any, Any], param_rewrites: dict[str, Result[str]]
    ) -> Result[dict[Any, Any]]:
        # substitute leaf-only parameters
        for key in self._get_leaf_keys(data):
            raw_value = param_rewrites.get(key)
            if raw_value is not None:
                if not raw_value.is_resolved:
                    return Result.of_dict(source=raw_value.source)
                if not self.convert_types.is_resolved:
                    return Result.of_dict(source=self.convert_types.source)
                data[key] = self._convert(raw_value.value)
        # substitute total path parameters
        yaml_paths = self._pathify(Result.of_dict(data), '', {})
        for path in yaml_paths:
            if path in param_rewrites:
                # this is an absolute path (ex. 'key.keyA.keyB.val')
                value = param_rewrites[path]
                if not value.is_resolved:
                    return Result.of_dict(source=value.source)
                rewrite_val = self._convert(value.value)
                yaml_keys = path.split('.')
                data = self._update_yaml_path_vals(data, yaml_keys, rewrite_val)
        return Result.of_dict(data)

    def _get_leaf_keys(self, data: dict[Any, Any]) -> Iterator[str]:
        for key, value in data.items():
            if isinstance(value, dict):
                yield from self._get_leaf_keys(value)
            yield key

    def _convert(self, text_value: str) -> Union[bool, int, float, str]:
        if self.convert_types.value:
            # try converting to int or float
            try:
                if '.' in text_value:
                    return float(text_value)
                return int(text_value)
            except ValueError:
                pass
        # try converting to bool
        if text_value.lower() == 'true':
            return True
        if text_value.lower() == 'false':
            return False
        # nothing else worked so fall through and return text
        return text_value

    def _pathify(self, data: Any, p: str, paths: dict[str, Any], joinchar='.') -> dict[str, Any]:
        pn = p if not p else p + joinchar
        if isinstance(data, dict):
            for k, v in data.items():
                self._pathify(v, f'{pn}{k}', paths, joinchar=joinchar)
        elif isinstance(data, list):
            for i, e in enumerate(data):
                self._pathify(e, f'{pn}{i}', paths, joinchar=joinchar)
        else:
            paths[p] = data
        return paths

    def _update_yaml_path_vals(
        self,
        data: dict[Any, Any],
        yaml_key_list: list[str],
        rewrite_val: Union[bool, int, float, str],
    ) -> dict[Any, Any]:
        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                data[key] = rewrite_val
                break
            key = yaml_key_list.pop(0)
            if isinstance(data, list):
                data[int(key)] = self._update_yaml_path_vals(
                    data[int(key)], yaml_key_list, rewrite_val
                )
            else:
                data[key] = self._update_yaml_path_vals(
                    data.get(key, {}), yaml_key_list, rewrite_val
                )
        return data

    def _add_params(
        self, data: Mapping[Any, Any], param_rewrites: Mapping[str, Result[str]]
    ) -> Result[Mapping[Any, Any]]:
        # add new total path parameters
        yaml_paths: dict[str, Any] = self._pathify(data, '', {})
        for path in param_rewrites:
            if path not in yaml_paths:
                value = param_rewrites[path]
                if not value.is_resolved:
                    return Result.of_dict(source=value.source)
                new_val = self._convert(value.value)
                yaml_keys = path.split('.')
                if 'ros__parameters' in yaml_keys:
                    data = self._update_yaml_path_vals(data, yaml_keys, new_val)
        return Result.of_dict(data)

    def _substitute_keys(
        self, data: Mapping[Any, Any], key_rewrites: Mapping[str, Result[str]]
    ) -> Result[Mapping[Any, Any]]:
        if len(key_rewrites) != 0:
            for key, val in list(data.items()):
                final_key = key
                if key in key_rewrites:
                    new_key = key_rewrites[key]
                    if not new_key.is_resolved:
                        return Result.of_dict(source=new_key.source)
                    data[new_key.value] = val
                    del data[key]
                    final_key = new_key.value
                if isinstance(val, dict):
                    new_val = self._substitute_keys(val, key_rewrites)
                    if not new_val.is_resolved:
                        return Result.of_dict(source=new_val.source)
                    data[final_key] = new_val.value
        return Result.of_dict(data)

    def __str__(self) -> str:
        return (
            '$(nav2_common/rewritten-yaml '
            f'{self.source_file} {self.param_rewrites} {self.key_rewrites} '
            f'{self.root_key} {self.convert_types})'
        )


###############################################################################
# ROS Launch Description Entities
###############################################################################


@define
class ParameterFileDescription:
    filepath: Result[LaunchSubstitution]
    allow_subs: Result[Union[bool, LaunchSubstitution]]
    __cached_result: Optional[Result[Mapping[Result[str], Result[Any]]]] = None

    @classmethod
    def factory(
        cls,
        filepath: Result[Union[str, LaunchSubstitution]],
        allow_substs: Optional[Result[Union[bool, LaunchSubstitution]]] = None,
    ) -> 'ParameterFileDescription':
        if filepath.is_resolved:
            if not isinstance(filepath.value, LaunchSubstitution):
                filepath = Result.of(TextSubstitution(filepath.value), source=filepath.source)
        if allow_substs is None:
            allow_substs = Result.of_bool(False)
        return cls(filepath, allow_substs)

    def evaluate(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[dict[Result[str], Result[Any]]]:
        if self.__cached_result is None:
            self.__cached_result = self._evaluate(ctx, source=source)
        return self.__cached_result

    def _evaluate(
        self,
        ctx: LaunchScopeContext,
        source: Optional[TrackedCode] = None,
    ) -> Result[dict[Result[str], Result[Any]]]:
        if not self.filepath.is_resolved:
            return Result.of_dict(source=source)
        if not self.allow_subs.is_resolved:
            return Result.of_dict(source=source)
        filepath = substitute(self.filepath, ctx, source=source)
        if not filepath.is_resolved:
            return Result.of_dict(source=source)
        if self.allow_subs.type.is_bool:
            allow_subs: Result[bool] = self.allow_subs
        else:
            r = substitute(self.allow_subs, ctx, source=source)
            if not r.is_resolved:
                return Result.of_dict(source=source)
            if r.value.lower() == 'true':
                allow_subs = Result.of_bool(True, source=self.allow_subs.source)
            elif r.value.lower == 'false':
                allow_subs = Result.of_bool(False, source=self.allow_subs.source)
            else:
                allow_subs = Result.of_bool(source=self.allow_subs.source)
        if not allow_subs.is_resolved:
            return Result.of_dict(source=source)

        if allow_subs.value:  # FIXME
            # try:
            #     text = ctx.read_text_file(filepath.value)
            # except (FileNotFoundError, IOError):
            #     return Result.of_dict(source=source)
            # https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/parameter_descriptions.py
            logger.debug(f'parse and apply substitutions on {filepath.value}')

        try:
            raw_data = ctx.read_yaml_file(filepath.value)
        except IOError:
            return Result.of_dict(source=source)

        data: dict[Result[str], Result[Any]] = {}
        for key, value in raw_data.items():
            data[Result.of_string(key)] = Result.of(value)
        return Result.of_dict(value=data, source=source)

    def __str__(self) -> str:
        return f'$(parameter-file {self.filepath} {self.allow_subs})'


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

    @property
    def is_set_environment(self) -> bool:
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


type LaunchNodeParameterDict = Result[
    Mapping[Result[LaunchSubstitution], Result[LaunchSubstitution]]
]
type LaunchNodeParameterItem = Union[Result[LaunchSubstitution], LaunchNodeParameterDict]
type LaunchNodeParameterList = Result[Sequence[LaunchNodeParameterItem]]


def unknown_parameter_list(source: Optional[TrackedCode] = None) -> LaunchNodeParameterList:
    return Result.of_list(source=source)


def empty_parameter_list(source: Optional[TrackedCode] = None) -> LaunchNodeParameterList:
    return Result.of_list([], source=source)


type LaunchNodeRemapName = Result[Union[str, LaunchSubstitution]]
type LaunchNodeRemapItem = Result[Tuple[LaunchNodeRemapName, LaunchNodeRemapName]]
type LaunchNodeRemapList = Result[Sequence[LaunchNodeRemapItem]]


def unknown_remap_list(source: Optional[TrackedCode] = None) -> LaunchNodeRemapList:
    return Result.of_list(source=source)


def empty_remap_list(source: Optional[TrackedCode] = None) -> LaunchNodeRemapList:
    return Result.of_list([], source=source)


@frozen
class LaunchNode(LaunchEntity):
    """
    From `launch_ros.actions.Node` documentation.

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
class LaunchSetEnvironment(LaunchEntity):
    key: Result[LaunchSubstitution]
    value: Result[LaunchSubstitution]

    @property
    def is_set_environment(self) -> bool:
        return True


@frozen
class LaunchDescription:
    entities: Result[Tuple[Result[LaunchEntity]]]

    def serialize(self) -> dict[str, Any]:
        return asdict(self)


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
    # known_possible_values: Sequence[Result[str]] = field(factory=list)
    known_possible_values: Sequence[str] = field(factory=list)
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
    def arguments(self) -> Result[list[str]]:
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
    arguments: Mapping[FeatureId, ArgumentFeature] = field(factory=dict)
    nodes: Mapping[FeatureId, NodeFeature] = field(factory=dict)
    inclusions: Set[FeatureId] = field(factory=set)
    conflicts: Mapping[FeatureId, LogicValue] = field(factory=dict)

    @property
    def is_launch_file(self) -> bool:
        return True

    def features(self) -> dict[FeatureId, LaunchFeature]:
        features = dict(self.arguments)
        features.update(self.nodes)
        return features


@frozen
class LaunchModel:
    name: str
    files: Sequence[LaunchFileFeature] = field(factory=list)
