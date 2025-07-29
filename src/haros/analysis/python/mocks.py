# SPDX-License-Identifier: MIT
# Copyright © 2024 André Santos

###############################################################################
# Imports
###############################################################################

from types import SimpleNamespace
from typing import Any, Callable, Dict, Final, Optional

import logging
import os
from pathlib import Path

from attrs import define, frozen

from haros.analysis.python.dataflow import BUILTINS_MODULE, MockObject, StrictFunctionCaller
from haros.internal.interface import AnalysisSystemInterface, PathType
from haros.metamodel.common import T
from haros.metamodel.result import Result

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Mocks
###############################################################################


@define
class HarosMockObject[T](MockObject):
    def _haros_freeze(self) -> T:
        # use a method name with a low probability of name collision
        # with one of the mocked object's methods or attributes
        raise NotImplementedError()


@frozen
class LazyFileHandle(MockObject):
    path: Result[PathType]
    system: AnalysisSystemInterface

    def read(self, encoding: Optional[Result[str]] = None) -> Result[str]:
        try:
            if self.path.is_resolved:
                encoding = encoding or Result.of_none()
                text = self.system.read_text_file(self.path.value, encoding=encoding.value)
                return Result.of_string(text)
        except ValueError:
            pass
        return Result.of_string()

    def __str__(self) -> str:
        return f'{self.__class__.__name__}(path={self.path})'


@frozen
class BuiltinOpen(MockObject, Callable[[Result[str], Result[str]], Result[LazyFileHandle]]):
    system: AnalysisSystemInterface

    def __call__(
        self,
        path: Result[str],
        mode: Optional[Result[str]] = None,
    ) -> Result[LazyFileHandle]:
        if not path.is_resolved:
            return Result.unknown_value()
        if mode is None:
            mode = Result.of_string('r')
        if not mode.is_resolved or mode.value != 'r':
            return Result.unknown_value()
        return Result.of(LazyFileHandle(path, self.system))


###############################################################################
# Interface
###############################################################################


def standard_symbols(system: AnalysisSystemInterface) -> Dict[str, Any]:
    symbols = {
        f'{BUILTINS_MODULE}.open': BuiltinOpen(system),
    }
    ns = SimpleNamespace()
    ns.path = SimpleNamespace()
    for key in dir(os.path):
        if key.startswith('_'):
            continue
        value = getattr(os.path, key)
        if key == 'join':
            value = StrictFunctionCaller(_os_path_wrapper, name='join', module='os.path')
        setattr(ns.path, key, value)
    ns.environ = dict(system.environment)
    symbols['os'] = ns
    return symbols


def _os_path_wrapper(*args: str) -> str:
    return Path(*args).as_posix()
