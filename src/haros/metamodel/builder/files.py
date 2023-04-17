# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final

import logging
from pathlib import Path

from haros.metamodel.ros import FileModel, Languages

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Interface
###############################################################################


def build_from_package(package: str, filepath: str, prefix: Path) -> FileModel:
    path = prefix / filepath
    try:
        path = path.resolve(strict=True)
    except FileNotFoundError:
        raise ValueError('not a file: ' + str(path))
    if not path.is_file():
        raise ValueError('not a file: ' + str(path))
    file = FileModel(package, filepath)  # raise ValueError
    file.source.language = _get_language(path)
    # file.source.lines = ...
    # file.source.ast = ...
    # file.dependencies.build.add(...)
    # file.dependencies.runtime.add(...)
    return file


###############################################################################
# File Languages
###############################################################################


_EXT_CPP = ('.c', '.cpp', '.cxx', '.cc', '.h', '.hpp', '.hxx')


def _get_language(path: Path) -> Languages:
    name = path.name
    ext = path.suffix.lower()
    if name == 'package.xml':
        return Languages.ROSPKG
    if name == 'CMakeLists.txt' or ext == '.cmake':
        return Languages.CMAKE
    if ext == '.yaml' or ext == '.yml':
        return Languages.YAML
    if ext == '.json':
        return Languages.JSON
    if ext == '.xml':
        if path.name.endswith('.launch.xml'):
            return Languages.ROSLAUNCH
        return Languages.XML
    if ext == '.msg':
        return Languages.ROSMSG
    if ext == '.srv':
        return Languages.ROSSRV
    if ext == '.action':
        return Languages.ROSACTION
    if ext == '.py':
        return Languages.PYTHON
    if ext in _EXT_CPP:
        return Languages.CPP
    if not ext:
        # FIXME
        return Languages.BINARY
    return Languages.UNKNOWN
