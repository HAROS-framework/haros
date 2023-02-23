# SPDX-License-Identifier: MIT
# Copyright © 2023 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Iterable, Mapping, Optional, Union

from errno import EACCES
import logging
import os
from pathlib import Path

from attrs import field, frozen

from haros.errors import AnalysisError
from haros.metamodel.launch import LaunchDescription

###############################################################################
# Constants
###############################################################################

PathType = Union[str, Path]

###############################################################################
# Interface
###############################################################################


@frozen
class AnalysisSystemInterface:
    strict: bool = False
    environment: Mapping[str, str] = field(factory=dict)
    workspace: Optional[str] = None
    packages: Mapping[str, str] = field(factory=dict)
    executables: Mapping[str, Iterable[str]] = field(factory=dict)
    launch_cache: Mapping[str, LaunchDescription] = field(factory=dict)

    def get_environment_var(self, name: str) -> Optional[str]:
        value: Optional[str] = self.environment.get(name)
        return value if value is not None else os.environ.get(name)

    def get_package_path(self, name: str) -> Optional[str]:
        strpath: Optional[str] = self.packages.get(name)
        if strpath:
            return strpath
        d: Optional[Path] = None
        strpath = self.workspace
        if strpath:
            d = Path(strpath) / 'src' / name
        else:
            strpath = self.get_environment_var('ROS_ROOT')
            if strpath:
                d = Path(strpath).parent / name
        if d is not None and d.is_dir():
            p: Path = d / 'package.xml'
            if p.is_file():
                self.pkg_paths[name] = str(d)
                return str(d)
        return None

    def get_launch_description(self, path: PathType) -> LaunchDescription:
        filepath = str(filepath)
        if self.strict:
            safe_dir = self._safe_root()
            if safe_dir and not filepath.startswith(safe_dir):
                raise ValueError(filepath)
        description = self.launch_cache.get(filepath)
        if description is None:
            # FIXME
            # description = parse_from_file(filepath)  # !!
            # self.launch_cache[filepath] = description
            raise AnalysisError(f'unable to parse {path}')
        return description

    def read_text_file(self, filepath: PathType) -> str:
        filepath = str(filepath)
        if self.strict:
            safe_dir = self._safe_root()
            if safe_dir and not filepath.startswith(safe_dir):
                raise ValueError(filepath)
        return Path(filepath).read_text()

    def execute_command(self, cmd: str) -> str:
        raise EnvironmentError(EACCES, cmd)

    def _safe_root(self) -> Optional[str]:
        if self.workspace is not None:
            return self.workspace
        path: Optional[str] = self.get_environment_var('ROS_ROOT')
        if path is not None:
            return str(Path(path).parent)
        return None
