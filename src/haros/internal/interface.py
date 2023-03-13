# SPDX-License-Identifier: MIT
# Copyright © 2023 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Union

from errno import EACCES
import logging
import os
from pathlib import Path

from attrs import field, frozen
from yaml import safe_load

from haros.errors import AnalysisError
from haros.metamodel.launch import LaunchDescription
from haros.metamodel.ros import NodeModel, ProjectModel, uid_node

###############################################################################
# Constants
###############################################################################

PathType = Union[str, Path]

PACKAGE_SHARE_DIR = '/usr/share/ros/'

###############################################################################
# Helper Functions
###############################################################################


def uniform_path_string(path: PathType) -> str:
    if isinstance(path, Path):
        return path.as_posix().replace('\\', '/')
    return path.replace('\\', '/')


def fail_to_parse_launch_file(path: PathType) -> LaunchDescription:
    raise AnalysisError('no launch file parser has been provided')


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
    model: Optional[ProjectModel] = None
    launch_cache: Mapping[str, LaunchDescription] = field(factory=dict)
    parse_launch_description: Callable = fail_to_parse_launch_file

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

    def get_node_model(self, package: str, executable: str) -> Optional[NodeModel]:
        # executables = self.executables.get(package)
        # if executables is None or executable not in executables:
        #     return None
        if self.model is None:
            return None
        uid = uid_node(package, executable)
        return self.model.nodes.get(uid)

    def get_launch_description(self, filepath: PathType) -> LaunchDescription:
        filepath = str(filepath)
        if self.strict:
            safe_dir = self._safe_root()
            if safe_dir and not filepath.startswith(safe_dir):
                raise ValueError(filepath)
        description = self.launch_cache.get(filepath)
        if description is None:
            description = self.parse_launch_description(Path(filepath))  # !!
            self.launch_cache[filepath] = description
        return description

    def read_text_file(self, filepath: PathType, encoding: Optional[str] = None) -> str:
        filepath = self._redirect_to_local_packages(filepath)
        if self.strict:
            safe_dir = self._safe_root()
            if safe_dir and not filepath.startswith(safe_dir):
                raise ValueError(filepath)
        return Path(filepath).read_text()

    def read_yaml_file(self, filepath: PathType, encoding: Optional[str] = None) -> Dict[Any, Any]:
        return safe_load(self.read_text_file(filepath, encoding=encoding))

    def execute_command(self, cmd: str) -> str:
        raise EnvironmentError(EACCES, cmd)

    def _redirect_to_local_packages(self, path: PathType) -> str:
        path = uniform_path_string(path)
        if path.startswith(PACKAGE_SHARE_DIR):
            relative = path[len(PACKAGE_SHARE_DIR):]
            parts = relative.split('/', maxsplit=1)
            package = parts[0]
            suffix = relative[len(package):]
            new_path = self.packages.get(package, f'{PACKAGE_SHARE_DIR}{package}')
            if new_path is not None:
                return f'{new_path}{suffix}'
        return path

    def _safe_root(self) -> Optional[str]:
        if self.workspace is not None:
            return self.workspace
        path: Optional[str] = self.get_environment_var('ROS_ROOT')
        if path is not None:
            return str(Path(path).parent)
        return None
