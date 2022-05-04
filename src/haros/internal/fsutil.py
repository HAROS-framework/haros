# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List

import os
from pathlib import Path

###############################################################################
# Interface
###############################################################################


def generate_dir(path: Path, structure: Dict[str, Any], overwrite: bool = True):
    """Recursively create a given directory structure."""

    # log.debug(f'generate_dir({path}, {structure})')
    for name, contents in structure.items():
        new_path = (path / name).resolve()
        exists = new_path.exists()
        if isinstance(contents, str):
            if exists and not new_path.is_file():
                raise FileExistsError(str(new_path))
            if overwrite or not exists:
                # log.info(f'Creating {new_path}')
                new_path.write_text(contents, encoding='utf-8')
        elif isinstance(contents, dict):
            if exists and not new_path.is_dir():
                raise FileExistsError(str(new_path))
            if not exists:
                # log.info(f'Creating {new_path}')
                # A missing parent raises FileNotFoundError.
                new_path.mkdir(exist_ok=True)
            generate_dir(new_path, contents, overwrite=overwrite)


def ensure_structure(path: Path, structure: Dict[str, Any]):
    """Ensure that the given path contains the given structure."""

    for name, contents in structure.items():
        new_path = (path / name).resolve()
        if isinstance(contents, str):
            if not new_path.is_file():
                raise FileNotFoundError(str(new_path))
        elif isinstance(contents, dict):
            if not new_path.is_dir():
                raise FileNotFoundError(str(new_path))


def is_workspace(path: Path) -> bool:
    """Check whether the given path is likely to be a ROS workspace."""

    try:
        ws = path.resolve(strict=True)
    except FileNotFoundError:
        return False

    if not ws.is_dir():
        return False

    src = ws / 'src'
    if not src.is_dir():
        # build = ws / 'build'
        return False  # this can be amended, it is only a best practice
    return True


def is_ros_package(path: Path) -> bool:
    """Check whether the given path is likely to be a ROS package."""

    try:
        pkg = path.resolve(strict=True)
    except FileNotFoundError:
        return False

    if not pkg.is_dir():
        return False

    manifest = pkg / 'package.xml'
    return manifest.is_file()


def crawl_workspace(ws: Path, *, relative: bool = False) -> Dict[str, Path]:
    packages = {}
    stack = [ws]
    while stack:
        path = stack.pop()
        try:
            path = path.resolve(strict=True)
        except FileNotFoundError:
            continue
        if not path.is_dir():
            continue
        if path.name.startswith('.'):
            continue
        ignore = path / 'COLCON_IGNORE'
        if ignore.is_file():
            continue
        ignore = path / 'AMENT_IGNORE'
        if ignore.is_file():
            continue
        ignore = path / 'CATKIN_IGNORE'
        if ignore.is_file():
            continue
        ignore = path / 'HAROS_IGNORE'
        if ignore.is_file():
            continue
        manifest = path / 'package.xml'
        if manifest.is_file():
            name = path.name
            if relative:
                packages[name] = path.relative_to(ws)
            else:
                packages[name] = path
        else:
            stack.extend(path.iterdir())
    return packages


def get_package_path(name: str, *, workspaces: Iterable[Path] = None) -> Path:
    """
    Find the file system path for a given ROS package.
    This essentially duplicates the behaviour of ament.
    See https://github.com/ros2/ros2cli/tree/master/ros2pkg for the original.
    """

    # from ament_index_python import get_package_prefix
    # from ament_index_python import get_packages_with_prefixes

    workspaces = workspaces or ()
    for ws in workspaces:
        pass  # TODO

    if not True:
        _get_ament_prefix_path(name)

    return


###############################################################################
# ROS Package Functions
###############################################################################

AMENT_PREFIX_PATH_ENV_VAR: Final[str] = 'AMENT_PREFIX_PATH'
RESOURCE_INDEX_SUBFOLDER: Final[str] = 'share/ament_index/resource_index'


def _get_ament_search_paths() -> List[Path]:
    ament_prefix_path = os.environ.get(AMENT_PREFIX_PATH_ENV_VAR)
    if not ament_prefix_path:
        # raise EnvironmentError(AMENT_PREFIX_PATH_ENV_VAR)
        return []
    paths = ament_prefix_path.split(os.pathsep)
    return list(map(Path, (p for p in paths if p and os.path.exists(p))))


def _get_ament_prefix_path(name: str) -> Path:
    for prefix in _get_ament_search_paths():
        path = prefix / RESOURCE_INDEX_SUBFOLDER / 'packages' / name
        try:
            path = path.resolve(strict=True)
            if path.is_file():
                return path
        except FileNotFoundError:
            pass
    raise LookupError(f'package: {name}')
