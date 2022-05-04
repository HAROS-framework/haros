# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Iterable

import logging
from pathlib import Path

from haros.internal.fsutil import is_ros_package, is_workspace, StorageManager
from haros.metamodel.builder.packages import build_from_path as build_package
from haros.metamodel.ros import ProjectModel

###############################################################################
# Globals
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Interface
###############################################################################

def build_from_paths(name: str, paths: Iterable[Path]) -> ProjectModel:
    storage = StorageManager()
    adhoc = []
    for path in paths:
        if is_ros_package(path):
            adhoc.append(path)
        else:
            storage.workspaces.append(path)
            if not is_workspace(path):
                logger.warning(f'builder: workspace without "src" directory: {str(path)}')
    storage.crawl()
    for path in adhoc:
        storage.packages[path.name] = path
    return build(name, storage)


def build(name: str, storage: StorageManager) -> ProjectModel:
    model = ProjectModel(name)
    _build_packages_and_files(model, storage)
    return model


###############################################################################
# Helper Functions
###############################################################################


def _build_packages_and_files(model: ProjectModel, storage: StorageManager):
    for name, path in storage.packages.items():
        try:
            package = build_package(path)
            assert package.name == name, f'(package.name) {package.name} != {name} (name)'
        except ValueError as e:
            logger.warning(f'builder: the package {name} will be ignored:\n{e}')
        model.packages[name] = package
        for fp in package.files:
            pass  # TODO: build file
