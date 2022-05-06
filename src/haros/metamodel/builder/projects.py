# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final, Iterable, Mapping

import logging
from pathlib import Path

from haros.internal.fsutil import crawl_workspace
from haros.metamodel.builder.files import build_from_package as build_file
from haros.metamodel.builder.packages import build_from_path as build_package
from haros.metamodel.ros import ProjectModel

###############################################################################
# Globals
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Interface
###############################################################################


def build_from_ws(name: str, ws: Path) -> ProjectModel:
    packages = crawl_workspace(ws)
    return build_from_package_paths(name, packages)


def build_from_package_paths(name: str, packages: Mapping[str, Path]) -> ProjectModel:
    model = ProjectModel(name)
    _build_packages_and_files(model, packages)
    return model


###############################################################################
# Helper Functions
###############################################################################


def _build_packages_and_files(model: ProjectModel, packages: Mapping[str, Path]):
    for name, path in packages.items():
        try:
            package = build_package(path)
            assert package.name == name, f'(package.name) {package.name} != {name} (name)'
        except ValueError as e:
            logger.warning(f'builder: the package {name} will be ignored:\n{e}')
        model.packages[package.uid] = package
        for fp in package.files:
            file = build_file(name, fp, path)
            model.files[file.uid] = file
