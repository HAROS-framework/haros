# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final

import logging
from pathlib import Path

from haros.internal.fsutil import crawl_package, is_ros_package
from haros.metamodel.ros import PackageModel

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Interface
###############################################################################


def build_from_path(path: Path) -> PackageModel:
    if not is_ros_package(path):
        raise ValueError('not a package: ' + str(path))
    name = path.name
    package = PackageModel(name)  # raise ValueError
    files = crawl_package(path)
    if not files:
        raise ValueError('ignored package: ' + str(path))
    package.files.extend((fp.as_posix() for fp in files))
    return package
