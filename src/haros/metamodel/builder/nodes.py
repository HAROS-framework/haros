# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final, List

import logging
from pathlib import Path

from haros.analysis.cmake import get_targets_from_cmake
from haros.metamodel.ros import NodeModel, Languages, uid_file
from haros.parsing.cmake import parser as cmake_parser

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Interface
###############################################################################


def build_from_cmake(package: str, root: Path) -> List[NodeModel]:
    logger.info(f'build_from_cmake("{package}", "{root}")')
    nodes = []
    path = root / 'CMakeLists.txt'
    path = path.resolve(strict=True)

    logger.debug(f'calling cmake analysis on: {path}')
    targets = get_targets_from_cmake(path)

    for target in targets.values():
        logger.info(f'building node from CMake target: {target.name}')
        node = NodeModel(package, target.name, is_library=target.is_library)
        node.source.language = Languages.CPP
        node.dependencies.build.update(target.dependencies)
        for fp in target.sources:
            node.files.append(uid_file(package, fp))
        nodes.append(node)

    return nodes
