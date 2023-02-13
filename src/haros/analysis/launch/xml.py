# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final, Tuple

from pathlib import Path

from haros.errors import ParseError
from haros.metamodel.launch import LaunchDescription

###############################################################################
# Constants
###############################################################################

EXTENSIONS: Final[Tuple[str]] = ('.launch.xml', '.xml', '.launch')

###############################################################################
# Interface
###############################################################################


def get_xml_launch_description(path: Path) -> LaunchDescription:
    if not path.is_file():
        raise ValueError(f'not a file: {path}')
    ext = path.suffix.lower()
    if ext not in EXTENSIONS:
        raise ValueError(f'not a valid launch file: {path}')
    return parse_file(path)


def parse_file(path: Path):
    return
