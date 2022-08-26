# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, Tuple

from pathlib import Path

from haros.errors import ParseError

LaunchModel = Any

###############################################################################
# Constants
###############################################################################

EXTENSIONS: Final[Tuple[str]] = ('.launch.yaml', '.launch.yml', '.yaml', '.yml', '.launch')

###############################################################################
# Interface
###############################################################################


def get_yaml_launch_model(path: Path) -> LaunchModel:
    if not path.is_file():
        raise ValueError(f'not a file: {path}')
    ext = path.suffix.lower()
    if ext not in EXTENSIONS:
        raise ValueError(f'not a valid launch file: {path}')
    return parse_file(path)


def parse_file(path: Path):
    return
