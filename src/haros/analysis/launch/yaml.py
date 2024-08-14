# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final, Tuple

from pathlib import Path

from haros.errors import WrongFileTypeError
from haros.internal.interface import AnalysisSystemInterface
from haros.metamodel.launch import LaunchDescription

###############################################################################
# Constants
###############################################################################

EXTENSIONS: Final[Tuple[str]] = ('.launch.yaml', '.launch.yml', '.yaml', '.yml', '.launch')

###############################################################################
# Interface
###############################################################################


def get_yaml_launch_description(path: Path, system: AnalysisSystemInterface) -> LaunchDescription:
    if not path.is_file():
        raise FileNotFoundError(f'not a file: {path}')
    ext = path.suffix.lower()
    if ext not in EXTENSIONS:
        raise WrongFileTypeError(f'not a valid launch file: {path}')
    return parse_file(path, system)


def parse_file(path: Path, system: AnalysisSystemInterface):
    return
