# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Callable, Final, Tuple

from pathlib import Path

from haros.analysis.launch.python import get_python_launch_description
from haros.analysis.launch.xml import get_xml_launch_description
from haros.analysis.launch.yaml import get_yaml_launch_description
from haros.metamodel.launch import LaunchDescription

###############################################################################
# Constants
###############################################################################

PARSERS: Final[Tuple[Callable]] = (
    get_python_launch_description,
    get_xml_launch_description,
    get_yaml_launch_description,
)

###############################################################################
# Interface
###############################################################################


def get_launch_description(path: Path) -> LaunchDescription:
    for parser in PARSERS:
        try:
            return parser(path)
        except ValueError:
            pass  # wrong file extension
    raise ValueError(f'not a valid launch file: {path}')
