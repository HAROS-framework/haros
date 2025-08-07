# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final

from collections.abc import Callable
from pathlib import Path

from haros.analysis.launch.python import get_python_launch_description
from haros.analysis.launch.xml import get_xml_launch_description
from haros.analysis.launch.yaml import get_yaml_launch_description
from haros.errors import WrongFileTypeError
from haros.internal.interface import AnalysisSystemInterface
from haros.metamodel.launch import LaunchDescription

###############################################################################
# Constants
###############################################################################

PARSERS: Final[tuple[Callable, ...]] = (
    get_python_launch_description,
    get_xml_launch_description,
    get_yaml_launch_description,
)

###############################################################################
# Interface
###############################################################################


def get_launch_description(path: Path, system: AnalysisSystemInterface) -> LaunchDescription:
    # let FileNotFoundError bubble up
    for parser in PARSERS:
        try:
            return parser(path, system)
        except WrongFileTypeError:
            pass  # wrong file extension
    raise WrongFileTypeError(f'not a valid launch file: {path}')
