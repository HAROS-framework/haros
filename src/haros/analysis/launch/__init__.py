# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Final, Tuple

from pathlib import Path

LaunchModel = Any

###############################################################################
# Constants
###############################################################################

PARSERS: Final[Tuple[Callable]] = (
    get_python_launch_model,
    get_xml_launch_model,
    get_yaml_launch_model,
)

###############################################################################
# Interface
###############################################################################


def get_launch_model(path: Path) -> LaunchModel:
    for parser in PARSERS:
        try:
            return parser(path)
        except ValueError:
            pass  # wrong file extension
    raise ValueError(f'not a valid launch file: {path}')
