# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, Tuple

from pathlib import Path

from haros.errors import ParseError
from haros.parsing.python import parse

LaunchModel = Any

###############################################################################
# Interface
###############################################################################


def get_python_launch_model(path: Path) -> LaunchModel:
    if not path.is_file():
        raise ValueError(f'not a file: {path}')
    ext = path.suffix.lower()
    if ext != '.py':
        raise ValueError(f'not a valid launch file: {path}')
    ast = parse(path)
