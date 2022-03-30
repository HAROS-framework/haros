# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Union

from pathlib import Path

from haros.internal.fsutil import generate_dir
from haros.internal.settings import SETTINGS_FILE, DEFAULT_SETTINGS

###############################################################################
# Constants
###############################################################################

DIR_NAME: Final[str] = '.haros'

DIR_STRUCTURE: Final[Dict[str, Any]] = {
    'logs': {},
    'extras': {},
    'plugins': {},
    'data': {},
    'cache': {},
    'output': {},
    SETTINGS_FILE: DEFAULT_SETTINGS,
}


###############################################################################
# Interface
###############################################################################


def make_at(dirpath: Union[str, Path], overwrite: bool = False):
    # ensures a '.haros' directory
    # uses the given path directly, if it ends with '.haros', or uses it
    # as a parent for a new '.haros' directory
    path = Path(dirpath).resolve()
    if path.name != DIR_NAME:
        path = path / DIR_NAME
    # Raises `FileExistsError` if the last path component exists and is not a directory.
    path.mkdir(parents=True, exist_ok=True)
    generate_dir(path, DIR_STRUCTURE, overwrite=overwrite)
