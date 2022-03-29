# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Union

from pathlib import Path

from haros.internal.config import CONFIG_FILE, DEFAULT_CONFIGS
from haros.internal.fsutil import generate_dir

###############################################################################
# Constants
###############################################################################

DIR_STRUCTURE: Final[Dict[str, Any]] = {
    'logs': {},
    'extras': {},
    'plugins': {},
    'data': {},
    'cache': {},
    'output': {},
    CONFIG_FILE: DEFAULT_CONFIGS,
}


###############################################################################
# Interface
###############################################################################


def make_at(dirpath: Union[str, Path], overwrite: bool = False):
    path = Path(dirpath).resolve()
    # Raises `FileExistsError` if the last path component exists and is not a directory.
    path.mkdir(parents=True, exist_ok=True)
    generate_dir(path, DIR_STRUCTURE, overwrite=overwrite)
