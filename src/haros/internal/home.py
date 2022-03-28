# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Union

from pathlib import Path

from haros.internal.fsutil import generate_dir

###############################################################################
# Constants
###############################################################################

CONFIG_YAML: Final[str] = r"""%YAML 1.1
---
logs:
    # level: info | warning | error
    level: warning
    max_files: 10
    max_size_kb: 10000
environment: true
plugins:
    disable: []
parsing:
    cpp:
        parser: clang
"""


DIR_STRUCTURE: Final[Dict[str, Any]] = {
    'logs': {},
    'extras': {},
    'plugins': {},
    'data': {},
    'cache': {},
    'output': {},
    'config.yaml': CONFIG_YAML,
}


###############################################################################
# Interface
###############################################################################


def make_at(dirpath: Union[str, Path], overwrite: bool = False):
    path = Path(dirpath).resolve()
    # Raises `FileExistsError` if the last path component exists and is not a directory.
    path.mkdir(parents=True, exist_ok=True)
    generate_dir(path, DIR_STRUCTURE, overwrite=overwrite)
