# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final

from pathlib import Path
from yaml import safe_load

###############################################################################
# Constants
###############################################################################

SETTINGS_FILE: Final[str] = 'config.yaml'

DEFAULT_SETTINGS: Final[str] = r"""%YAML 1.1
---
logs:
    # level: info | warning | error
    level: warning
    max_files: 10
    max_size_kb: 10000
environment: true
plugins:
    dummy:
        enabled: false
        args: {}
parsing:
    cpp:
        parser: clang
"""

###############################################################################
# Interface
###############################################################################


def load(haroshome: Path) -> Dict[str, Any]:
    path = haroshome / SETTINGS_FILE
    path = path.resolve()
    with path.open(mode='r', encoding='utf-8') as f:
        settings = safe_load(f)
    return settings


def defaults() -> Dict[str, Any]:
    return {}
