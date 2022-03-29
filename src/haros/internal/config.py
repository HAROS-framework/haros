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

CONFIG_FILE: Final[str] = 'config.yaml'

DEFAULT_CONFIGS: Final[str] = r"""%YAML 1.1
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

###############################################################################
# Interface
###############################################################################


def load(haroshome: Path) -> Dict[str, Any]:
    path = haroshome / CONFIG_FILE
    path = path.resolve()
    with path.open(mode='r', encoding='utf-8') as f:
        settings = safe_load(f)
    return settings
