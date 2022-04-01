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
logging:
    # follows Python's Dictionary configuration schema
    # https://docs.python.org/3.8/library/logging.config.html#dictionary-schema-details
    version: 1
    formatters:
        screenFormat:
            format: '[HAROS - %(levelname)s]: %(message)s'
        logfileFormat:
            format: '%(levelname)s - %(name)s:%(lineno)d#%(funcName)s: %(message)s'
    handlers:
        console:
            class: logging.StreamHandler
            level: WARNING
            formatter: screenFormat
            stream: ext://sys.stdout
        logfile:
            class : logging.handlers.RotatingFileHandler
            level: DEBUG
            formatter: logfileFormat
            filename: haros.log
            maxBytes: 10485760  # 10 MB
            backupCount: 5
    root:
        level: DEBUG
        handlers: [console, logfile]
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
    return safe_load(DEFAULT_SETTINGS)
