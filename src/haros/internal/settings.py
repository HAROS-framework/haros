# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final

from pathlib import Path

try:
    from yaml import CDumper as Dumper
except ImportError:
    from yaml import Dumper

from attrs import asdict, define, field
from yaml import dump, safe_load

###############################################################################
# Constants
###############################################################################

SETTINGS_FILE: Final[str] = 'config.yaml'


def _default_logging_formatters() -> dict[str, Any]:
    return {
        'screenFormat': {
            'format': '[HAROS - %(levelname)s]: %(message)s',
        },
        'logfileFormat': {
            'format': '%(levelname)s - %(name)s:%(lineno)d#%(funcName)s: %(message)s',
        },
    }


def _default_logging_handlers() -> dict[str, Any]:
    return {
        'console': {
            'class': 'logging.StreamHandler',
            'level': 'WARNING',
            'formatter': 'screenFormat',
            'stream': 'ext://sys.stdout',
        },
        'logfile': {
            'class': 'logging.handlers.RotatingFileHandler',
            'level': 'DEBUG',
            'formatter': 'logfileFormat',
            'filename': 'haros.log',
            'maxBytes': 10485760,  # 10 MB
            'backupCount': 5,
        },
    }


def _default_logging_root() -> dict[str, Any]:
    return {'level': 'DEBUG', 'handlers': ['console', 'logfile']}


# follows Python's dictionary configuration schema
# https://docs.python.org/3.12/library/logging.config.html#dictionary-schema-details
DEFAULT_SETTINGS_LOGGING: Final[dict[str, Any]] = {
    'version': 1,
    'formatters': _default_logging_formatters(),
    'handlers': _default_logging_handlers(),
    'root': _default_logging_root(),
}

DEFAULT_SETTINGS_ENVIRONMENT: Final[dict[str, Any]] = {
    'copy': True,  # whether to copy the current environment
    'variables': {},  # dict[str, str]: override variables with these values
}


def _default_plugin_settings() -> dict[str, Any]:
    return {
        'enabled': True,
        'setup': {},  # keyword arguments provided at load/setup
        'teardown': {},  # keyword arguments provided at teardown
    }


DEFAULT_SETTINGS_PLUGINS: Final[dict[str, Any]] = {
    'dummy': _default_plugin_settings(),
}


def _default_parsing_cpp() -> dict[str, Any]:
    return {'parser': 'clang'}


DEFAULT_SETTINGS_PARSING: Final[dict[str, Any]] = {
    'cpp': _default_parsing_cpp(),
}

DEFAULT_SETTINGS: Final[dict[str, Any]] = {
    'logging': DEFAULT_SETTINGS_LOGGING,
    'environment': DEFAULT_SETTINGS_ENVIRONMENT,
    'plugins': DEFAULT_SETTINGS_PLUGINS,
    'parsing': DEFAULT_SETTINGS_PARSING,
}

YAML_DEFAULT_SETTINGS: Final[str] = (
    rf"""%YAML 1.1
---
{dump(DEFAULT_SETTINGS, Dumper=Dumper)}
"""
)

###############################################################################
# Data Types
###############################################################################


@define
class EnvironmentSettings:
    copy: bool = DEFAULT_SETTINGS_ENVIRONMENT['copy']
    variables: dict[str, str] = field(factory=dict)

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


@define
class LoggingSettings:
    # follows Python's dictionary configuration schema
    # https://docs.python.org/3.12/library/logging.config.html#dictionary-schema-details
    version: int = DEFAULT_SETTINGS_LOGGING['version']
    formatters: dict[str, Any] = field(factory=_default_logging_formatters)
    handlers: dict[str, Any] = field(factory=_default_logging_handlers)
    root: dict[str, Any] = field(factory=_default_logging_root)

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


@define
class ParsingSettings:
    cpp: dict[str, str] = field(factory=_default_parsing_cpp)

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


@define
class Settings:
    home: Path
    environment: EnvironmentSettings = field(factory=EnvironmentSettings)
    logging: LoggingSettings = field(factory=LoggingSettings)
    plugins: dict[str, dict[str, Any]] = field(factory=dict)
    parsing: ParsingSettings = field(factory=ParsingSettings)

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


###############################################################################
# Interface
###############################################################################


def load_as_dict(haroshome: Path) -> dict[str, Any]:
    path = haroshome / SETTINGS_FILE
    path = path.resolve()
    with path.open(mode='r', encoding='utf-8') as f:
        settings = safe_load(f)
    settings['home'] = haroshome
    return settings


def load(haroshome: Path) -> Settings:
    settings = load_as_dict(haroshome)
    return Settings(
        home=haroshome,
        environment=EnvironmentSettings(**settings['environment']),
        logging=LoggingSettings(**settings['logging']),
        plugins=settings['plugins'],
        parsing=ParsingSettings(**settings['parsing']),
    )


def defaults_as_dict() -> dict[str, Any]:
    return dict(DEFAULT_SETTINGS)


def defaults() -> dict[str, Any]:
    return Settings(
        home=Path.cwd(),  # FIXME
        environment=EnvironmentSettings(**DEFAULT_SETTINGS_ENVIRONMENT),
        logging=LoggingSettings(**DEFAULT_SETTINGS_LOGGING),
        plugins=dict(DEFAULT_SETTINGS_PLUGINS),
        parsing=ParsingSettings(**DEFAULT_SETTINGS_PARSING),
    )
