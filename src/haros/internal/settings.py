# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final

from pathlib import Path
try:
    from yaml import CDumper as Dumper
except ImportError:
    from yaml import Dumper
from yaml import dump, safe_load

import attr

###############################################################################
# Constants
###############################################################################

SETTINGS_FILE: Final[str] = 'config.yaml'


def _default_logging_formatters() -> Dict[str, Any]:
    return {
        'screenFormat': {
            'format': '[HAROS - %(levelname)s]: %(message)s',
        },
        'logfileFormat': {
            'format': '%(levelname)s - %(name)s:%(lineno)d#%(funcName)s: %(message)s',
        },
    }


def _default_logging_handlers() -> Dict[str, Any]:
    return {
        'console': {
            'class': 'logging.StreamHandler',
            'level': 'WARNING',
            'formatter': 'screenFormat',
            'stream': 'ext://sys.stdout',
        },
        'logfile': {
            'class' : 'logging.handlers.RotatingFileHandler',
            'level': 'DEBUG',
            'formatter': 'logfileFormat',
            'filename': 'haros.log',
            'maxBytes': 10485760,  # 10 MB
            'backupCount': 5,
        },
    }


def _default_logging_root() -> Dict[str, Any]:
    return {
        'level': 'DEBUG',
        'handlers': ['console', 'logfile']
    }


# follows Python's Dictionary configuration schema
# https://docs.python.org/3.8/library/logging.config.html#dictionary-schema-details
DEFAULT_SETTINGS_LOGGING: Final[Dict[str, Any]] = {
    'version': 1,
    'formatters': _default_logging_formatters(),
    'handlers': _default_logging_handlers(),
    'root': _default_logging_root(),
}

DEFAULT_SETTINGS_ENVIRONMENT: Final[Dict[str, Any]] = {
    'copy': True,  # whether to copy the current environment
    'variables': {},  # Dict[str, str]: override variables with these values
}


def _default_plugin_settings() -> Dict[str, Any]:
    return {
        'enabled': False,
        'args': {},  # keyword arguments provided at load/setup
    }


DEFAULT_SETTINGS_PLUGINS: Final[Dict[str, Any]] = {
    'dummy': _default_plugin_settings(),
}


def _default_parsing_cpp() -> Dict[str, Any]:
    return { 'parser': 'clang' }


DEFAULT_SETTINGS_PARSING: Final[Dict[str, Any]] = {
    'cpp': _default_parsing_cpp(),
}

DEFAULT_SETTINGS: Final[Dict[str, Any]] = {
    'logging': DEFAULT_SETTINGS_LOGGING,
    'environment': DEFAULT_SETTINGS_ENVIRONMENT,
    'plugins': DEFAULT_SETTINGS_PLUGINS,
    'parsing': DEFAULT_SETTINGS_PARSING,
}

YAML_DEFAULT_SETTINGS: Final[str] = r"""%YAML 1.1
---
{}
""".format(dump(DEFAULT_SETTINGS, Dumper=Dumper))

###############################################################################
# Data Types
###############################################################################

@attr.s(auto_attribs=True, slots=True, frozen=False)
class EnvironmentSettings:
    copy: bool = DEFAULT_SETTINGS_ENVIRONMENT['copy']
    variables: Dict[str, str] = attr.Factory(dict)

    def asdict(self) -> Dict[str, Any]:
        return attr.asdict(self)


@attr.s(auto_attribs=True, slots=True, frozen=False)
class LoggingSettings:
    # follows Python's Dictionary configuration schema
    # https://docs.python.org/3.8/library/logging.config.html#dictionary-schema-details
    version: int = DEFAULT_SETTINGS_LOGGING['version']
    formatters: Dict[str, Any] = attr.Factory(_default_logging_formatters)
    handlers: Dict[str, Any] = attr.Factory(_default_logging_handlers)
    root: Dict[str, Any] = attr.Factory(_default_logging_root)

    def asdict(self) -> Dict[str, Any]:
        return attr.asdict(self)


@attr.s(auto_attribs=True, slots=True, frozen=False)
class ParsingSettings:
    cpp: Dict[str, str] = attr.Factory(_default_parsing_cpp)

    def asdict(self) -> Dict[str, Any]:
        return attr.asdict(self)


@attr.s(auto_attribs=True, slots=True, frozen=False)
class Settings:
    home: Path
    environment: EnvironmentSettings = attr.Factory(EnvironmentSettings)
    logging: LoggingSettings = attr.Factory(LoggingSettings)
    plugins: Dict[str, Dict[str, Any]] = attr.Factory(dict)
    parsing: ParsingSettings = attr.Factory(ParsingSettings)

    def asdict(self) -> Dict[str, Any]:
        return attr.asdict(self)


###############################################################################
# Interface
###############################################################################


def load_as_dict(haroshome: Path) -> Dict[str, Any]:
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


def defaults_as_dict() -> Dict[str, Any]:
    return dict(DEFAULT_SETTINGS)


def defaults() -> Dict[str, Any]:
    return Settings(
        home=Path.cwd(),  # FIXME
        environment=EnvironmentSettings(**DEFAULT_SETTINGS_ENVIRONMENT),
        logging=LoggingSettings(**DEFAULT_SETTINGS_LOGGING),
        plugins=dict(DEFAULT_SETTINGS_PLUGINS),
        parsing=ParsingSettings(**DEFAULT_SETTINGS_PARSING),
    )
