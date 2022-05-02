# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Dict, Final, List

from importlib import metadata
import importlib.util
import logging
from pathlib import Path

import attr

from haros.internal.settings import Settings

###############################################################################
# Constants
###############################################################################

ENTRY_POINT: Final[str] = 'haros.plugins'

logger: Final[logging.Logger] = logging.getLogger(__name__)


def _noop(*args, **kwargs):
    pass


###############################################################################
# Interface
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class HarosPluginInterface:
    # Attributes
    name: str
    module: Any
    # Hooks
    setup: Callable = _noop
    on_analysis_begin: Callable = _noop
    on_analysis_end: Callable = _noop


    @classmethod
    def from_module(cls, name: str, module: Any, settings: Dict[str, Any]):
        args = settings.get('args', {})
        setup = getattr(module, 'haros_setup', _noop)
        return cls(
            name,
            module,
            setup=lambda: setup(**args),
            on_analysis_begin=getattr(module, 'haros_on_analysis_begin', _noop),
            on_analysis_end=getattr(module, 'haros_on_analysis_end', _noop),
        )


def load(haroshome: Path, settings: Dict[str, Dict[str, Any]]) -> List[HarosPluginInterface]:
    plugins = _load_from_entrypoints(settings)
    plugins.extend(_load_from_haroshome(haroshome, settings))
    if not plugins:
        logger.warning('plugins: none to load')
    return plugins


###############################################################################
# Helper Functions
###############################################################################


def _load_from_entrypoints(settings: Dict[str, Dict[str, Any]]) -> List[HarosPluginInterface]:
    logger.info(f'plugins: searching {ENTRY_POINT}')
    plugins = []
    try:
        eps = metadata.entry_points()[ENTRY_POINT]
        for ep in eps:
            logger.info(f'plugins: found {ep.name} ({ep.value})')
            config = settings.get(ep.name, {})
            if config['enabled'] is False:
                logger.info(f'plugins: skipping {ep.name} (disabled)')
                continue
            try:
                module = ep.load()
                logger.info(f'plugins: imported {ep.name}')
                plugin = HarosPluginInterface.from_module(ep.name, module, config)
                plugin.setup()
                plugins.append(plugin)
                logger.info(f'plugins: loaded {ep.name}')
            except Exception as e:
                logger.exception(f'plugins: error {ep.name}', e)
    except KeyError:
        pass
    return plugins


def _load_from_haroshome(
    haroshome: Path,
    settings: Dict[str, Dict[str, Any]],
) -> List[HarosPluginInterface]:
    try:
        directory = (haroshome / 'plugins').resolve(strict=True)
    except FileNotFoundError:
        logger.error(f'plugins: {haroshome / "plugins"} does not exist')
        return []
    plugins = []
    for path in directory.iterdir():
        if not path.is_file():
            continue
        if path.suffix and path.suffix != '.py':
            logger.warning(f'plugins: not a plugin module: {path}')
            continue
        name = path.stem
        logger.info(f'plugins: found {name} ({path})')
        config = settings.get(name, {})
        if config['enabled'] is False:
            logger.info(f'plugins: skipping {name} (disabled)')
            continue
        try:
            spec = importlib.util.spec_from_file_location(name, str(path))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            logger.info(f'plugins: imported {name}')
            plugin = HarosPluginInterface.from_module(name, module, config)
            plugin.setup()
            plugins.append(plugin)
            logger.info(f'plugins: loaded {name}')
        except Exception as e:
            logger.exception(f'plugins: error {name}', e)
    return plugins
