# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Dict, Final, List, Tuple

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

HOOKS: Final[Tuple] = (
    # 'setup',
    'on_analysis_begin',
    'on_analysis_end',
)

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
    settings: Dict[str, Any] = attr.Factory(dict)
    hooks: Dict[str, Callable] = attr.Factory(dict)

    def __attrs_post_init__(self):
        for name in HOOKS:
            fn = self.hooks.get(name)
            if fn is None:
                self.hooks[name] = getattr(self.module, f'haros_{name}', _noop)

    def setup(self):
        setup = getattr(self.module, 'haros_setup', _noop)
        args = self.settings.get('setup', {})
        return setup(**args)

    def teardown(self):
        teardown = getattr(self.module, 'haros_teardown', _noop)
        args = self.settings.get('teardown', {})
        return teardown(**args)


@attr.s(auto_attribs=True, slots=False, frozen=True)
class PluginManager:
    plugins: List[HarosPluginInterface] = attr.Factory(list)
    # plugin name -> error
    # this serves to disable a plugin after it crashes
    errors: Dict[str, Exception] = attr.Factory(dict)

    def __attrs_post_init__(self):
        for name in HOOKS:
            object.__setattr__(self, name, self._hook(name))

    def _hook(self, name):
        def hook(*args, **kwargs):
            for plugin in self.plugins:
                if plugin.name in self.errors:
                    logger.info(f'plugins: skip haros_{name} for {plugin.name} due to error')
                    continue
                try:
                    plugin.hooks[name](*args, **kwargs)
                except Exception as e:
                    logger.error(f'plugins: error on {plugin.name}.haros_{name}:\n{e}')
                    self.errors[plugin.name] = e
                    try:
                        plugin.teardown()
                    except Exception:
                        pass
        return hook


def load(haroshome: Path, settings: Dict[str, Dict[str, Any]]) -> PluginManager:
    plugins = _load_from_entrypoints(settings)
    plugins.extend(_load_from_haroshome(haroshome, settings))
    if not plugins:
        logger.warning('plugins: none to load')
    return PluginManager(plugins=plugins)


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
            config = settings.get(ep.name, {'enabled': True})
            if config['enabled'] is False:
                logger.info(f'plugins: skipping {ep.name} (disabled)')
                continue
            try:
                module = ep.load()
                logger.info(f'plugins: imported {ep.name}')
                plugin = HarosPluginInterface(ep.name, module, settings=config)
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
        config = settings.get(name, {'enabled': True})
        if config['enabled'] is False:
            logger.info(f'plugins: skipping {name} (disabled)')
            continue
        try:
            spec = importlib.util.spec_from_file_location(name, str(path))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            logger.info(f'plugins: imported {name}')
            plugin = HarosPluginInterface(name, module, settings=config)
            plugin.setup()
            plugins.append(plugin)
            logger.info(f'plugins: loaded {name}')
        except Exception as e:
            logger.exception(f'plugins: error {name}', e)
    return plugins
