# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Callable, Dict, Final, List

from importlib import metadata
import logging

import attr

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
    def from_module(cls, name, module):
        return cls(
            name,
            module,
            setup=getattr(module, 'haros_setup', _noop),
            on_analysis_begin=getattr(module, 'haros_on_analysis_begin', _noop),
            on_analysis_end=getattr(module, 'haros_on_analysis_end', _noop),
        )


def load(settings: Dict[str, Any]) -> List[HarosPluginInterface]:
    logger.info(f'plugins: searching {ENTRY_POINT}')
    plugins = []
    try:
        eps = metadata.entry_points()[ENTRY_POINT]
        for ep in eps:
            logger.info(f'plugins: found {ep.name} ({ep.value})')
            try:
                module = ep.load()
                logger.info(f'plugins: imported {ep.name}')
                plugin = HarosPluginInterface.from_module(ep.name, module)
                plugin.setup(**settings.get(ep.name, {}))
                plugins.append(plugin)
                logger.info(f'plugins: loaded {ep.name}')
            except Exception as e:
                logger.exception(f'plugins: error {ep.name}', e)
    except KeyError:
        logger.warning('plugins: none to load')
    return plugins
