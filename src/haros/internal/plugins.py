# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, List

from importlib import metadata
import logging

###############################################################################
# Constants
###############################################################################

ENTRY_POINT: Final[str] = 'haros.plugins'

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Interface
###############################################################################


def load(settings: Dict[str, Any]) -> List[Any]:
    logger.info(f'plugins: searching {ENTRY_POINT}')
    plugins = []
    try:
        eps = metadata.entry_points()[ENTRY_POINT]
        for ep in eps:
            logger.info(f'plugins: found {ep.name} ({ep.value})')
            try:
                plugin = ep.load()
                logger.info(f'plugins: imported {ep.name}')
                try:
                    plugin.haros_setup()
                setup = getattr(plugin, 'haros_setup', _noop)
                setup()
                plugins.append(plugin)
                logger.info(f'plugins: loaded {ep.name}')
            except Exception as e:
                logger.exception(f'plugins: error {ep.name}', e)
    except KeyError:
        logger.warning('plugins: none to load')
    return plugins


###############################################################################
# Helper Functions
###############################################################################


def _noop():
    pass
