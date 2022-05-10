# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final

from pathlib import Path

from lark import Lark

###############################################################################
# Constants
###############################################################################

GRAMMAR_FILE: Final[Path] = Path(__file__).resolve().parent / 'cmake.lark'

###############################################################################
# Interface
###############################################################################


def parser():
    grammar = GRAMMAR_FILE.read_text()
    return Lark(grammar)
