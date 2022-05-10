# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from pathlib import Path

from haros.parsing.cmake import parser as cmake_parser

###############################################################################
# Tests
###############################################################################


def test_valid_files():
    parser = cmake_parser()
    examples = Path(__file__).parent.parent / 'ws1' / 'src2' / 'cmake'
    examples = examples.resolve(strict=True)
    for path in examples.iterdir():
        if not path.is_file():
            continue
        text = path.read_text()
        parser.parse(text)
