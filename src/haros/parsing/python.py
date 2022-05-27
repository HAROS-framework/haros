# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

# from typing import Final, List, Tuple

from lark import Lark
# from lark import Lark, Transformer, v_args
from lark.indenter import PythonIndenter

###############################################################################
# Constants
###############################################################################


###############################################################################
# Interface
###############################################################################


class _PythonParser:
    def __init__(self):
        self._parser = Lark.open_from_package(
            'lark',
            'python.lark',
            ['grammars'],
            parser='lalr',
            postlex=PythonIndenter(),
            start='file_input',
        )
        # self._transformer = _ToAst()

    def parse(self, text: str):
        tree = self._parser.parse(text)
        # return self._transformer.transform(tree)
        return tree


_parser = None


def parser():
    global _parser
    if _parser is None:
        _parser = _PythonParser()
    return _parser


def parse(text):
    return parser().parse(text)
