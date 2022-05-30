# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

# from typing import Final, List, Tuple

import attr

from lark import Lark, Transformer, v_args
from lark.indenter import PythonIndenter

###############################################################################
# AST
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class PythonAst:
    line: int
    column: int


###############################################################################
# Transformer
###############################################################################


class _ToAst(Transformer):
    @v_args(inline=True)
    def number(self, n):
        return n

    @v_args(inline=True)
    def string(self, s):
        return str(s)

    @v_args(inline=True)
    def name(self, n):
        return

    def DEC_NUMBER(self, n):
        return int(n)

    def HEX_NUMBER(self, n):
        return int(n, 16)

    def OCT_NUMBER(self, n):
        return int(n, 8)

    def BIN_NUMBER(self, n):
        return int(n, 2)

    def DECIMAL(self, n):
        return float(n)

    def FLOAT_NUMBER(self, n):
        return float(n)

    def IMAG_NUMBER(self, n):
        return complex(0, float(n[:-1]))


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
        self._transformer = _ToAst()

    def parse(self, text: str):
        tree = self._parser.parse(text)
        return self._transformer.transform(tree)


_parser = None


def parser():
    global _parser
    if _parser is None:
        _parser = _PythonParser()
    return _parser


def parse(text):
    return parser().parse(text)
