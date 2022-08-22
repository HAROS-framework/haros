# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from attrs import field, frozen
from lark import Lark, Transformer
from lark.indenter import PythonIndenter

from haros.parsing.python._transformer import ToAst
from haros.parsing.python.ast.common import PythonAst

###############################################################################
# Helper Functions
###############################################################################


def _lark_python_parser() -> Lark:
    return Lark.open_from_package(
        'lark',
        'python.lark',
        ['grammars'],
        parser='lalr',
        postlex=PythonIndenter(),
        start='file_input',
    )


###############################################################################
# Interface
###############################################################################


@frozen
class PythonParser:
    _parser: Lark = field(init=False, factory=_lark_python_parser, eq=False, repr=False)
    _transformer: Transformer = field(init=False, factory=ToAst, eq=False, repr=False)

    def parse(self, text: str) -> PythonAst:
        tree = self._parser.parse(text)
        return self._transformer.transform(tree)


_parser = None


def parser() -> PythonParser:
    global _parser
    if _parser is None:
        _parser = PythonParser()
    return _parser


def parse(text) -> PythonAst:
    return parser().parse(text)
