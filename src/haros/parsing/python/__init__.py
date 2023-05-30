# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final, Optional

from pathlib import Path

from attrs import field, frozen
from lark import Lark, Transformer
from lark.indenter import PythonIndenter

from haros.parsing.python._transformer import ToAst
from haros.parsing.python.ast.common import PythonModule

###############################################################################
# Constants
###############################################################################

GRAMMAR_FILE: Final[Path] = Path(__file__).resolve().parent / 'grammar.lark'

###############################################################################
# Helper Functions
###############################################################################


def _lark_python_parser() -> Lark:
    grammar = GRAMMAR_FILE.read_text(encoding='utf-8')
    # return Lark.open_from_package(
    #     'lark',
    #     'python.lark',
    #     ['grammars'],
    #     parser='lalr',
    #     postlex=PythonIndenter(),
    #     start='file_input',
    # )
    return Lark(
        grammar,
        parser='lalr',
        postlex=PythonIndenter(),
        start='file_input',
        maybe_placeholders=True,
    )


###############################################################################
# Interface
###############################################################################


@frozen
class PythonParser:
    _parser: Lark = field(init=False, factory=_lark_python_parser, eq=False, repr=False)
    _transformer: Transformer = field(init=False, factory=ToAst, eq=False, repr=False)

    def parse(
        self,
        text: str,
        package: Optional[str] = None,
        path: Optional[str] = None,
    ) -> PythonModule:
        tree = self._parser.parse(text)
        self._transformer.file_path = path
        self._transformer.package = package
        return self._transformer.transform(tree)


_parser = None


def parser() -> PythonParser:
    global _parser
    if _parser is None:
        _parser = PythonParser()
    return _parser


def parse(text: str, package: Optional[str] = None, path: Optional[str] = None) -> PythonModule:
    return parser().parse(text, package=package, path=path)
