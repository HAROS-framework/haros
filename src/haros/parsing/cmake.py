# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final, List

from pathlib import Path
import sys

import attr
from lark import Lark, ast_utils, Transformer, v_args
from lark.tree import Meta

###############################################################################
# Constants
###############################################################################

GRAMMAR_FILE: Final[Path] = Path(__file__).resolve().parent / 'cmake.lark'

###############################################################################
# AST
###############################################################################


class _Ast(ast_utils.Ast):
    pass


class _Element(_Ast):
    pass


@attr.s(auto_attribs=True, slots=True, frozen=True)
class File(_Ast, ast_utils.AsList):
    elements: List[_Element]


@attr.s(auto_attribs=True, slots=True, frozen=True)
class CommandInvocation(_Element, ast_utils.WithMeta):
    # Uses WithMeta to include line-number metadata in the meta attribute
    meta: Meta
    identifier: str
    arguments: List[_Ast]


@attr.s(auto_attribs=True, slots=True, frozen=True)
class Identifier(_Ast, ast_utils.WithMeta):
    meta: Meta
    name: str


@attr.s(auto_attribs=True, slots=True, frozen=True)
class If(_Statement):
    cond: Value
    then: CodeBlock


@attr.s(auto_attribs=True, slots=True, frozen=True)
class SetVar(_Statement):
    # Corresponds to set_var in the grammar
    name: str
    value: Value


@attr.s(auto_attribs=True, slots=True, frozen=True)
class Print(_Statement):
    value: Value


###############################################################################
# Transformer
###############################################################################


class ToAst(Transformer):
    # Define extra transformation functions, for rules that don't correspond to an AST class.

    def STRING(self, s):
        # Remove quotation marks
        return s[1:-1]

    def DEC_NUMBER(self, n):
        return int(n)

    @v_args(inline=True)
    def start(self, x):
        return x


transformer = ast_utils.create_transformer(sys.modules[__name__], ToAst())


###############################################################################
# Interface
###############################################################################


_parser = None


def parser():
    global _parser
    if _parser is None:
        grammar = GRAMMAR_FILE.read_text()
        transformer = ast_utils.create_transformer(sys.modules[__name__], ToAst())
        _parser = Lark(grammar, Transformer=transformer)
    return _parser


def parse(text):
    return parser().parse(text)
