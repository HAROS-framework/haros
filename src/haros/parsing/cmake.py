# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Final, List, Tuple

from pathlib import Path
import re
import sys

import attr
from lark import Lark, Transformer, v_args

###############################################################################
# Constants
###############################################################################

GRAMMAR_FILE: Final[Path] = Path(__file__).resolve().parent / 'cmake.lark'

RE_BRACKET: Final[re.Pattern] = re.compile(r'#\[=*\[')

_INDENT_SPACE: Final[str] = '  '

###############################################################################
# AST
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class CMakeComment:
    text: str
    line: int = 1
    column: int = 1

    def pretty(self, indent: int = 0) -> str:
        ws = _INDENT_SPACE * indent
        return '\n'.join((
            f'{ws}comment: [{self.line},{self.column}]',
            f'{ws}{_INDENT_SPACE}text:\t{self.text}'
        ))


@attr.s(auto_attribs=True, slots=True, frozen=True)
class CMakeArgument:
    value: str
    line: int = 1
    column: int = 1

    def pretty(self, indent: int = 0) -> str:
        ws = _INDENT_SPACE * indent
        return '\n'.join((
            f'{ws}argument: [{self.line},{self.column}]',
            f'{ws}{_INDENT_SPACE}value:\t{self.value}'
        ))


@attr.s(auto_attribs=True, slots=True, frozen=True)
class CMakeCommand:
    name: str
    arguments: List[CMakeArgument]
    line: int = 1
    column: int = 1
    comments: List[CMakeComment] = attr.Factory(list)

    def pretty(self, indent: int = 0) -> str:
        ws = _INDENT_SPACE * indent
        ws2 = _INDENT_SPACE * (indent + 1)
        parts = [
            f'{ws}command: [{self.line},{self.column}]',
            f'{ws2}name:\t{self.name}',
        ]
        if self.arguments:
            parts.append(f'{ws2}arguments:')
            parts.extend(a.pretty(indent=indent+2) for a in self.arguments)
        if self.comments:
            parts.append(f'{ws2}comments:')
            parts.extend(c.pretty(indent=indent+2) for c in self.comments)
        return '\n'.join(parts)


@attr.s(auto_attribs=True, slots=True, frozen=True)
class CMakeFile:
    commands: List[CMakeCommand] = attr.Factory(list)
    comments: List[CMakeComment] = attr.Factory(list)

    def pretty(self) -> str:
        parts = ['file:']
        if self.commands:
            parts.append(f'{_INDENT_SPACE}commands:')
            parts.extend(c.pretty(indent=2) for c in self.commands)
        if self.comments:
            parts.append(f'{_INDENT_SPACE}comments:')
            parts.extend(c.pretty(indent=2) for c in self.comments)
        return '\n'.join(parts)


###############################################################################
# Transformer
###############################################################################


class _ToAst(Transformer):
    @v_args(inline=True)
    def start(self, cmake: CMakeFile) -> CMakeFile:
        return cmake

    def file(self, children) -> CMakeFile:
        return CMakeFile(
            commands=[c for c in children if isinstance(c, CMakeCommand)],
            comments=[c for c in children if isinstance(c, CMakeComment)],
        )

    def command_invocation(self, children) -> CMakeCommand:
        assert len(children) == 2, 'expected an identifier and a tuple (arguments, comments)'
        arguments, comments = children[1]
        return CMakeCommand(
            str(children[0]),
            arguments,
            line=children[0].line,
            column=children[0].column,
            comments=comments,
        )

    def arguments(self, children) -> Tuple[List[CMakeArgument], List[CMakeComment]]:
        arguments = [c for c in children if isinstance(c, CMakeArgument)]
        comments = [c for c in children if isinstance(c, CMakeComment)]
        return arguments, comments

    @v_args(inline=True)
    def bracket_argument(self, token) -> CMakeArgument:
        text = self._handle_brackets(token)
        return CMakeArgument(text, line=token.line, column=token.column)

    @v_args(inline=True)
    def quoted_argument(self, token) -> CMakeArgument:
        assert len(token) >= 2, 'expected quotation marks'
        assert token.startswith('"'), 'expected quotation marks'
        assert token.endswith('"'), 'expected quotation marks'
        text = token[1:-1]
        return CMakeArgument(text, line=token.line, column=token.column)

    @v_args(inline=True)
    def unquoted_argument(self, token) -> CMakeArgument:
        assert len(token) > 0, 'expected a non-empty string'
        return CMakeArgument(str(token), line=token.line, column=token.column)

    @v_args(inline=True)
    def bracket_comment(self, token) -> CMakeComment:
        text = self._handle_brackets(token)
        return CMakeComment(text, line=token.line, column=token.column)

    @v_args(inline=True)
    def line_comment(self, token) -> CMakeComment:
        assert len(token) > 0, 'expected a non-empty string'
        assert token.startswith('#'), 'expected comment token'
        text = token[1:].strip()
        return CMakeComment(text, line=token.line, column=token.column)

    def _handle_brackets(self, token) -> str:
        match = RE_BRACKET.match(token)
        assert match is not None, f'expected bracket match at start of string: {token}'
        assert match.start() == 0, f'expected bracket match at start of string: {token}'
        n = match.end()
        assert n >= 3, f'expected at least two brakcet characters: {token}'
        text = token[n:-(n-1)]  # remove brackets from both ends
        # remove also optional starting newline
        if text.startswith('\n'):
            text = text[1:]
        elif text.startswith('\r\n'):
            text = text[2:]
        elif text.startswith('\r'):
            text = text[1:]
        return text


###############################################################################
# Interface
###############################################################################


class _CMakeParser:
    def __init__(self, grammar: str):
        self._parser = Lark(grammar)
        self._arg_parser = Lark(grammar, start='arguments')
        self._transformer = _ToAst()

    def parse(self, text: str) -> CMakeFile:
        tree = self._parser.parse(text)
        return self._transformer.transform(tree)

    def parse_arguments(self, text: str) -> List[CMakeArgument]:
        tree = self._arg_parser.parse(text)
        arguments, comments = self._transformer.transform(tree)
        return arguments


_parser = None


def parser():
    global _parser
    if _parser is None:
        _parser = _CMakeParser(GRAMMAR_FILE.read_text())
    return _parser


def parse(text):
    return parser().parse(text)
