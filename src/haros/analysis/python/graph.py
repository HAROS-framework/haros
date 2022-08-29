# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from attrs import define

from haros.parsing.python.ast import PythonAst, PythonModule

###############################################################################
# Data Structures
###############################################################################


@define
class Scope:
    pass


###############################################################################
# Interface
###############################################################################


def from_ast(ast: PythonAst) -> Scope:
    if not ast.is_statement:
        raise TypeError(f'expected a statement, got {ast!r}')
    if ast.is_module:
        return from_module(ast)
    raise TypeError(f'unexpected tree node: {ast!r}')


def from_module(module: PythonModule) -> Scope:
    return
