# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Self

###############################################################################
# Interface
###############################################################################


class HarosError(Exception):
    """Base class for all HAROS exceptions."""


###############################################################################
# Parsing Errors
###############################################################################


class ParseError(HarosError):
    """Errors related to parsing code."""


class WrongFileTypeError(ParseError):
    """The provided file is not in the expected language/extension/format."""


###############################################################################
# Analysis Errors
###############################################################################


class AnalysisError(HarosError):
    """Errors related to program analysis."""


class ControlFlowError(AnalysisError):
    """Errors related to control flow analysis."""

    # def __init__(self, *args, **kwargs):
    #     super().__init__(*args)
    #     self.module = kwargs.get('module')
    #     self.line = kwargs.get('line')
    #     self.column = kwargs.get('column')
    #     self.statement = kwargs.get('statement')

    @classmethod
    def not_looping(cls) -> Self:
        return cls('found a loop jump statement outside of a loop')

    @classmethod
    def not_branching(cls) -> Self:
        return cls('found a branching statement outside of a conditional')

    @classmethod
    def if_after_else(cls) -> Self:
        return cls('found a branching statement after an else statement')


class DataFlowError(AnalysisError):
    """Errors related to data flow analysis."""

    @classmethod
    def type_check(cls, expected: str, found: str, expr) -> Self:
        return cls(f'type check failed: expected {expected}, found {found}: {expr}')
