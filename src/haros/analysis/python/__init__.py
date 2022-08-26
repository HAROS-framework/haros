# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from haros.analysis.python.queries import ModuleQuery, Query
from haros.parsing.python.ast import PythonAst

###############################################################################
# Interface
###############################################################################


def query(obj: PythonAst) -> Query:
    if obj.is_module:
        return ModuleQuery.q([obj])
    raise TypeError(type(obj).__name__)
