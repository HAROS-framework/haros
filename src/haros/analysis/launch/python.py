# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, Tuple

from pathlib import Path

#from haros.analysis.python import query
from haros.analysis.python.graph import from_ast
from haros.errors import ParseError
from haros.parsing.python import parse

LaunchModel = Any

###############################################################################
# Constants
###############################################################################

LAUNCH_ENTRY_POINT: Final[str] = 'generate_launch_description'

###############################################################################
# Interface
###############################################################################


def get_python_launch_model(path: Path) -> LaunchModel:
    if not path.is_file():
        raise ValueError(f'not a file: {path}')
    ext = path.suffix.lower()
    if ext != '.py':
        raise ValueError(f'not a valid launch file: {path}')
    code = path.read_text(encoding='utf-8')
    ast = parse(code)
    #q = query(ast)
    #return q.functions().named(LAUNCH_ENTRY_POINT)
    symbols = {
        'mymodule.MY_CONSTANT': 44,
        'mymodule.my_division': lambda a, b: (a.value // b.value) if a.is_resolved and b.is_resolved else None,
    }
    graph = from_ast(ast, symbols=symbols)
    return graph
