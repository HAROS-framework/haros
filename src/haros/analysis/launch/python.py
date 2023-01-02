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


def launch_description_function(arg_list):
    values = []
    if arg_list.is_resolved:
        for arg in arg_list.value:
            if arg.is_resolved:
                values.append(arg)
            else:
                values.append('?')
    return f'launch.LaunchDescription({values!r})'


def declare_launch_argument_function(name, default_value=None, description=None):
    arg = name.value if name.is_resolved else '?'
    value = default_value.value if (default_value is not None and default_value.is_resolved) else '?'
    text = description.value if (description is not None and description.is_resolved) else ''
    return f'DeclareLaunchArgument({arg!r}, default_value={value!r}, description={text!r})'


def launch_configuration_function(name, default=None):
    cfg = name.value if name.is_resolved else '?'
    value = default.value if (default is not None and default.is_resolved) else '?'
    return f'LaunchConfiguration({cfg!r}, default={value!r})'


LAUNCH_SYMBOLS = {
    'launch.LaunchDescription': launch_description_function,
    'launch.actions.DeclareLaunchArgument': declare_launch_argument_function,
    'launch.substitutions.LaunchConfiguration': launch_configuration_function,
}


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
    symbols.update(LAUNCH_SYMBOLS)
    graph = from_ast(ast, symbols=symbols)
    return graph
