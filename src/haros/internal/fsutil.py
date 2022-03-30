# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict

from pathlib import Path

###############################################################################
# Interface
###############################################################################


def generate_dir(path: Path, structure: Dict[str, Any], overwrite: bool = True):
    """Recursively create a given directory structure."""

    # log.debug(f'generate_dir({path}, {structure})')
    for name, contents in structure.items():
        new_path = (path / name).resolve()
        exists = new_path.exists()
        if isinstance(contents, str):
            if exists and not new_path.is_file():
                raise FileExistsError(str(new_path))
            if overwrite or not exists:
                # log.info(f'Creating {new_path}')
                new_path.write_text(contents, encoding='utf-8')
        elif isinstance(contents, dict):
            if exists and not new_path.is_dir():
                raise FileExistsError(str(new_path))
            if not exists:
                # log.info(f'Creating {new_path}')
                # A missing parent raises FileNotFoundError.
                new_path.mkdir(exist_ok=True)
            generate_dir(new_path, contents, overwrite=overwrite)


def ensure_structure(path: Path, structure: Dict[str, Any]):
    """Ensure that the given path contains the given structure."""

    for name, contents in structure.items():
        new_path = (path / name).resolve()
        if isinstance(contents, str):
            if not new_path.is_file():
                raise FileNotFoundError(str(new_path))
        elif isinstance(contents, dict):
            if not new_path.is_dir():
                raise FileNotFoundError(str(new_path))
