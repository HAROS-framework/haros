# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Optional, Set

import attr

###############################################################################
# File System
###############################################################################


@attr.s(auto_attribs=True, slots=True)
class StorageMetadata:
    path: Optional[str] = None  # real path (e.g. '/home/user/file')
    size: Optional[int] = None  # in bytes
    timestamp: int = 0


###############################################################################
# Source Code
###############################################################################


@attr.s(auto_attribs=True, slots=True)
class SourceCodeMetadata:
    language: str = 'Text'  # e.g. C++, Python...
    lines: int = 1  # always at least 1, even if the file is empty
    ast: Any = None


@attr.s(auto_attribs=True, slots=True)
class DevelopmentMetadata:
    description: str = ''
    authors: Set[str] = attr.Factory(set)
    maintainers: Set[str] = attr.Factory(set)
    version: str = 'unknown'
    license: str = 'unknown'
    url_home: Optional[str] = None
    url_source: Optional[str] = None
    url_tracker: Optional[str] = None


@attr.s(auto_attribs=True, slots=True, frozen=True)
class SourceCodeDependencies:
    build: Set[str] = attr.Factory(set)
    runtime: Set[str] = attr.Factory(set)
