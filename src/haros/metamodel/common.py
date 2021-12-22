# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

import typing

import attr

###############################################################################
# File System
###############################################################################


@attr.s(auto_attribs=True, slots=True)
class StorageMetadata:
    path: typing.Optional[str] = None  # real path (e.g. '/home/user/file')
    size: typing.Optional[int] = None  # in bytes
    timestamp: int = 0


###############################################################################
# Source Code
###############################################################################


@attr.s(auto_attribs=True, slots=True)
class SourceCodeMetadata:
    language: str = 'Text'  # e.g. C++, Python...
    lines: int = 1  # always at least 1, even if the file is empty
    ast: typing.Any = None


@attr.s(auto_attribs=True, slots=True)
class DevelopmentMetadata:
    description: str = ''
    authors: typing.Set[str] = attr.Factory(set)
    maintainers: typing.Set[str] = attr.Factory(set)
    version: str = 'unknown'
    license: str = 'unknown'
    url_home: typing.Optional[str] = None
    url_source: typing.Optional[str] = None
    url_tracker: typing.Optional[str] = None
