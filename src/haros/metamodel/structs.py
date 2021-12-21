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


@attr.s(auto_attribs=True, slots=True, frozen=True)
class StorageMetadata:
    path: str  # real storage path (e.g. '/home/user/favourite/file.txt')
    size: int
    timestamp: int


###############################################################################
# Source Code
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class SourceMetadata:
    language: str  # real storage path (e.g. '/home/user/favourite/file.txt')
    lines: int
    ast: typing.Any = None


###############################################################################
# File Objects
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class FileID:
    package: str
    path: str  # relative path within package (e.g. 'src/code.cpp')

    @property
    def name(self) -> str:
        return self.path.rsplit(sep='/', maxsplit=1)[-1]

    @property
    def directory(self) -> str:
        parts = self.path.rsplit(sep='/', maxsplit=1)
        if len(parts) > 1:
            return parts[0]
        return '.'

    def __str__(self) -> str:
        return f'file:{self.package}/{self.path}'


@attr.s(auto_attribs=True, slots=True)
class File:
    file_id: FileID
    storage: typing.Optional[StorageMetadata] = None
    source: typing.Optional[SourceMetadata] = None

    @property
    def uid(self) -> str:
        return str(self.file_id)

    @property
    def package(self):
        return self.file_id.package

    @property
    def path(self):
        return self.file_id.path
