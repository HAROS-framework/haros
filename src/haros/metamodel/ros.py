# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

import typing

import enum

import attr

from haros.metamodel.common import DevelopmentMetadata, SourceCodeMetadata, StorageMetadata

###############################################################################
# Constants
###############################################################################


@enum.unique
class Languages(enum.Enum):
    TEXT = 'Text'
    XML = 'XML'
    YAML = 'YAML'
    JSON = 'JSON'
    CPP = 'C++'
    PYTHON = 'Python'
    CMAKE = 'CMake'
    ROSPKG = 'ROS Package XML'
    ROSLAUNCH = 'ROS Launch XML'
    ROSMSG = 'ROS Message'
    ROSSRV = 'ROS Service'
    ROSACTION = 'ROS Action'


###############################################################################
# Metamodel Classes
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class File:
    uid: str = attr.ib(init=False)
    package: str
    path: str  # relative path within package (e.g. 'src/code.cpp')
    storage: StorageMetadata = attr.Factory(StorageMetadata)
    source: SourceCodeMetadata = attr.Factory(SourceCodeMetadata)

    def __attrs_post_init__(self):
        object.__setattr__(self, 'uid', f'file:{self.package}/{self.path}')

    @property
    def name(self) -> str:
        return self.path.rsplit(sep='/', maxsplit=1)[-1]

    @property
    def directory(self) -> str:
        parts = self.path.rsplit(sep='/', maxsplit=1)
        if len(parts) > 1:
            return parts[0]
        return '.'

    def asdict(self):
        return attr.asdict(self)


@attr.s(auto_attribs=True, slots=True, frozen=True)
class Package:
    uid: str = attr.ib(init=False)
    name: str
    is_metapackage: bool = False
    files: typing.List[str] = attr.Factory(list)
    nodes: typing.List[str] = attr.Factory(list)
    storage: StorageMetadata = attr.Factory(StorageMetadata)
    metadata: DevelopmentMetadata = attr.Factory(DevelopmentMetadata)

    def __attrs_post_init__(self):
        object.__setattr__(self, 'uid', f'pkg:{self.name}')

    def asdict(self):
        return attr.asdict(self)
