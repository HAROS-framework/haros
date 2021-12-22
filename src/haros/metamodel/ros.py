# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

import typing

import enum

import attr

from haros.metamodel.common import DevelopmentMetadata, SourceCodeDependencies, SourceCodeMetadata

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
# Metamodel Objects
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class File:
    uid: str = attr.ib(init=False)
    # Parameters
    package: str
    path: str  # relative path within package (e.g. 'src/code.cpp')
    # Defaults
    # storage: StorageMetadata = attr.Factory(StorageMetadata)
    source: SourceCodeMetadata = attr.Factory(SourceCodeMetadata)
    dependencies: SourceCodeDependencies = attr.Factory(SourceCodeDependencies)

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
    # Parameters
    name: str
    # Defaults
    is_metapackage: bool = False
    files: typing.List[str] = attr.Factory(list)
    nodes: typing.List[str] = attr.Factory(list)
    # storage: StorageMetadata = attr.Factory(StorageMetadata)
    metadata: DevelopmentMetadata = attr.Factory(DevelopmentMetadata)
    dependencies: SourceCodeDependencies = attr.Factory(SourceCodeDependencies)

    def __attrs_post_init__(self):
        object.__setattr__(self, 'uid', f'pkg:{self.name}')

    def asdict(self):
        return attr.asdict(self)


@attr.s(auto_attribs=True, slots=True, frozen=True)
class Node:
    uid: str = attr.ib(init=False)
    # Parameters
    package: str
    name: str
    # Defaults
    is_nodelet: bool = False
    # TODO rosname: typing.Optional[RosName] = None
    files: typing.List[str] = attr.Factory(list)
    source: SourceCodeMetadata = attr.Factory(SourceCodeMetadata)
    # TODO function calls

    def __attrs_post_init__(self):
        object.__setattr__(self, 'uid', f'node:{self.package}/{self.name}')

    def asdict(self):
        return attr.asdict(self)


###############################################################################
# Metamodel Instance
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class Metamodel:
    name: str
    packages: typing.Dict[str, Package] = attr.Factory(dict)
    files: typing.Dict[str, File] = attr.Factory(dict)
    # nodes: typing.Dict[str, Node] = attr.Factory(dict)
    # NOTE: still not sure whether to include storage here
    # storage: typing.Dict[str, StorageMetadata] = attr.Factory(dict)

    def __attrs_post_init__(self):
        object.__setattr__(self, 'uid', f'pkg:{self.name}')

    def asdict(self):
        return attr.asdict(self)
