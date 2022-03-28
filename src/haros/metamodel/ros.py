# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, List

import enum

import attr

from haros.metamodel.common import DevelopmentMetadata, SourceCodeDependencies, SourceCodeMetadata

###############################################################################
# Constants
###############################################################################


RE_NAME: Final = r'(^[a-zA-Z][a-zA-Z0-9_]*$)'


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
# ROS Source Entities
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class RosType:
    # Parameters
    package: str = attr.ib(validator=attr.validators.matches_re(RE_NAME))
    name: str = attr.ib(validator=attr.validators.matches_re(RE_NAME))

    def __str__(self) -> str:
        return f'{self.package}/{self.name}'


@attr.s(auto_attribs=True, slots=True, frozen=True)
class RosName:
    # Parameters
    text: str

    @property
    def name(self) -> str:
        name: str = self.text.rsplit(sep='/', maxsplit=1)[-1]
        if name.startswith('~'):
            return name[1:]
        return name

    @property
    def namespace(self) -> str:
        parts: List[str] = self.text.rsplit(sep='/', maxsplit=1)
        if len(parts) > 1:
            # e.g. 'a/b', '~a/b' or '/a'
            return parts[0] if parts[0] else '/'
        # else: e.g. 'a' or '~a'
        return '~' if parts[0].startswith('~') else ''

    @property
    def is_global(self) -> bool:
        return self.text.startswith('/')

    @property
    def is_private(self) -> bool:
        return self.text.startswith('~')

    @property
    def is_relative(self) -> bool:
        return not self.is_global and not self.is_private

    def join(self, other: 'RosName') -> 'RosName':
        """Uses `self` as a namespace to join with `other`."""
        if other.is_global:
            return other
        suffix: str = other.text
        if other.is_private:
            assert suffix.startswith('~')
            suffix = suffix[1:]
        else:
            assert other.is_relative
        prefix: str = self.text
        if prefix.endswith('/'):
            prefix = prefix[:-1]
        name: str = f'{prefix}/{suffix}'
        return RosName(name)

    def resolve(self, ns: 'RosName', private_ns: 'RosName') -> 'RosName':
        """Resolves `self` relative to `ns` or `private_ns`."""
        if self.is_global:
            return self
        if self.is_private:
            return private_ns.join(self)
        return ns.join(self)

    def __str__(self) -> str:
        return self.text


@attr.s(auto_attribs=True, slots=True, frozen=True)
class RosAdvertiseCall:
    # Parameters
    name: str
    namespace: str


###############################################################################
# File System Objects
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class File:
    # Parameters
    package: str = attr.ib(validator=attr.validators.matches_re(RE_NAME))
    path: str  # relative path within package (e.g. 'src/code.cpp')
    # Defaults
    # storage: StorageMetadata = attr.Factory(StorageMetadata)
    source: SourceCodeMetadata = attr.Factory(SourceCodeMetadata)
    dependencies: SourceCodeDependencies = attr.Factory(SourceCodeDependencies)

    @property
    def uid(self) -> str:
        return f'{self.package}/{self.path}'

    @property
    def name(self) -> str:
        return self.path.rsplit(sep='/', maxsplit=1)[-1]

    @property
    def directory(self) -> str:
        parts = self.path.rsplit(sep='/', maxsplit=1)
        if len(parts) > 1:
            return parts[0]
        return '.'

    def asdict(self) -> Dict[str, Any]:
        data = attr.asdict(self)
        data['uid'] = self.uid
        return data


@attr.s(auto_attribs=True, slots=True, frozen=True)
class Package:
    # Parameters
    name: str = attr.ib(validator=attr.validators.matches_re(RE_NAME))
    # Defaults
    is_metapackage: bool = False
    files: List[str] = attr.Factory(list)
    nodes: List[str] = attr.Factory(list)
    # storage: StorageMetadata = attr.Factory(StorageMetadata)
    metadata: DevelopmentMetadata = attr.Factory(DevelopmentMetadata)
    dependencies: SourceCodeDependencies = attr.Factory(SourceCodeDependencies)

    @property
    def uid(self) -> str:
        return self.name

    def asdict(self) -> Dict[str, Any]:
        data = attr.asdict(self)
        data['uid'] = self.uid
        return data


@attr.s(auto_attribs=True, slots=True, frozen=True)
class Node:
    # Parameters
    package: str = attr.ib(validator=attr.validators.matches_re(RE_NAME))
    name: str = attr.ib(validator=attr.validators.matches_re(RE_NAME))
    # Defaults
    is_nodelet: bool = False
    # TODO rosname: typing.Optional[RosName] = None
    files: List[str] = attr.Factory(list)
    source: SourceCodeMetadata = attr.Factory(SourceCodeMetadata)
    # TODO function calls

    @property
    def uid(self) -> str:
        return f'{self.package}/{self.name}'

    def asdict(self) -> Dict[str, Any]:
        return attr.asdict(self)


###############################################################################
# ROS Runtime Objects
###############################################################################


###############################################################################
# Metamodel Instance
###############################################################################


@attr.s(auto_attribs=True, slots=True, frozen=True)
class Metamodel:
    name: str
    packages: Dict[str, Package] = attr.Factory(dict)
    files: Dict[str, File] = attr.Factory(dict)
    nodes: Dict[str, Node] = attr.Factory(dict)
    # NOTE: still not sure whether to include storage here
    # storage: Dict[str, StorageMetadata] = attr.Factory(dict)

    def asdict(self) -> Dict[str, Any]:
        return attr.asdict(self)
