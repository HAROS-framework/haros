# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, List

import enum

from attrs import asdict, field, frozen
from attrs.validators import matches_re

from haros.metamodel.common import DevelopmentMetadata, SourceCodeDependencies, SourceCodeMetadata

###############################################################################
# Constants
###############################################################################


RE_NAME: Final = r'(^[a-zA-Z][a-zA-Z0-9_]*$)'


@enum.unique
class Languages(enum.Enum):
    TEXT = 'Text'
    BINARY = 'Binary'
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
# Base Classes
###############################################################################


@frozen
class RosMetamodelEntity:
    @property
    def is_filesystem_entity(self) -> bool:
        return False

    @property
    def is_source_entity(self) -> bool:
        return False

    @property
    def is_runtime_entity(self) -> bool:
        return False

    def asdict(self) -> Dict[str, Any]:
        return asdict(self)


@frozen
class RosFileSystemEntity(RosMetamodelEntity):
    @property
    def is_filesystem_entity(self) -> bool:
        return True


@frozen
class RosSourceEntity(RosMetamodelEntity):
    @property
    def is_source_entity(self) -> bool:
        return True


@frozen
class RosRuntimeEntity(RosMetamodelEntity):
    @property
    def is_runtime_entity(self) -> bool:
        return True


###############################################################################
# ROS Source Entities
###############################################################################


@frozen
class RosType:
    # Parameters
    package: str = field(validator=matches_re(RE_NAME))
    name: str = field(validator=matches_re(RE_NAME))

    def __str__(self) -> str:
        return f'{self.package}/{self.name}'


@frozen
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

    @classmethod
    def global_namespace(cls) -> 'RosName':
        return cls('/')

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


@frozen
class RosAdvertiseCall(RosSourceEntity):
    # Parameters
    name: str
    namespace: str


###############################################################################
# File System Objects
###############################################################################


@frozen
class FileModel(RosFileSystemEntity):
    # Parameters
    package: str = field(validator=matches_re(RE_NAME))
    path: str  # relative path within package (e.g. 'src/code.cpp')
    # Defaults
    # storage: StorageMetadata = field(factory=StorageMetadata)
    source: SourceCodeMetadata = field(factory=SourceCodeMetadata)
    dependencies: SourceCodeDependencies = field(factory=SourceCodeDependencies)

    @property
    def uid(self) -> str:
        return uid_file(self.package, self.path)

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
        data = asdict(self)
        data['uid'] = self.uid
        return data


@frozen
class PackageModel(RosFileSystemEntity):
    # Parameters
    name: str = field(validator=matches_re(RE_NAME))
    # Defaults
    files: List[str] = field(factory=list)
    nodes: List[str] = field(factory=list)
    # storage: StorageMetadata = field(factory=StorageMetadata)
    metadata: DevelopmentMetadata = field(factory=DevelopmentMetadata)
    dependencies: SourceCodeDependencies = field(factory=SourceCodeDependencies)

    @property
    def uid(self) -> str:
        return self.name

    def asdict(self) -> Dict[str, Any]:
        data = asdict(self)
        data['uid'] = self.uid
        return data


@frozen
class NodeModel(RosFileSystemEntity):
    # Parameters
    package: str = field(validator=matches_re(RE_NAME))
    name: str = field(validator=matches_re(RE_NAME))
    # Defaults
    is_library: bool = False
    # TODO rosname: typing.Optional[RosName] = None
    files: List[str] = field(factory=list)
    source: SourceCodeMetadata = field(factory=SourceCodeMetadata)
    dependencies: SourceCodeDependencies = field(factory=SourceCodeDependencies)
    # TODO function calls

    @property
    def uid(self) -> str:
        return f'{self.package}/{self.name}'


###############################################################################
# ROS Runtime Objects
###############################################################################


###############################################################################
# Metamodel Instance
###############################################################################


@frozen
class ProjectModel:
    name: str
    packages: Dict[str, PackageModel] = field(factory=dict)
    files: Dict[str, FileModel] = field(factory=dict)
    nodes: Dict[str, NodeModel] = field(factory=dict)
    # NOTE: still not sure whether to include storage here
    # storage: Dict[str, StorageMetadata] = field(factory=dict)

    def asdict(self) -> Dict[str, Any]:
        return asdict(self)


###############################################################################
# Convenience Functions
###############################################################################


def uid_file(package: str, relative_path: str) -> str:
    return f'{package}/{relative_path}'
