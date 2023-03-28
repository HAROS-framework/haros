# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, Mapping, Optional

from enum import auto, Enum, Flag, unique

from attrs import asdict, field, frozen
from attrs.validators import matches_re

from haros.metamodel.common import (
    DevelopmentMetadata,
    K,
    Resolved,
    Result,
    SourceCodeDependencies,
    SourceCodeMetadata,
    TrackedCode,
    UnresolvedIterable,
    UnresolvedMapping,
    V,
)

###############################################################################
# Constants
###############################################################################


RE_NAME: Final = r'(^[a-zA-Z][a-zA-Z0-9_]*$)'


@unique
class Languages(Enum):
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


class TypeMask(Flag):
    BOOL = auto()
    INT = auto()
    DOUBLE = auto()
    STRING = auto()
    LIST = auto()
    MAPPING = auto()
    NUMBER = INT | DOUBLE
    PRIMITIVE = BOOL | NUMBER | STRING
    ANY = PRIMITIVE | LIST | MAPPING

    @property
    def can_be_bool(self) -> bool:
        return bool(self & TypeMask.BOOL)

    @property
    def can_be_int(self) -> bool:
        return bool(self & TypeMask.INT)

    @property
    def can_be_double(self) -> bool:
        return bool(self & TypeMask.DOUBLE)

    @property
    def can_be_number(self) -> bool:
        return bool(self & TypeMask.NUMBER)

    @property
    def can_be_string(self) -> bool:
        return bool(self & TypeMask.STRING)

    @property
    def can_be_primitive(self) -> bool:
        return bool(self & TypeMask.PRIMITIVE)

    @property
    def can_be_list(self) -> bool:
        return bool(self & TypeMask.LIST)

    @property
    def can_be_mapping(self) -> bool:
        return bool(self & TypeMask.MAPPING)

    @property
    def can_have_items(self) -> bool:
        mask = TypeMask.STRING | TypeMask.LIST | TypeMask.MAPPING
        return bool(self & mask)


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


def maybe_str_to_rosname(name: Optional[str]) -> Optional[RosName]:
    return None if name is None else RosName(name)


@frozen
class RosClientAdvertiseCall(RosSourceEntity):
    # Parameters
    name: str
    namespace: str


@frozen
class RosClientSubscribeCall(RosSourceEntity):
    # Parameters
    name: str
    namespace: str


@frozen
class RosClientLibraryCalls:
    advertise: List[RosClientAdvertiseCall] = field(factory=list)
    subscribe: List[RosClientSubscribeCall] = field(factory=list)


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
    rosname: Optional[RosName] = field(default=None, converter=maybe_str_to_rosname)
    files: List[str] = field(factory=list)
    rcl_calls: RosClientLibraryCalls = field(factory=RosClientLibraryCalls)
    source: SourceCodeMetadata = field(factory=SourceCodeMetadata)
    dependencies: SourceCodeDependencies = field(factory=SourceCodeDependencies)

    @property
    def uid(self) -> str:
        return uid_node(self.package, self.name)


###############################################################################
# ROS Runtime Analysis
###############################################################################


def unknown_value(
    type: TypeMask = TypeMask.STRING,
    source: Optional[TrackedCode] = None,
) -> Result:
    return Result(source, type)

def const_string(value: str, source: Optional[TrackedCode] = None) -> Result[str]:
    # TODO validate values
    return Resolved(source, TypeMask.STRING, value)

def const_list(
    value: str,
    source: Optional[TrackedCode] = None,
) -> Result[Iterable[Result]]:
    # TODO validate values
    return Resolved(source, TypeMask.LIST, value)

def const_mapping(
    value: str,
    source: Optional[TrackedCode] = None,
) -> Result[Mapping[str, Result]]:
    # TODO validate values
    return Resolved(source, TypeMask.MAPPING, value)


###############################################################################
# ROS Runtime Objects
###############################################################################


@frozen
class RosNodeModel(RosRuntimeEntity):
    rosname: Result[RosName]
    node: Result[str]
    arguments: Result[List[str]] = field(factory=UnresolvedIterable.unknown_list)
    parameters: Result = field(factory=UnresolvedMapping.unknown_dict)
    remappings: Result = field(factory=UnresolvedMapping.unknown_dict)
    output: Result[str] = Resolved.from_string('log')


@frozen
class RosLinkModel(RosRuntimeEntity):
    pass


@frozen
class RosTopicModel(RosRuntimeEntity):
    pass


@frozen
class RosServiceModel(RosRuntimeEntity):
    pass


@frozen
class RosActionModel(RosRuntimeEntity):
    pass


@frozen
class RosParameterModel(RosRuntimeEntity):
    pass


@frozen
class ComputationGraphModel(RosRuntimeEntity):
    nodes: List[RosNodeModel] = field(factory=list)


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

def uid_node(package: str, executable: str) -> str:
    return f'{package}/{executable}'
