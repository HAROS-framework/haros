# SPDX-License-Identifier: MIT
# Copyright © 2023 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Optional, Tuple, Type

from collections.abc import Iterable as IterableType, Mapping as MappingType
import json

from haros.metamodel.common import (
    DevelopmentMetadata,
    Result,
    SourceCodeDependencies,
    SourceCodeMetadata,
    TrackedCode,
)
from haros.metamodel.launch import LaunchModel
from haros.metamodel.ros import (
    FileModel,
    NodeModel,
    PackageModel,
    ProjectModel,
    RosClientLibraryCalls,
    RosName,
    RosNodeModel,
)

###############################################################################
# Constants
###############################################################################

JSON_PRIMITIVE_TYPES: Final[Tuple[Type]] = (bool, int, float, str)

###############################################################################
# Project Interface
###############################################################################


def export_project(model: ProjectModel) -> Dict[str, Any]:
    return {
        'name': model.name,
        'packages': {k: export_package(v) for k, v in model.packages.items()},
        'files': {k: export_file(v) for k, v in model.files.items()},
        'nodes': {k: export_node(v) for k, v in model.nodes.items()},
    }


def export_package(model: PackageModel) -> Dict[str, Any]:
    return {
        'name': model.name,
        'files': list(model.files),
        'nodes': list(model.nodes),
        'metadata': _dev_metadata(model.metadata),
        'dependencies': model.dependencies.asdict(),
    }


def export_node(model: NodeModel) -> Dict[str, Any]:
    return {
        'package': model.package,
        'name': model.name,
        'is_library': model.is_library,
        'rosname': _rosname(model.rosname),
        'files': list(model.files),
        'rcl_calls': _rcl_calls(model.rcl_calls),
        'source': _source_metadata(model.source),
        'dependencies': model.dependencies.asdict(),
    }


def export_file(model: FileModel) -> Dict[str, Any]:
    return {
        'package': model.package,
        'path': model.path,
        'source': _source_metadata(model.source),
        'dependencies': model.dependencies.asdict(),
    }


###############################################################################
# ROS Launch Interface
###############################################################################


def export_launch_model(model: LaunchModel) -> Dict[str, Any]:
    return {
        'name': model.name,
        'nodes': list(map(ros_node_model, model.nodes))
    }


def ros_node_model(model: RosNodeModel) -> Dict[str, Any]:
    return {
        'rosname': str(model.rosname),
        'node': str(model.node),
        'arguments': serialize(model.arguments),
        'parameters': serialize(model.parameters),
        'remappings': serialize(model.remappings),
        'output': serialize(model.output),
    }


###############################################################################
# Helper Functions
###############################################################################


def serialize(value: Any) -> Any:
    if value is None or isinstance(value, JSON_PRIMITIVE_TYPES):
        return value
    if isinstance(value, Result):
        return _result(value)
    if isinstance(value, MappingType):
        return {str(serialize(k)): serialize(v) for k, v in value.items()}
    if isinstance(value, IterableType):
        return list(map(serialize, value))
    if hasattr(value, 'serialize'):
        return value.serialize()
    return str(value)


def _result(result: Result[Any]) -> Dict[str, Any]:
    data = {
        'type': str(result.type),
        'source': _tracked_code(result.source)
    }
    if result.is_resolved:
        data['value'] = serialize(result.value)
    return data


def _tracked_code(code: Optional[TrackedCode]) -> Optional[Dict[str, Any]]:
    return None if code is None else code.location.serialize()


def _rosname(rosname: Optional[RosName]) -> Optional[str]:
    return None if rosname is None else str(rosname)


def _rcl_calls(calls: RosClientLibraryCalls) -> Dict[str, Any]:
    return calls.asdict()


def _source_metadata(data: SourceCodeMetadata) -> Dict[str, Any]:
    return {
        'language': getattr(data.language, 'value', str(data.language)),
        'lines': data.lines,
        'ast': None if data.ast is None else serialize(data.ast),
    }


def _dev_metadata(data: DevelopmentMetadata) -> Dict[str, Any]:
    return {
        'description': data.description,
        'authors': list(data.authors),
        'maintainers': list(data.maintainers),
        'version': data.version,
        'license': data.license,
        'url_home': data.url_home,
        'url_source': data.url_source,
        'url_tracker': data.url_tracker,
    }
