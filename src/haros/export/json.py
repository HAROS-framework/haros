# SPDX-License-Identifier: MIT
# Copyright © 2023 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Final, Optional, Type

from collections.abc import Iterable, Mapping

from haros.metamodel.common import (
    DevelopmentMetadata,
    SourceCodeMetadata,
    TrackedCode,
)
from haros.metamodel.launch import (
    ArgumentFeature,
    LaunchDescription,
    LaunchFileFeature,
    LaunchModel,
    NodeFeature,
)
from haros.metamodel.result import Result
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

JSON_PRIMITIVE_TYPES: Final[tuple[Type]] = (bool, int, float, str)

###############################################################################
# Project Interface
###############################################################################


def export_project(model: ProjectModel) -> dict[str, Any]:
    return {
        'name': model.name,
        'packages': {k: export_package(v) for k, v in model.packages.items()},
        'files': {k: export_file(v) for k, v in model.files.items()},
        'nodes': {k: export_node(v) for k, v in model.nodes.items()},
    }


def export_package(model: PackageModel) -> dict[str, Any]:
    return {
        'name': model.name,
        'files': list(model.files),
        'nodes': list(model.nodes),
        'metadata': _dev_metadata(model.metadata),
        'dependencies': model.dependencies.asdict(),
    }


def export_node(model: NodeModel) -> dict[str, Any]:
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


def export_file(model: FileModel) -> dict[str, Any]:
    return {
        'package': model.package,
        'path': model.path,
        'source': _source_metadata(model.source),
        'dependencies': model.dependencies.asdict(),
    }


###############################################################################
# ROS Launch Interface
###############################################################################


def export_launch_description(launch: LaunchDescription) -> dict[str, Any]:
    return serialize(launch)


###############################################################################
# ROS Launch Interface
###############################################################################


def export_launch_model(model: LaunchModel) -> dict[str, Any]:
    return {'name': model.name, 'files': list(map(launch_feature, model.files))}


def launch_feature(model: LaunchFileFeature) -> dict[str, Any]:
    return {
        'id': model.id,
        'file': model.file,
        'arguments': {k: launch_argument_feature(v) for k, v in model.arguments.items()},
        'nodes': {k: launch_node_feature(v) for k, v in model.nodes.items()},
        'inclusions': list(model.inclusions),
        'conflicts': serialize(model.conflicts),
    }


def launch_argument_feature(model: ArgumentFeature) -> dict[str, Any]:
    return {
        'id': model.id,
        'name': model.name,
        'default_value': serialize(model.default_value),
        'description': serialize(model.description),
        'known_possible_values': serialize(model.known_possible_values),
        'inferred_type': model.inferred_type.value,
        # 'affects_cg': model.affects_cg,
        # decision_points: int = 0
    }


def launch_node_feature(model: NodeFeature) -> dict[str, Any]:
    return {
        'id': model.id,
        'rosnode': rosnode_model(model.rosnode),
    }


###############################################################################
# ROS Runtime
###############################################################################


def rosnode_model(model: RosNodeModel) -> dict[str, Any]:
    return {
        'rosname': serialize(model.rosname),
        'package': serialize(model.package),
        'executable': serialize(model.executable),
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
    if isinstance(value, RosName):
        return _rosname(value)
    if isinstance(value, Mapping):
        return {str(serialize(k)): serialize(v) for k, v in value.items()}
    if isinstance(value, Iterable):
        return list(map(serialize, value))
    if hasattr(value, 'serialize'):
        return serialize(value.serialize())
    return str(value)


def _result(result: Result[Any]) -> dict[str, Any]:
    data = {'type': str(result.type), 'source': _tracked_code(result.source)}
    if result.is_resolved:
        data['value'] = serialize(result.value)
    return data


def _tracked_code(code: Optional[TrackedCode]) -> Optional[dict[str, Any]]:
    return None if code is None else code.location.serialize()


def _rosname(rosname: Optional[RosName]) -> Optional[str]:
    return None if rosname is None else str(rosname)


def _rcl_calls(calls: RosClientLibraryCalls) -> dict[str, Any]:
    return calls.asdict()


def _source_metadata(data: SourceCodeMetadata) -> dict[str, Any]:
    return {
        'language': getattr(data.language, 'value', str(data.language)),
        'lines': data.lines,
        'ast': None if data.ast is None else serialize(data.ast),
    }


def _dev_metadata(data: DevelopmentMetadata) -> dict[str, Any]:
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
