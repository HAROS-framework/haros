# SPDX-License-Identifier: MIT
# Copyright © 2023 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Optional, Tuple, Type

from collections.abc import Iterable as IterableType, Mapping as MappingType
import json

from haros.metamodel.common import Result, TrackedCode
from haros.metamodel.launch import LaunchModel
from haros.metamodel.ros import RosNodeModel

###############################################################################
# Constants
###############################################################################

JSON_PRIMITIVE_TYPES: Final[Tuple[Type]] = (bool, int, float, str)

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
