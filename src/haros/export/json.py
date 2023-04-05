# SPDX-License-Identifier: MIT
# Copyright © 2023 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Tuple, Type

from collections.abc import Iterable as IterableType, Mapping as MappingType
import json

from haros.metamodel.common import Result
from haros.metamodel.launch import LaunchModel

###############################################################################
# Constants
###############################################################################

JSON_PRIMITIVE_TYPES: Final[Tuple[Type]] = (bool, int, float, str)

###############################################################################
# ROS Launch Interface
###############################################################################


def launch_model(model: LaunchModel) -> Dict[str, Any]:
    return {
        'name': model.name,
        'nodes': list(map(ros_node_model, model.nodes))
    }


def ros_node_model(model: RosNodeModel) -> Dict[str, Any]:
    return {
        'rosname': str(model.rosname),
        'node': str(model.node),
        'arguments': _result(model.arguments),
        'parameters': _result(model.parameters),
        'remappings': _result(model.remappings),
        'output': _result(model.output),
    }


###############################################################################
# Helper Functions
###############################################################################


def _result(result: Result[Any]) -> Dict[str, Any]:
    data = {
        'type': str(result.type),
        'source': _tracked_code(result.source)
    }
    if result.is_resolved:
        data['value'] = _serialize(result.value)
    return data


def _tracked_code(code: Optional[TrackedCode]) -> Optional[Dict[str, Any]]:
    if code is None:
        return None
    return code.location.serialize()


def _serialize(value: Any) -> Any:
    if value is None or isinstance(value, JSON_PRIMITIVE_TYPES):
        return value
    if isinstance(value, IterableType):
        return list(map(_serialize, value))
    if isinstance(value, MappingType):
        return {str(_serialize(k)): _serialize(v) for k, v in value.items()}
    if hasattr(value, 'serialize'):
        return value.serialize()
    return str(value)
