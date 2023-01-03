# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, List, Optional

from attrs import field, frozen

from haros.metamodel.ros import RosName

###############################################################################
# Constants
###############################################################################


###############################################################################
# ROS Launch Entities
###############################################################################


@frozen
class LaunchValue:
    @property
    def is_resolved(self) -> bool:
        return False

    def __str__(self) -> str:
        return '{?}'


@frozen
class TextSubstitution(LaunchValue):
    value: str

    @property
    def is_resolved(self) -> bool:
        return True

    def __str__(self) -> str:
        return self.value


UNKNOWN: Final[LaunchValue] = LaunchValue()


@frozen
class LaunchConfiguration:
    name: str
    default_value: Optional[LaunchValue] = None


@frozen
class LaunchArgument:
    name: str
    default_value: LaunchValue = UNKNOWN

    def to_xml(self) -> str:
        return f'<arg name="{self.name}" default="{self.value}"/>'


@frozen
class LaunchInclusion:
    file: LaunchValue = UNKNOWN
    namespace: RosName = field(factory=RosName.global_namespace)


@frozen
class LaunchNode:
    name: LaunchValue = UNKNOWN
    package: LaunchValue = UNKNOWN
    executable: LaunchValue = UNKNOWN
    namespace: RosName = field(factory=RosName.global_namespace)
    parameters: Dict[str, LaunchValue] = field(factory=dict)
    remaps: Dict[RosName, RosName] = field(factory=dict)


@frozen
class LaunchModel:
    name: str
    nodes: List[LaunchNode] = field(factory=list)
    builder: Any = None  # FIXME
