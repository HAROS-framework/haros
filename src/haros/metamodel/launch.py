# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, Optional, Tuple

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
class LaunchEntity:
    @property
    def is_configuration(self) -> bool:
        return False

    @property
    def is_argument(self) -> bool:
        return False

    @property
    def is_inclusion(self) -> bool:
        return False

    @property
    def is_node(self) -> bool:
        return False


@frozen
class LaunchConfiguration(LaunchEntity):
    name: str
    default_value: Optional[LaunchValue] = None

    @property
    def is_configuration(self) -> bool:
        return True


@frozen
class LaunchArgument(LaunchEntity):
    name: str
    default_value: LaunchValue = UNKNOWN
    description: LaunchValue = UNKNOWN

    @property
    def is_argument(self) -> bool:
        return True

    def to_xml(self) -> str:
        return f'<arg name="{self.name}" default="{self.value}"/>'


@frozen
class LaunchInclusion(LaunchEntity):
    file: LaunchValue = UNKNOWN
    namespace: RosName = field(factory=RosName.global_namespace)
    arguments: Dict[str, LaunchValue] = field(factory=dict)

    @property
    def is_inclusion(self) -> bool:
        return True


@frozen
class LaunchNode(LaunchEntity):
    name: Optional[LaunchValue] = None
    package: LaunchValue = UNKNOWN
    executable: LaunchValue = UNKNOWN
    namespace: RosName = field(factory=RosName.global_namespace)
    parameters: Dict[str, LaunchValue] = field(factory=dict)
    remaps: Dict[RosName, RosName] = field(factory=dict)
    output: LaunchValue = UNKNOWN
    arguments: Iterable[LaunchValue] = field(factory=tuple)

    @property
    def is_node(self) -> bool:
        return True


@frozen
class LaunchDescription:
    entities: Tuple[LaunchEntity] = ()


@frozen
class LaunchModel:
    name: str
    nodes: List[LaunchNode] = field(factory=list)
    builder: Any = None  # FIXME
