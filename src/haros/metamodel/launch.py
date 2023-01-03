# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final

from attrs import field, frozen

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
class LaunchArgument:
    name: str
    default_value: LaunchValue = UNKNOWN

    def to_xml(self) -> str:
        return f'<arg name="{self.name}" default="{self.value}"/>'


@frozen
class LaunchInclusion:
    file: LaunchValue = UNKNOWN
    namespace: str = '/'


@frozen
class LaunchNode:
    name: LaunchValue = UNKNOWN
    package: LaunchValue = UNKNOWN
    executable: LaunchValue = UNKNOWN
    namespace: str = '/'
    parameters: Dict[str, LaunchValue] = field(factory=dict)
    remaps: Dict[str, str] = field(factory=dict)
