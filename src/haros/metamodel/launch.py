# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, Optional, Tuple

from attrs import field, frozen

from haros.metamodel.common import TrackedCode
from haros.metamodel.ros import RosName

###############################################################################
# Constants
###############################################################################


###############################################################################
# ROS Launch Substitutions
###############################################################################


@frozen
class LaunchSubstitution:
    @property
    def is_unknown(self) -> bool:
        return False

    @property
    def is_text(self) -> bool:
        return False

    @property
    def is_configuration(self) -> bool:
        return False

    def __str__(self) -> str:
        return '$(?)'


@frozen
class UnknownSubstitution(LaunchSubstitution):
    source: TrackedCode

    @property
    def is_unknown(self) -> bool:
        return True


UNKNOWN_SUBSTITUTION: Final[UnknownSubstitution] = UnknownSubstitution(TrackedCode.unknown())


@frozen
class TextSubstitution(LaunchSubstitution):
    value: str

    @property
    def is_text(self) -> bool:
        return True

    def __str__(self) -> str:
        return self.value


@frozen
class LaunchConfiguration(LaunchSubstitution):
    name: str
    default_value: Optional[LaunchSubstitution] = None

    @property
    def is_configuration(self) -> bool:
        return True

    def __str__(self) -> str:
        return f'$(var {self.name} {self.default_value})'


###############################################################################
# ROS Launch Entities
###############################################################################


@frozen
class LaunchEntity:
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
class LaunchArgument(LaunchEntity):
    name: str
    default_value: Optional[LaunchSubstitution] = None
    description: Optional[LaunchSubstitution] = None

    @property
    def is_argument(self) -> bool:
        return True

    def to_xml(self) -> str:
        parts = [f'name="{self.name}"']
        if self.default_value is not None:
            parts.append(f'default="{self.default_value}"')
        if self.description is not None:
            parts.append(f'description="{self.description}"')
        return f'<arg {" ".join(parts)} />'


@frozen
class LaunchInclusion(LaunchEntity):
    file: LaunchSubstitution
    namespace: RosName = field(factory=RosName.global_namespace)
    arguments: Dict[str, LaunchSubstitution] = field(factory=dict)

    @property
    def is_inclusion(self) -> bool:
        return True


@frozen
class LaunchNode(LaunchEntity):
    name: Optional[LaunchSubstitution] = None
    package: LaunchSubstitution = UNKNOWN_SUBSTITUTION
    executable: LaunchSubstitution = UNKNOWN_SUBSTITUTION
    namespace: RosName = field(factory=RosName.global_namespace)
    parameters: Dict[str, LaunchSubstitution] = field(factory=dict)
    remaps: Dict[RosName, RosName] = field(factory=dict)
    output: LaunchSubstitution = UNKNOWN_SUBSTITUTION
    arguments: Iterable[LaunchSubstitution] = field(factory=tuple)

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
