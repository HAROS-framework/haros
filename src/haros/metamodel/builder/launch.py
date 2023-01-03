# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Dict, Final, List

import logging

from attrs import define, field, frozen

from haros.metamodel.launch import (
    LaunchArgument,
    LaunchInclusion,
    LaunchModel,
    LaunchNode,
    LaunchValue,
    TextSubstitution,
)
from haros.metamodel.ros import RosName

###############################################################################
# Constants
###############################################################################

logger: Final[logging.Logger] = logging.getLogger(__name__)

###############################################################################
# Interface
###############################################################################


@frozen
class LaunchScope:
    args: Dict[str, LaunchValue] = field(factory=dict)

    def get(self, name: str) -> LaunchValue:
        value = self.args.get(name)
        if value is None:
            return LaunchValue()  # FIXME maybe raise error
        return value

    def set(self, name: str, value: LaunchValue):
        # FIXME raise error for duplicate names
        if name not in self.args:
            self.args[name] = value

    def set_unknown(self, name: str):
        return self.set(name, LaunchValue())

    def set_text(self, name: str, text: str):
        return self.set(name, TextSubstitution(text))

    def duplicate(self) -> 'LaunchScope':
        return LaunchScope(args=dict(self.args))


@define
class LaunchModelBuilder:
    name: str
    nodes: List[LaunchNode] = field(factory=list)
    _scope_stack: List[LaunchScope] = field(factory=lambda: [LaunchScope()])

    @property
    def scope(self) -> LaunchScope:
        return self._scope_stack[-1]

    def build(self) -> LaunchModel:
        return LaunchModel(self.name, nodes=list(self.nodes))

    def enter_group(self):
        self._scope_stack.append(self.scope.duplicate())

    def exit_group(self):
        self._scope_stack.pop()
