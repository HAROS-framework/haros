# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Dict, Final, List, Optional

import logging

from attrs import define, field, frozen

from haros.metamodel.launch import (
    LaunchArgument,
    LaunchDescription,
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
class AnalysisSystemInterface:
    packages: Dict[str, str] = field(factory=dict)


@frozen
class LaunchScope:
    args: Dict[str, LaunchValue] = field(factory=dict)
    configs: Dict[str, LaunchValue] = field(factory=dict)

    def get(self, name: str) -> LaunchValue:
        value = self.configs.get(name, self.args.get(name))
        if value is None:
            return LaunchValue()  # FIXME maybe raise error
        return value

    def set(self, name: str, value: LaunchValue):
        # if name not in self.configs:
        self.configs[name] = self.resolve(value)

    def set_unknown(self, name: str):
        return self.set(name, LaunchValue())

    def set_text(self, name: str, text: str):
        return self.set(name, TextSubstitution(text))

    def resolve(self, value: LaunchValue) -> LaunchValue:
        if value.is_configuration:
            new_value = self.get(value.name)
            if new_value.is_resolved:
                return new_value
        return value

    def duplicate(self) -> 'LaunchScope':
        # LaunchArgument is defined globally
        # LaunchConfiguration is scoped
        return LaunchScope(args=self.args, configs=dict(self.configs))


@define
class LaunchModelBuilder:
    name: str
    system: AnalysisSystemInterface = field(factory=AnalysisSystemInterface)
    nodes: List[LaunchNode] = field(factory=list)
    _scope_stack: List[LaunchScope] = field(factory=lambda: [LaunchScope()])

    @property
    def scope(self) -> LaunchScope:
        return self._scope_stack[-1]

    @property
    def root(self) -> LaunchScope:
        return self._scope_stack[0]

    def build(self) -> LaunchModel:
        return LaunchModel(self.name, nodes=list(self.nodes))

    def enter_group(self):
        self._scope_stack.append(self.scope.duplicate())

    def exit_group(self):
        self._scope_stack.pop()

    def declare_argument(self, name: str, default_value: LaunchValue):
        # NOTE: probably better to separate LaunchValue from LaunchSubstitution...
        self.root.args[name] = self.scope.resolve(default_value)


def model_from_description(name: str, description: LaunchDescription) -> LaunchModel:
    builder = LaunchModelBuilder(name)
    for entity in description.entities:
        if entity.is_configuration:
            pass
        elif entity.is_argument:
            builder.declare_argument(entity.name, entity.default_value)
        elif entity.is_inclusion:
            pass
        elif entity.is_node:
            pass
    return builder.build()
