# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

import typing

import enum

import attr

###############################################################################
# Constants
###############################################################################


@enum.unique
class Languages(enum.Enum):
    XML = 'XML'
    YAML = 'YAML'
    JSON = 'JSON'
    CPP = 'C++'
    PYTHON = 'Python'
    CMAKE = 'CMake'
    ROSPKG = 'ROS Package XML'
    ROSLAUNCH = 'ROS Launch XML'
    ROSMSG = 'ROS Message'
    ROSSRV = 'ROS Service'
    ROSACTION = 'ROS Action'


###############################################################################
# Metamodel Classes
###############################################################################


@attr.s(auto_attribs=True, slots=True)
class RosPackage:
    uid: str
    name: str
    files: typing.List[str] = attr.Factory(list)
