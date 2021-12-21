# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

import haros.metamodel.ros as metamodel

###############################################################################
# Tests
###############################################################################


def test_ros_pkg_creation():
    obj = metamodel.RosPackage('uid', 'name')
    assert obj.uid == 'uid'
    assert obj.name == 'name'
    assert obj.files == []


def test_ros_pkg_requires_args():
    try:
        metamodel.RosPackage()
        raise AssertionError('expected constructor to require arguments')
    except TypeError:
        pass
