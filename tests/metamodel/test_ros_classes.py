# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from attr.exceptions import FrozenInstanceError

from haros.metamodel.ros import File, RosPackage

###############################################################################
# File Tests
###############################################################################


def test_file_slots():
    assert 'uid' in File.__slots__
    assert 'package' in File.__slots__
    assert 'path' in File.__slots__
    assert 'name' not in File.__slots__
    assert 'directory' not in File.__slots__
    assert 'storage' in File.__slots__
    assert 'source' in File.__slots__


def test_file_creation():
    f = File('pkg', 'file.txt')
    assert f.package == 'pkg'
    assert f.path == 'file.txt'
    assert f.storage.path is None
    assert f.storage.size is None
    assert f.storage.timestamp == 0
    assert f.source.language == 'Text'
    assert f.source.lines == 1
    assert f.source.ast is None
    assert f.uid == 'file:pkg/file.txt'
    assert f.name == 'file.txt'
    assert f.directory == '.'


def test_file_requires_args():
    try:
        File()
        raise AssertionError('constructor should expect arguments')
    except TypeError:
        pass


def test_two_files_equal():
    f1 = File('pkg', 'file.txt')
    f2 = File('pkg', 'file.txt')
    assert f1 == f2
    assert f1 is not f2
    assert f1.uid == f2.uid
    assert f1.package == f2.package
    assert f1.directory == f2.directory
    assert f1.name == f2.name


def test_two_files_not_equal():
    f1 = File('pkg', 'file1.txt')
    f2 = File('pkg', 'file2.txt')
    assert f1 != f2
    assert f1.uid != f2.uid
    assert f1.package == f2.package
    assert f1.directory == f2.directory
    assert f1.name != f2.name


def test_file_change_frozen():
    f = File('pkg', 'file.txt')
    try:
        f.package = 'other'
        raise AssertionError('frozen instance should be immutable')
    except FrozenInstanceError:
        pass
    try:
        f.path = 'other'
        raise AssertionError('frozen instance should be immutable')
    except FrozenInstanceError:
        pass
    try:
        f.storage = None
        raise AssertionError('frozen instance should be immutable')
    except FrozenInstanceError:
        pass
    try:
        f.source = None
        raise AssertionError('frozen instance should be immutable')
    except FrozenInstanceError:
        pass


def test_file_change_metadata():
    f = File('pkg', 'file.txt')
    f.storage.path = '/etc/file'
    assert f.storage.path == '/etc/file'
    f.storage.size = 1
    assert f.storage.size == 1
    f.storage.timestamp = 100
    assert f.storage.timestamp == 100
    f.source.language = 'C++'
    assert f.source.language == 'C++'
    f.source.lines = 100
    assert f.source.lines == 100
    f.source.ast = 'AST'
    assert f.source.ast == 'AST'


def test_file_json():
    data = File('pkg', 'file.txt').asdict()
    assert data['uid'] == 'file:pkg/file.txt'
    assert data['package'] == 'pkg'
    assert data['path'] == 'file.txt'
    assert data['storage']['path'] is None
    assert data['storage']['size'] is None
    assert data['storage']['timestamp'] == 0
    assert data['source']['language'] == 'Text'
    assert data['source']['lines'] == 1
    assert data['source']['ast'] is None
    assert 'name' not in data
    assert 'directory' not in data


###############################################################################
# RosPackage Tests
###############################################################################


def test_ros_pkg_creation():
    p = RosPackage('name')
    assert p.uid == 'pkg:name'
    assert p.name == 'name'
    assert p.files == []


def test_ros_pkg_requires_args():
    try:
        RosPackage()
        raise AssertionError('constructor should expect arguments')
    except TypeError:
        pass
