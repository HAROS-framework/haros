# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from attr.exceptions import FrozenInstanceError

from haros.metamodel.ros import File, Package

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
# Package Tests
###############################################################################


def test_pkg_slots():
    assert 'uid' in Package.__slots__
    assert 'name' in Package.__slots__
    assert 'is_metapackage' in Package.__slots__
    assert 'files' in Package.__slots__
    assert 'nodes' in Package.__slots__
    assert 'storage' in Package.__slots__
    assert 'metadata' in Package.__slots__


def test_pkg_creation():
    p = Package('name')
    assert p.uid == 'pkg:name'
    assert p.name == 'name'
    assert p.is_metapackage is False
    assert p.files == []
    assert p.nodes == []
    assert p.storage.path is None
    assert p.storage.size is None
    assert p.storage.timestamp == 0
    assert p.metadata.description == ''
    assert p.metadata.authors == set()
    assert p.metadata.maintainers == set()
    assert p.metadata.version == 'unknown'
    assert p.metadata.license == 'unknown'
    assert p.metadata.url_home is None
    assert p.metadata.url_source is None
    assert p.metadata.url_tracker is None


def test_pkg_requires_args():
    try:
        Package()
        raise AssertionError('constructor should expect arguments')
    except TypeError:
        pass


def test_two_pkgs_equal():
    p1 = Package('pkg')
    p2 = Package('pkg')
    assert p1 == p2
    assert p1 is not p2
    assert p1.uid == p2.uid
    assert p1.name == p2.name
    assert p1.is_metapackage == p2.is_metapackage
    assert p1.files == p2.files
    assert p1.nodes == p2.nodes
    assert p1.storage == p2.storage
    assert p1.metadata == p2.metadata


def test_two_pkgs_not_equal():
    p1 = Package('pkg1')
    p2 = Package('pkg2')
    assert p1 != p2
    assert p1.uid != p2.uid
    assert p1.name != p2.name
    assert p1.is_metapackage == p2.is_metapackage
    assert p1.files == p2.files
    assert p1.nodes == p2.nodes


def test_pkg_change_frozen():
    p = Package('pkg')
    try:
        p.name = 'other'
        raise AssertionError('frozen instance should be immutable')
    except FrozenInstanceError:
        pass


def test_pkg_change_metadata():
    p = Package('pkg')
    p.storage.path = '/etc/pkg'
    assert p.storage.path == '/etc/pkg'
    p.storage.size = 1
    assert p.storage.size == 1
    p.storage.timestamp = 100
    assert p.storage.timestamp == 100
    p.metadata.description = 'README'
    assert p.metadata.description == 'README'
    p.metadata.authors.add('Joe')
    assert p.metadata.authors == {'Joe'}
    p.metadata.maintainers.add('Joe')
    assert p.metadata.maintainers == {'Joe'}
    p.metadata.version = '1.0.0'
    assert p.metadata.version == '1.0.0'
    p.metadata.license = 'MIT'
    assert p.metadata.license == 'MIT'
    p.metadata.url_home = 'https://example.com'
    assert p.metadata.url_home == 'https://example.com'
    p.metadata.url_source = 'https://github.com/ros/pkg'
    assert p.metadata.url_source == 'https://github.com/ros/pkg'
    p.metadata.url_tracker = 'https://github.com/ros/pkg/issues'
    assert p.metadata.url_tracker == 'https://github.com/ros/pkg/issues'


def test_pkg_json():
    data = Package('pkg').asdict()
    assert data['uid'] == 'pkg:pkg'
    assert data['name'] == 'pkg'
    assert data['is_metapackage'] is False
    assert data['files'] == []
    assert data['nodes'] == []
    assert data['storage']['path'] is None
    assert data['storage']['size'] is None
    assert data['storage']['timestamp'] == 0
    assert data['metadata']['description'] == ''
    assert data['metadata']['authors'] == []
    assert data['metadata']['maintainers'] == []
    assert data['metadata']['version'] == 'unknown'
    assert data['metadata']['license'] == 'unknown'
    assert data['metadata']['url_home'] is None
    assert data['metadata']['url_source'] is None
    assert data['metadata']['url_tracker'] is None
