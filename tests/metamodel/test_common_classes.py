# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from haros.metamodel.common import (
    DevelopmentMetadata,
    SourceCodeDependencies,
    SourceCodeMetadata,
    StorageMetadata,
)

###############################################################################
# StorageMetadata Tests
###############################################################################


def test_storage_metadata_slots():
    assert 'path' in StorageMetadata.__slots__
    assert 'size' in StorageMetadata.__slots__
    assert 'timestamp' in StorageMetadata.__slots__


def test_storage_metadata_creation():
    s = StorageMetadata()
    assert s.path is None
    assert s.size is None
    assert s.timestamp == 0


def test_storage_metadata_with_args():
    s = StorageMetadata(path='/etc/file', size=1, timestamp=1000)
    assert s.path == '/etc/file'
    assert s.size == 1
    assert s.timestamp == 1000


###############################################################################
# SourceCodeMetadata Tests
###############################################################################


def test_source_metadata_slots():
    assert 'language' in SourceCodeMetadata.__slots__
    assert 'lines' in SourceCodeMetadata.__slots__
    assert 'ast' in SourceCodeMetadata.__slots__


def test_source_metadata_creation():
    s = SourceCodeMetadata()
    assert s.language == 'Unknown'
    assert s.lines == 1
    assert s.ast is None


def test_source_metadata_with_args():
    s = SourceCodeMetadata(language='C++', lines=100, ast=None)
    assert s.language == 'C++'
    assert s.lines == 100
    assert s.ast is None


###############################################################################
# DevelopmentMetadata Tests
###############################################################################


def test_dev_metadata_slots():
    assert 'description' in DevelopmentMetadata.__slots__
    assert 'authors' in DevelopmentMetadata.__slots__
    assert 'maintainers' in DevelopmentMetadata.__slots__
    assert 'version' in DevelopmentMetadata.__slots__
    assert 'license' in DevelopmentMetadata.__slots__
    assert 'url_home' in DevelopmentMetadata.__slots__
    assert 'url_source' in DevelopmentMetadata.__slots__
    assert 'url_tracker' in DevelopmentMetadata.__slots__


def test_dev_metadata_creation():
    m = DevelopmentMetadata()
    assert m.description == ''
    assert m.authors == set()
    assert m.maintainers == set()
    assert m.version == 'unknown'
    assert m.license == 'unknown'
    assert m.url_home is None
    assert m.url_source is None
    assert m.url_tracker is None


def test_dev_metadata_with_args():
    m = DevelopmentMetadata(
        description='description',
        authors={'Joe'},
        maintainers={'Moe'},
        version='1.0',
        license='MIT',
        url_home='example.com',
        url_source='github.com/a/b',
        url_tracker='github.com/a/b/issues',
    )
    assert m.description == 'description'
    assert m.authors == {'Joe'}
    assert m.maintainers == {'Moe'}
    assert m.version == '1.0'
    assert m.license == 'MIT'
    assert m.url_home == 'example.com'
    assert m.url_source == 'github.com/a/b'
    assert m.url_tracker == 'github.com/a/b/issues'


###############################################################################
# SourceCodeDependencies Tests
###############################################################################


def test_source_deps_slots():
    assert 'build' in SourceCodeDependencies.__slots__
    assert 'runtime' in SourceCodeDependencies.__slots__


def test_source_deps_creation():
    s = SourceCodeDependencies()
    assert s.build == set()
    assert s.runtime == set()


def test_source_deps_with_args():
    s = SourceCodeDependencies(build={'x'}, runtime={'y'})
    assert s.build == {'x'}
    assert s.runtime == {'y'}
