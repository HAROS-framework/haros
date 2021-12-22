# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

from haros.metamodel.common import SourceCodeMetadata, StorageMetadata

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
    assert s.language == 'Text'
    assert s.lines == 1
    assert s.ast is None


def test_source_metadata_with_args():
    s = SourceCodeMetadata(language='C++', lines=100, ast=None)
    assert s.language == 'C++'
    assert s.lines == 100
    assert s.ast is None
