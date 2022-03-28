# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

###############################################################################
# Imports
###############################################################################

import pytest

from haros import cli

###############################################################################
# Tests
###############################################################################


def test_zero_arguments():
    with pytest.raises(SystemExit) as pytest_wrapped_e:
        cli.main([])
    assert pytest_wrapped_e.type == SystemExit
    assert pytest_wrapped_e.value.code == 2
