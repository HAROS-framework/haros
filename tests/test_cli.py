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


def test_invalid_cmd():
    with pytest.raises(SystemExit) as pytest_wrapped_e:
        cli.main(['dummy', 'x', 'y', 'z'])
    assert pytest_wrapped_e.type == SystemExit
    assert pytest_wrapped_e.value.code == 2


def test_echo_args():
    cli.main(['echo-args', 'x', 'y', 'z'])


def test_parse_echo_args():
    args = cli.parse_arguments(['echo-args', 'x', 'y', 'z'])
    assert isinstance(args, dict)
    assert args.get('cmd') == 'echo-args'
    assert args.get('args') == ['x', 'y', 'z']


def test_parse_echo_args_no_args():
    args = cli.parse_arguments(['echo-args'])
    assert isinstance(args, dict)
    assert args.get('cmd') == 'echo-args'
    assert args.get('args') == []
