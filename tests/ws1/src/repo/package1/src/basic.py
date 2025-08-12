# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

import os

from mymodule import MY_CONSTANT, my_division  # type: ignore

###############################################################################
# Tests
###############################################################################


THE_X = 42
FILE_NAME = 'number'


def basic_math():
    a = THE_X
    b = a**2 // 64
    c = (a * b) / (a + b)
    if c > b:
        d = 1
    else:
        d = 2
    return a - d


def basic_if_control_flow():
    x = 1
    if MY_CONSTANT in (1, 2, 3):
        if MY_CONSTANT > x:
            y = 1
        elif MY_CONSTANT < x:
            y = 2
        else:
            y = 3
    elif MY_CONSTANT < 42:
        if MY_CONSTANT > 16:
            y = 4
    else:
        x = THE_X
        y = 5
    return x, y


def sum_args(a, b=0):
    return a + b


def basic_function_call():
    return sum_args(THE_X - 2, 2)


def two_function_calls():
    return max(sum_args(1, 1), 1)


def imported_function_call():
    return my_division(42 * 2, 2)


def file_io():
    param_file_name = FILE_NAME + '.txt'
    number_file = os.path.join(__file__, '..', '..', 'param', param_file_name)
    with open(number_file, 'r') as infp:
        number = infp.read()
        # number = int(infp.read())
    return number
    # params = {'number': number}
    # return params['number']
