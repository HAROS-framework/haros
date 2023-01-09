# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from mymodule import MY_CONSTANT, my_division

###############################################################################
# Tests
###############################################################################


THE_X = 42


def basic_math():
    a = THE_X
    b = a ** 2 // 64
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