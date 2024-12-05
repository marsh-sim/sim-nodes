"""
Collection of utilities useful for multiple nodes
"""

from argparse import ArgumentDefaultsHelpFormatter, ArgumentTypeError, RawDescriptionHelpFormatter
from typing import Any


class NodeFormatter(ArgumentDefaultsHelpFormatter, RawDescriptionHelpFormatter):
    """Subclassed from `argparse` formatters to use moultiple features in `formatter_class`"""
    pass

def check_number(t: Any, value: str, allow_zero: bool = False, allow_negative: bool = False):
    """
    Helper function to check numbers passed on the command line.

    Pass in `add_argument` for `type` using a lambda to create a converter from string:
    ```
    add_argument(..., type=lambda s: check_number(int, s)) # non-negative int
    ```
    """
    try:
        number = t(value)
        if not allow_negative:
            if allow_zero:
                if number <= 0:
                    raise ArgumentTypeError(f"{number} is not a positive number")
            else:
                if number < 0:
                    raise ArgumentTypeError(f"{number} is not a non-negative number")
    except ValueError:
        raise ArgumentTypeError('string "{}" could not be converted to {}'.format(value, t.__name__))
    return number
