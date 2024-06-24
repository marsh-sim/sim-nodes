"""
Collection of utilities useful for multiple nodes
"""

from argparse import ArgumentDefaultsHelpFormatter, RawDescriptionHelpFormatter


class NodeFormatter(ArgumentDefaultsHelpFormatter, RawDescriptionHelpFormatter):
    """Subclassed from `argparse` formatters to use moultiple features in `formatter_class`"""
    pass
