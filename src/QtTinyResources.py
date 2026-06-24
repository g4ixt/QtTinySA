# -*- coding: utf-8 -*-

"""Resource helper functions."""

import os
import sys


def resource_path(filename: str) -> str:
    """Get resources path in a safe way to work on terminal AND in macOS app bundles."""
    if getattr(sys, 'frozen', False):
        base = sys._MEIPASS
    else:
        base = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base, filename)
