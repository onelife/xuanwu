# -*- coding: utf-8 -*-

import sys
from os import path

__author__ = "onelife"
__license__ = "LGPL-2.1"
__version__ = "0.0.1"

__setup = False
__depth = 1

while True:
    try:
        stack = sys._getframe(__depth)
        if stack.f_globals.get("__file__"):
            path.basename(stack.f_globals.get("__file__"))
        __depth += 1
    except ValueError:
        break

__setup = stack.f_globals.get("__file__") and (path.basename(stack.f_globals.get("__file__")) == "setup.py")

if not __setup:
    sys.path.append(path.dirname(__file__))
    from .xuanwu import *
