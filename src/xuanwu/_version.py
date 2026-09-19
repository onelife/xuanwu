# -*- coding: utf-8 -*-

"""Single source of truth for package metadata.

Kept dependency-free on purpose: ``setup.py`` reads this file as text so that
building the package never has to import :mod:`xuanwu` (and therefore never has
to import unicorn/capstone).
"""

__author__ = "onelife"
__license__ = "LGPL-2.1"
__version__ = "0.0.1"
