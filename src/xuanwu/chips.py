# -*- coding: utf-8 -*-

"""Chip description lookup.

Chip descriptions live inside the package (``xuanwu/data/chips``) so that an
installed xuanwu can find them.  Users may either pass a path or simply a chip
name::

    XuanWu("stm32f411", "firmware.elf")
    XuanWu("chip/arm/cortex_m/stm32f411.yaml", "firmware.elf")
"""

import os
from os import path
from typing import List, Optional

from .config import RESOURCE


__all__ = ["chip_dir", "list_chips", "resolve_chip", "describe_chip_error"]


def chip_dir() -> str:
    """Root directory holding the bundled chip descriptions."""
    return RESOURCE["chip"]


def list_chips() -> List[str]:
    """Names of every bundled chip description, without the ``.yaml`` suffix."""
    root = chip_dir()
    names = []
    if not path.isdir(root):
        return names
    for dirpath, _dirnames, filenames in os.walk(root):
        for filename in filenames:
            if filename.endswith(".yaml"):
                names.append(filename[: -len(".yaml")])
    return sorted(names)


def resolve_chip(chip: str) -> str:
    """Return a readable path for ``chip``, which may be a path or a chip name.

    Returns the input unchanged when nothing matches, so the caller can raise a
    helpful error mentioning what was requested.
    """
    if path.isfile(chip):
        return chip

    stem = chip[: -len(".yaml")] if chip.endswith(".yaml") else chip
    root = chip_dir()
    if path.isdir(root):
        for dirpath, _dirnames, filenames in os.walk(root):
            for filename in (stem + ".yaml", chip):
                if filename in filenames:
                    return path.join(dirpath, filename)
    return chip


def describe_chip_error(chip: str) -> str:
    """Error text listing the available chip names."""
    available = list_chips()
    return f"Unknown chip {chip!r}. Looked for a file and for a bundled chip name; available: {', '.join(available) or 'none'}"
