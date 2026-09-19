# -*- coding: utf-8 -*-

"""Chip description lookup.

Chip descriptions live inside the package (``xuanwu/data/chips``) so that an
installed xuanwu can find them.  Users may either pass a path or simply a chip
name::

    XuanWu("stm32f411", "firmware.elf")
    XuanWu("chip/arm/cortex_m/stm32f411.yaml", "firmware.elf")

A description may build on another one with ``include:``, which is how a board with a
shield on it stays a handful of lines instead of a copy of the whole part::

    chip:
      name: sam3x8e_tft
      include: sam3x8e.yaml
      devices:
        - {name: LCD, type: ili9341, ...}
"""

import os
from os import path
from typing import Any, Dict, List

import yaml

from .config import RESOURCE
from .exception import XwInvalidChipInformation

__all__ = ["chip_dir", "list_chips", "resolve_chip", "describe_chip_error", "load_chip_document"]

# Keys whose lists are concatenated when a description includes another one.
LIST_KEYS = ("peripherals", "devices")


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


def load_chip_document(chip: str) -> Dict[str, Any]:
    """Read a chip description, following its ``include:`` chain.

    Returns the document with the ``chip`` root key, ready to hand to
    :class:`~xuanwu.xuanwu.XuanWu`.  Scalars and lists of the including file win over
    the included one, except for ``peripherals`` and ``devices``, which are
    concatenated: a board adds to a part, it does not replace it.
    """
    return _load_document(chip, [])


def _load_document(chip: str, seen: List[str]) -> Dict[str, Any]:
    resolved = resolve_chip(chip)
    if not path.isfile(resolved):
        raise XwInvalidChipInformation(describe_chip_error(chip))
    with open(resolved, encoding="utf-8") as handle:
        doc = yaml.safe_load(handle) or {}
    if "chip" not in doc:
        raise XwInvalidChipInformation(f"Invalid chip information file: {resolved}")
    own = dict(doc["chip"])
    include = own.pop("include", None)
    if not include:
        return {"chip": own}

    target = include
    if not path.isabs(target):
        beside = path.join(path.dirname(resolved), include)
        target = beside if path.isfile(beside) else include
    if target in seen or resolved in seen:
        raise XwInvalidChipInformation(f"Chip description include cycle: {resolved} -> {include}")
    base = _load_document(target, seen + [resolved])["chip"]

    merged = dict(base)
    merged.update(own)
    for key in LIST_KEYS:
        combined = list(base.get(key) or []) + list(own.get(key) or [])
        if combined:
            merged[key] = combined
    return {"chip": merged}
