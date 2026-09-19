# -*- coding: utf-8 -*-

import os
import logging
import logging.config
from pathlib import Path

from .default import *
from .constant import *
from .default import LOGGING_CONFIG  # imported by name: the star import hides typos

DEBUG = os.environ.get("DEBUG", "0") == "1"
"""Debug mode switch."""

if DEBUG:
    try:
        from . import debug_override

        override_items = [
            "LOGGING_CONFIG",
        ]
        lcl = locals()
        for item in override_items:
            if hasattr(debug_override, item):
                lcl[item].update(getattr(debug_override, item))
    except ImportError:
        print("No debug override")


def _log_file_path() -> Path:
    """Return the per-user log file path.

    Deliberately *not* relative to the current working directory: importing a
    library must never create files in ``os.getcwd()`` (a read-only or
    foreign-owned cwd would make ``import xuanwu`` fail outright).
    """
    base = os.environ.get("XDG_STATE_HOME") or os.path.join(os.path.expanduser("~"), ".local", "state")
    return Path(base) / "xuanwu" / "xuanwu.log"


def _configure_logging() -> None:
    logfile = LOGGING_CONFIG["handlers"].get("logfile")
    if not DEBUG:
        # Library default: console at INFO, no file handler, no filesystem writes.
        LOGGING_CONFIG["handlers"].pop("logfile", None)
    elif logfile is not None:
        filename = _log_file_path()
        try:
            filename.parent.mkdir(parents=True, exist_ok=True)
            filename.touch(exist_ok=True)
        except OSError:
            # Home is not writable either -- keep going with console only.
            LOGGING_CONFIG["handlers"].pop("logfile", None)
        else:
            logfile["filename"] = str(filename)
    logging.config.dictConfig(LOGGING_CONFIG)


_configure_logging()
logger = logging.getLogger("xuanwu")
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)
