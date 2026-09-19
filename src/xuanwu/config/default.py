# -*- coding: utf-8 -*-

"""Project config.

"""

from os import path
from typing import Dict, Any


__all__ = ["BASE_DIR", "DATA_DIR", "RESOURCE", "LOGGING_CONFIG"]


BASE_DIR = path.dirname(path.dirname(path.abspath(__file__)))
"""Package root directory (``.../xuanwu``)."""

DATA_DIR = path.join(BASE_DIR, "data")
"""Package data root; shipped inside the wheel so installed copies work."""

RESOURCE = {
    "chip": path.join(DATA_DIR, "chips"),
    "gdb": path.join(DATA_DIR, "gdb", "features"),
}
"""Bundled chip descriptions and GDB target descriptions."""

LOGGING_CONFIG: Dict[str, Any] = {
    "version": 1,
    "formatters": {
        "console": {
            "()": "colorlog.ColoredFormatter",
            "format": "[%(log_color)s%(asctime)s %(levelname)-8s %(name)-8s %(module)-15s:%(lineno)6d] %(reset)s%(message)s",
            "datefmt": "%Y-%m-%d %H:%M:%S",
        },
        "file": {
            "format": "[%(asctime)s %(levelname)-8s %(name)-8s %(module)-15s:%(lineno)6d] %(message)s",
            "datefmt": "%Y-%m-%d %H:%M:%S",
        },
    },
    "handlers": {
        "console": {
            "level": "DEBUG",
            "class": "colorlog.StreamHandler",
            "formatter": "console",
        },
        "logfile": {
            "level": "DEBUG",
            "class": "logging.handlers.TimedRotatingFileHandler",
            # Placeholder only: config/__init__ drops this handler unless DEBUG=1
            # and otherwise points it at $XDG_STATE_HOME/xuanwu/. Never resolve it
            # relative to the current working directory or the (read-only)
            # package directory.
            "filename": "xuanwu.log",
            "when": "midnight",
            "backupCount": 30,
            "formatter": "file",
        },
    },
    "loggers": {
        "xuanwu": {
            "handlers": ["console"],
            "level": "INFO",
            "propagate": False,
        },
    },
    "root": {
        "handlers": ["console"],
        "level": "INFO",
    },
}
"""Log config."""
