# -*- coding: utf-8 -*-

"""Project config.

"""

from os import path, sep
from typing import Dict, Any


__all__ = ["BASE_DIR", "LOGGING_CONFIG"]


BASE_DIR = path.dirname(path.dirname(__file__))
"""Module root directory."""

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
            "filename": path.join(BASE_DIR, "xuanwu.log"),
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
