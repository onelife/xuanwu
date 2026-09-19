# -*- coding: utf-8 -*-

import re
from os import path

from setuptools import setup, find_packages

HERE = path.abspath(path.dirname(__file__))


def read(*parts: str) -> str:
    with open(path.join(HERE, *parts), encoding="utf-8") as file:
        return file.read()


def find_version() -> str:
    """Read __version__ from src/xuanwu/_version.py without importing the package.

    Importing the package would pull in unicorn/capstone, which are not
    available in an isolated PEP 517 build environment.
    """
    match = re.search(r'^__version__\s*=\s*["\']([^"\']+)["\']', read("src", "xuanwu", "_version.py"), re.MULTILINE)
    if not match:
        raise RuntimeError("Unable to find __version__ in src/xuanwu/_version.py")
    return match.group(1)


install_requires = [
    line.strip() for line in read("requirements.txt").split("\n") if line.strip() and not line.strip().startswith("#")
]

# The chip descriptions and GDB target descriptions must travel with the wheel,
# otherwise an installed xuanwu cannot load any chip.
package_data = {
    "xuanwu": [
        "data/chips/arm/cortex_m/*.yaml",
        "data/gdb/*.md",
        "data/gdb/features/arm/*.xml",
    ],
}

# Python >= 3.7
setup(
    name="xuanwu",
    version=find_version(),
    description="Micro-controller simulator.",
    long_description=read("README.md"),
    long_description_content_type="text/markdown",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    package_data=package_data,
    include_package_data=True,
    install_requires=install_requires,
    python_requires=">=3.9",
    author="onelife",
    author_email="onelife.real@gmail.com",
    license="LGPL-2.1",
    url="https://github.com/onelife/xuanwu",
    download_url="https://github.com/onelife/xuanwu/archive/%s.tar.gz" % find_version(),
    keywords=['mcu', 'simulator'],
)
