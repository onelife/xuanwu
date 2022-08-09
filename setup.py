# -*- coding: utf-8 -*-

from setuptools import setup, find_packages
from os import path
import xuanwu


with open(path.join(".", "README.md"), encoding="utf-8") as f:
    long_description = f.read()

with open("requirements.txt", encoding="utf-8") as f:
    install_requires = [line.strip() for line in f.read().split("\n") if not line.strip().startswith("#")]

# Python >= 3.7
setup(
    name="xuanwu",
    version=xuanwu.__version__,
    description="Micro-controller simulator.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(),
    install_requires=install_requires,
    author="onelife",
    author_email="onelife.real@gmail.com",
    license=xuanwu.__license__,
    url="https://github.com/onelife/xuanwu",
    download_url="https://github.com/onelife/xuanwu/archive/%s.tar.gz" % xuanwu.__version__,
    keywords=['mcu', 'simulator'],
)
