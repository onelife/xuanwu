# -*- coding: utf-8 -*-

"""Packaging regressions.

  * importing the library must not write into the current working directory
    (it used to create ./xuanwu.log, so ``import xuanwu`` failed in any
    read-only, container or CI working directory)
  * the version must have a single source of truth
  * ``setup.py`` must be runnable in an isolated build environment, where
    neither unicorn nor the package itself is importable
  * the chip descriptions and GDB target descriptions must live *inside* the
    package, otherwise an installed xuanwu cannot load any chip
"""

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"
VERSION_FILE = SRC_DIR / "xuanwu" / "_version.py"


def declared_version() -> str:
    match = re.search(r'^__version__\s*=\s*"([^"]+)"', VERSION_FILE.read_text(encoding="utf-8"), re.MULTILINE)
    assert match, "no __version__ in src/xuanwu/_version.py"
    return match.group(1)


def test_layout_is_src_based():
    assert SRC_DIR.is_dir(), "the package must live under src/"
    assert (SRC_DIR / "xuanwu" / "__init__.py").is_file()
    assert not (REPO_ROOT / "xuanwu").exists(), "no second copy of the package at the repository root"


def test_import_does_not_write_into_cwd(tmp_path):
    env = dict(os.environ)
    env.pop("DEBUG", None)
    env["PYTHONPATH"] = str(SRC_DIR)

    result = subprocess.run(
        [sys.executable, "-c", "import xuanwu; print(xuanwu.__version__)"],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
    )

    assert result.returncode == 0, result.stderr
    leftovers = [entry.name for entry in tmp_path.iterdir()]
    assert leftovers == [], f"importing xuanwu created {leftovers} in the cwd"


def test_version_has_a_single_source():
    import xuanwu

    assert xuanwu.__version__ == declared_version()


def test_setup_py_does_not_import_the_package():
    text = (REPO_ROOT / "setup.py").read_text(encoding="utf-8")
    assert "import xuanwu" not in text
    assert "find_version" in text


def test_package_data_lives_inside_the_package():
    """An installed xuanwu must be able to find its own chip/GDB descriptions."""
    import xuanwu
    from xuanwu.config import RESOURCE

    package_root = Path(xuanwu.__file__).resolve().parent
    for key, value in RESOURCE.items():
        location = Path(value).resolve()
        assert location.is_relative_to(package_root), f"RESOURCE[{key!r}] escapes the package: {location}"
        assert location.is_dir(), f"RESOURCE[{key!r}] does not exist: {location}"
        assert any(location.rglob("*")), f"RESOURCE[{key!r}] is empty: {location}"


def test_chips_are_discoverable_by_name():
    from xuanwu.chips import list_chips, resolve_chip

    names = list_chips()
    assert "stm32f411" in names and "sam3x8e" in names
    resolved = Path(resolve_chip("stm32f411"))
    assert resolved.is_file() and resolved.name == "stm32f411.yaml"
    # A path still works unchanged.
    assert resolve_chip(str(resolved)) == str(resolved)


def test_setup_py_runs_without_the_runtime_dependencies(tmp_path):
    """Simulate an isolated PEP 517 build environment (no unicorn importable)."""
    for name in ("setup.py", "requirements.txt", "README.md"):
        shutil.copy(REPO_ROOT / name, tmp_path / name)
    package = tmp_path / "src" / "xuanwu"
    package.mkdir(parents=True)
    shutil.copy(VERSION_FILE, package / "_version.py")
    (package / "__init__.py").write_text("", encoding="utf-8")

    result = subprocess.run(
        [sys.executable, "setup.py", "--version"],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=120,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip().endswith(declared_version())


@pytest.mark.slow
def test_wheel_ships_the_chip_and_gdb_descriptions(tmp_path):
    """Build a wheel and check the data files are really inside it.

    The build runs on a copy of the source tree so that setuptools' ``build/``
    and ``*.egg-info`` artifacts never land in the repository.
    """
    import zipfile

    build_root = tmp_path / "src-tree"
    build_root.mkdir()
    shutil.copytree(SRC_DIR, build_root / "src")
    for name in ("setup.py", "pyproject.toml", "MANIFEST.in", "requirements.txt", "README.md"):
        shutil.copy(REPO_ROOT / name, build_root / name)

    result = subprocess.run(
        [sys.executable, "-m", "pip", "wheel", ".", "--no-deps", "-w", str(tmp_path / "dist")],
        cwd=build_root,
        capture_output=True,
        text=True,
        timeout=600,
    )
    assert result.returncode == 0, result.stderr

    wheels = list((tmp_path / "dist").glob("xuanwu-*.whl"))
    assert wheels, "no wheel was produced"
    names = zipfile.ZipFile(wheels[0]).namelist()
    assert any(name.endswith(".yaml") for name in names), f"no chip descriptions in the wheel: {names}"
    assert any(name.endswith(".xml") for name in names), f"no GDB target descriptions in the wheel: {names}"
