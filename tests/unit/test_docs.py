# -*- coding: utf-8 -*-

"""The documentation has to point at things that exist.

A manual is only useful while it is true, and the failures are quiet: a renamed
example, a moved module and a link that goes nowhere all read like normal prose.
These checks are cheap and catch exactly that.
"""

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
DOCS = [
    REPO_ROOT / "README.md",
    *(REPO_ROOT / "docs").glob("*.md"),
    REPO_ROOT / "tests" / "firmware" / "README.md",
]

# [text](target) -- but not images
LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)]+)\)")

# Paths written in backticks that are *meant* to be examples of files that do not
# exist yet: a chip description, a board entry or a sketch the reader is being shown
# how to write.  Listing them here keeps the check honest instead of vague.
PLACEHOLDERS = {
    "src/xuanwu/data/chips/arm/cortex_m/stm32f103.yaml",
    "stm32f103.yaml",
    "samd21.yaml",
    "stm32f767.yaml",
    "tests/firmware/Blink_f103/Blink_f103.ino",
}

SKIP_DIRS = {".git", "__pycache__", ".pytest_cache", "build", "dist", ".venv", ".ruff_cache"}


def documents():
    return sorted(path for path in DOCS if path.is_file())


def relative_links(path: Path):
    for target in LINK.findall(path.read_text(encoding="utf-8")):
        target = target.split("#", 1)[0].strip()
        if not target or "://" in target or target.startswith(("mailto:", "#")):
            continue
        yield target


@pytest.fixture(scope="module")
def repo_files() -> set:
    return {
        path.relative_to(REPO_ROOT).as_posix()
        for path in REPO_ROOT.rglob("*")
        if path.is_file() and not SKIP_DIRS.intersection(path.parts)
    }


@pytest.mark.parametrize("path", documents(), ids=lambda path: path.name)
def test_relative_links_resolve(path):
    missing = [
        target for target in relative_links(path) if not (path.parent / target).resolve().exists()
    ]
    assert not missing, f"{path.relative_to(REPO_ROOT)} links to something that does not exist: {missing}"


@pytest.mark.parametrize("path", documents(), ids=lambda path: path.name)
def test_referenced_repo_paths_exist(path, repo_files):
    """Paths in backticks, e.g. ``tests/firmware/build.py`` or ``arch/base.py``.

    Docs refer to files by a suffix rather than a full path (``arch/base.py`` from
    the architecture guide, ``stm32f411.yaml`` from the README), so a mention counts
    as resolved when *some* file in the repository ends with it.

    A plan is the one document that names files on purpose *before* they exist, so
    it is exempt; its links still have to resolve.
    """
    if path.name.startswith("plan-"):
        pytest.skip("a plan describes files that do not exist yet")
    text = path.read_text(encoding="utf-8")
    candidates = set(re.findall(r"`([A-Za-z0-9_./-]+\.(?:py|md|yaml|yml|sh|ino|xml|cfg|toml|txt))`", text))
    missing = []
    for candidate in sorted(candidates):
        if candidate.startswith(("http", "/")) or "*" in candidate or candidate in PLACEHOLDERS:
            continue
        if not any(f == candidate or f.endswith("/" + candidate) for f in repo_files):
            missing.append(candidate)
    assert not missing, f"{path.relative_to(REPO_ROOT)} mentions files that do not exist: {missing}"


def slug(heading: str) -> str:
    """GitHub's heading anchor: drop markup and punctuation, spaces to hyphens."""
    text = heading.lstrip("# ").strip().lower().replace("`", "")
    text = re.sub(r"[^\w\s-]", "", text)
    return re.sub(r"\s+", "-", text)


class TestTheManual:
    def manual(self) -> str:
        return (REPO_ROOT / "docs" / "manual.zh-CN.md").read_text(encoding="utf-8")

    def test_it_is_linked_from_the_readme(self):
        assert (REPO_ROOT / "docs" / "manual.zh-CN.md").is_file()
        assert "manual.zh-CN.md" in (REPO_ROOT / "README.md").read_text(encoding="utf-8")

    def test_every_example_script_is_documented(self):
        examples = sorted(path.name for path in (REPO_ROOT / "examples").glob("*.py"))
        manual = self.manual()
        undocumented = [name for name in examples if name not in manual]
        assert not undocumented, f"examples not mentioned in the manual: {undocumented}"

    def test_its_table_of_contents_matches_its_headings(self):
        manual = self.manual()
        anchors = [target for target in LINK.findall(manual) if target.startswith("#")]
        assert anchors, "the manual has no table of contents"
        headings = {
            slug(line) for line in manual.splitlines() if line.startswith("#") and not line.startswith("###")
        }
        missing = [anchor for anchor in anchors if anchor[1:] not in headings]
        assert not missing, f"the table of contents points at headings that do not exist: {missing}"
