#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Build the bundled test firmware with the Arduino CLI.

Every image is described by ``board.yaml``: which board definition compiles it,
which sketch it comes from and which xuanwu chip description runs it.  This script
turns that into ``arduino-cli compile`` calls and copies the resulting
``.elf``/``.bin``/``.hex`` into the board's output directory, where the test suite
picks them up.

    python tests/firmware/build.py                 # everything
    python tests/firmware/build.py sam3x8e         # one board
    python tests/firmware/build.py --list          # the matrix, without building

The Arduino CLI and the cores it needs are part of the development image
(``docker/Dockerfile``); see ``docs/manual.zh-CN.md`` for the manual route.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
CONFIG = HERE / "board.yaml"


def load_config() -> dict:
    with open(CONFIG, encoding="utf-8") as file:
        return yaml.safe_load(file)


def arduino_cli(config: dict) -> str:
    """The CLI to run: board.yaml, then $ARDUINO_CLI, then whatever is on PATH."""
    candidate = os.environ.get("ARDUINO_CLI") or config.get("arduino_cli") or "arduino-cli"
    if shutil.which(candidate) is None:
        print(f"error: {candidate!r} is not on PATH.", file=sys.stderr)
        print("  Inside the development image it is already installed; otherwise see", file=sys.stderr)
        print("  docs/manual.zh-CN.md, section '构建测试固件'.", file=sys.stderr)
        raise SystemExit(2)
    return candidate


def describe(boards: list) -> None:
    print(f"{'name':16s} {'chip':12s} {'output':16s} board")
    for board in boards:
        print(
            f"{board['name']:16s} {board.get('chip') or '-':12s} "
            f"{board['output']:16s} {board['board']}"
        )
        print(f"{'':16s} {'':12s} {'':16s} {board['fqbn']}")


def build(cli: str, board: dict, build_root: Path) -> bool:
    sketch = HERE / board["sketch"] / f"{board['sketch']}.ino"
    if not sketch.is_file():
        print(f"  !! {sketch} does not exist")
        return False
    output = HERE / board["output"]
    output.mkdir(parents=True, exist_ok=True)
    build_path = build_root / board["name"]

    command = [
        cli,
        "compile",
        "--fqbn",
        board["fqbn"],
        "--output-dir",
        str(output),
        "--build-path",
        str(build_path),
        "--export-binaries",
        str(sketch.parent),
    ]
    print(f"  $ {' '.join(command)}")
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stdout[-4000:])
        print(result.stderr[-4000:], file=sys.stderr)
        return False

    produced = sorted(p.name for p in output.iterdir() if p.is_file())
    print(f"  ok: {', '.join(produced)}")
    return True


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("boards", nargs="*", help="board names to build (default: all)")
    parser.add_argument("--list", action="store_true", help="print the matrix and exit")
    parser.add_argument("--keep-build", action="store_true", help="keep the compiler's build directory")
    args = parser.parse_args(argv)

    config = load_config()
    boards = config["boards"]
    if args.list:
        describe(boards)
        return 0

    wanted = set(args.boards)
    unknown = wanted - {board["name"] for board in boards}
    if unknown:
        parser.error(f"unknown board(s): {', '.join(sorted(unknown))}")
    selected = [board for board in boards if not wanted or board["name"] in wanted]

    cli = arduino_cli(config)
    print(f"arduino-cli: {subprocess.run([cli, 'version'], capture_output=True, text=True).stdout.strip()}")

    failures = []
    with tempfile.TemporaryDirectory(prefix="xuanwu-firmware-") as tmp:
        build_root = Path(tmp) if not args.keep_build else Path(tempfile.mkdtemp(prefix="xuanwu-firmware-"))
        for board in selected:
            print(f"== {board['name']} ({board['board']})")
            if not build(cli, board, build_root):
                failures.append(board["name"])

    if failures:
        print(f"\nFAILED: {', '.join(failures)}", file=sys.stderr)
        return 1
    print(f"\nbuilt {len(selected)} firmware image(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
