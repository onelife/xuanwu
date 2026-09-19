# -*- coding: utf-8 -*-

"""Every script in ``examples/`` has to work.

The examples are the documentation's quick start, and a manual whose snippets have
rotted is worse than no manual, so they are run here as real programs -- a fresh
interpreter, the same command line a reader would type.
"""

import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = [pytest.mark.integration, pytest.mark.slow]

REPO_ROOT = Path(__file__).resolve().parents[2]

# name -> a string the example must print when it worked
EXAMPLES = {
    "blinky": "after",
    "uart_bridge": "millis1",
    "semihosting": "hello from semihosting",
    "fpu": "values held in s0-s3",
    "devices": "JEDEC ID",
    "unclaimed_io": "unclaimed address(es)",
    "tft": "frame digest",
}


def run_example(name: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(REPO_ROOT / "examples" / f"{name}.py"), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=300,
    )


@pytest.mark.parametrize("name", sorted(EXAMPLES))
def test_example_runs(name):
    result = run_example(name)
    output = result.stdout + result.stderr
    assert result.returncode == 0, f"examples/{name}.py exited with {result.returncode}:\n{output}"
    assert EXAMPLES[name] in result.stdout, f"examples/{name}.py printed:\n{result.stdout}"


def test_the_unclaimed_report_accepts_an_instruction_budget():
    result = run_example("unclaimed_io", "50000")
    assert result.returncode == 0
    assert "after 50,000 instructions" in result.stdout


def test_the_fpu_example_proves_the_frame_is_needed():
    """It must show the values surviving *and* the negative control destroying them."""
    result = run_example("fpu")
    assert result.returncode == 0
    assert "[('a', 1.5), ('b', 2.25), ('c', -3.75), ('d', 100.0)]" in result.stdout
    assert "[('a', 9.0), ('b', 9.0), ('c', 8.0), ('d', 8.0)]" in result.stdout


def test_the_uart_example_measures_simulated_time():
    """delay(5) twice must move millis() by at least 10 ms."""
    result = run_example("uart_bridge")
    assert result.returncode == 0
    assert "advanced by" in result.stdout
    advanced = int(result.stdout.split("advanced by ")[1].split(" ms")[0])
    assert advanced >= 10, result.stdout
