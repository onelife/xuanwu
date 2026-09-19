# -*- coding: utf-8 -*-

"""The Rung 3 milestone: Adafruit's ``graphicstest`` runs to the end.

This is the whole point of the TFT work so far -- a real, unmodified Adafruit example
driving a simulated panel through a simulated Due -- and it is deliberately a
``milestone`` test rather than part of the ordinary suite: the sketch fills the screen
five times over, draws every primitive the library has and streams about eight million
bytes down the SPI bus, which takes minutes.

Run it with::

    pytest -m milestone tests/integration/test_due_tft_milestone.py

What is checked is that the sketch completes, that the benchmark lines it prints name
every test in the order the example has them, and that the frame it leaves behind is
the one recorded as the golden digest.  Anything that changes the drawing -- a wrong
colour order, a lost pixel, a mis-decoded command -- changes the digest.
"""

import logging
from types import SimpleNamespace

import pytest

from xuanwu import XuanWu

pytestmark = [pytest.mark.integration, pytest.mark.slow, pytest.mark.milestone]

# The frame `graphicstest` leaves on the panel after its last test
# (`testFilledRoundRects`: a green gradient of rounded rectangles on black), from the
# run that produced the reviewed PNG in `docs/images/due_tft_graphicstest.png`.
GOLDEN_DIGEST = "53a338e761b4524ba965e3b7b26fe7ea214f5343f954f75524f52283cc220806"

BENCHMARKS = [
    "Screen fill",
    "Text",
    "Lines",
    "Horiz/Vert Lines",
    "Rectangles (outline)",
    "Rectangles (filled)",
    "Circles (filled)",
    "Circles (outline)",
    "Triangles (outline)",
    "Triangles (filled)",
    "Rounded rects (outline)",
    "Rounded rects (filled)",
]

REGISTERS = {
    "Display Power Mode": "9C",
    "MADCTL Mode": "48",
    "Pixel Format": "55",
    "Image Format": "0",
    "Self Diagnostic": "0",
}


@pytest.fixture(scope="module")
def graphicstest(sam3x8e_tft_path, due_tft_firmware):
    logging.disable(logging.CRITICAL)
    device = XuanWu(
        str(sam3x8e_tft_path), str(due_tft_firmware), hardware_options={"bridge": "loopback"}
    )
    device.reset()
    bridge = device.hw.perif["uart"]._bridge
    panel = device.dev["LCD"]

    text = b""
    for _ in range(400):  # 4 000 M instructions, several times what the run needs
        device.run(count=10_000_000)
        text += bridge.drain()
        if b"Done!" in text:
            break
    else:
        pytest.fail(f"graphicstest never finished: {text.decode('utf-8', 'replace')[-400:]!r}")

    logging.disable(logging.NOTSET)
    return SimpleNamespace(device=device, panel=panel, text=text.decode("utf-8", "replace"))


def lines(graphicstest) -> list:
    return [line.strip() for line in graphicstest.text.splitlines() if line.strip()]


class TestTheSketchRan:
    def test_it_says_so(self, graphicstest):
        assert lines(graphicstest)[0] == "ILI9341 Test!"
        assert lines(graphicstest)[-1] == "Done!"

    def test_the_register_diagnostics_are_the_panels_own_values(self, graphicstest):
        for label, value in REGISTERS.items():
            assert f"{label}: 0x{value}" in lines(graphicstest), label

    def test_every_benchmark_ran_in_order(self, graphicstest):
        for benchmark in BENCHMARKS:
            assert benchmark in graphicstest.text, f"{benchmark} never ran"
        order = [graphicstest.text.index(name) for name in BENCHMARKS]
        assert order == sorted(order), "the benchmarks ran out of order"

    def test_the_benchmarks_took_time_the_simulation_could_measure(self, graphicstest):
        times = [line for line in lines(graphicstest) if line.split()[-1].isdigit()]
        assert len(times) >= len(BENCHMARKS)
        # The screen fill is the one number with an obvious lower bound: it pushes
        # 240 * 320 * 2 bytes five times.
        fill = int(next(line for line in times if line.startswith("Screen fill")).split()[-1])
        assert fill > 100_000, "a five-screen fill cannot be that fast, even simulated"


class TestTheFrameItLeftBehind:
    def test_the_digest_is_the_recorded_one(self, graphicstest):
        surface = graphicstest.panel.surface
        assert surface.size == (240, 320)
        assert surface.digest() == GOLDEN_DIGEST

    def test_the_last_test_is_what_is_on_the_screen(self, graphicstest):
        """`testFilledRoundRects` fills its rectangles with ``color565(0, i, 0)``.

        One shade of green per rectangle, from ``i = 240`` down to just over 20, all
        centred on the panel: the largest spans ``y = 39 .. 278`` (``cy - i/2`` with
        ``cy = 159``) across the full width, and the rest nest inside it.  So the frame
        is a green gradient inside a rounded square, black outside it.
        """
        surface = graphicstest.panel.surface
        histogram = surface.histogram()
        greens = {
            colour: count
            for colour, count in histogram.items()
            if colour and (colour >> 5) & 0x3F > (colour >> 11) & 0x1F and (colour >> 5) & 0x3F > colour & 0x1F
        }
        assert len(greens) > 5, "the rectangles are a gradient, one shade each"
        assert sum(greens.values()) > 50_000, "and they cover most of the screen"
        assert surface.pixel(120, 160) in greens, "the largest one is filled, centre included"

        # Nothing outside the largest rectangle was touched: the rows just inside it
        # have colour, the rows just outside it are entirely black.
        assert set(surface.row(38)) == {0}
        assert set(surface.row(39)) != {0}
        assert set(surface.row(278)) != {0}
        assert set(surface.row(279)) == {0}
        assert histogram[0x0000] > 15_000, "the black border above and below it"

    def test_the_bus_was_not_shared_by_accident(self, graphicstest):
        bus = graphicstest.device.hw.perif["spi"].bridge
        assert graphicstest.panel.bytes_received > 8_000_000
        assert bus.unclaimed_bytes == 0

    def test_the_panel_is_on_when_the_test_ends(self, graphicstest):
        assert graphicstest.panel.core.display_on is True
        assert graphicstest.panel.core.colmod == 0x55
