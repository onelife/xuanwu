# -*- coding: utf-8 -*-

"""Ways of looking at a display surface.

``headless`` is the default and needs nothing: the surface is still there to assert
against, and can be written out as a PNG.  ``pygame`` is an optional extra
(``pip install xuanwu[gui]``) and is imported lazily, so nothing else pays for it.
"""

from abc import ABC
from typing import Any, Dict, Optional

from ...config import logger
from ...exception import XwInvalidParameter
from ...peripherals.display import DisplaySurface

__all__ = ["Viewer", "HeadlessViewer", "PygameViewer", "create_viewer"]


class Viewer(ABC):
    """Something that shows a surface and can be asked to refresh it."""

    name = "viewer"

    def __init__(self, title: str = "xuanwu", scale: int = 1, dump: Optional[str] = None) -> None:
        self.title = title
        self.scale = max(1, int(scale))
        self.dump = dump
        """Where to write the last frame on :meth:`close` (a PNG path), if anywhere."""
        self.frame: Optional[DisplaySurface] = None
        self.updates = 0

    def update(self, surface: DisplaySurface, force: bool = False) -> None:
        """Show the surface, using only what changed unless ``force``."""
        self.frame = surface
        self.updates += 1

    def close(self) -> None:
        """Release whatever the viewer holds; safe to call twice."""
        if self.dump and self.frame is not None:
            self.frame.to_png(self.dump)
            logger.info(f"[viewer  ]: wrote {self.dump}")

    def pump(self) -> bool:
        """Handle window events; returns False when the user closed the window."""
        return True

    def __enter__(self) -> "Viewer":
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.close()


class HeadlessViewer(Viewer):
    """Keeps the last frame and counts updates; for CI and for tests."""

    name = "headless"

    def update(self, surface: DisplaySurface, force: bool = False) -> None:
        super().update(surface, force)
        surface.clear_dirty()


class PygameViewer(Viewer):
    """A real window, one blit per change.

    The surface is uploaded with ``pygame.image.frombuffer`` only for the dirty box, so
    a firmware that fills a small rectangle does not cost a full-screen conversion.
    """

    name = "pygame"

    def __init__(
        self, title: str = "xuanwu", scale: int = 1, fps: int = 60, dump: Optional[str] = None
    ) -> None:
        super().__init__(title, scale, dump)
        try:
            import pygame
        except ImportError as error:  # pragma: no cover - depends on the extras
            raise XwInvalidParameter(
                "the pygame viewer needs the optional dependency: pip install 'xuanwu[gui]'"
            ) from error

        self._pygame = pygame
        pygame.init()
        self._screen = pygame.display.set_mode((0, 0), pygame.RESIZABLE)
        pygame.display.set_caption(title)
        self._clock = pygame.time.Clock()
        self._fps = max(1, int(fps))
        self._image: Any = None
        self._size = (0, 0)

    def update(self, surface: DisplaySurface, force: bool = False) -> None:
        pygame = self._pygame
        if surface.size != self._size:
            self._size = surface.size
            self._image = None
            force = True
        self._draw(surface, force)
        pygame.display.flip()
        super().update(surface, force)
        surface.clear_dirty()
        self._clock.tick(self._fps)

    def _draw(self, surface: DisplaySurface, force: bool) -> None:
        pygame = self._pygame
        if self._image is None:
            self._image = pygame.image.frombuffer(bytes(surface.pixels), surface.size, "RGB565")
            self._screen.blit(pygame.transform.scale(self._image, self._scaled()), (0, 0))
            return
        box = (0, 0, surface.width, surface.height) if force else (surface.dirty or None)
        if box is None:
            return
        x0, y0, x1, y1 = box
        if x1 <= x0 or y1 <= y0:
            return
        stride = surface.width * 2
        chunk = bytearray()
        for y in range(y0, y1):
            start = y * stride + x0 * 2
            chunk += surface.pixels[start : start + (x1 - x0) * 2]
        patch = pygame.image.frombuffer(bytes(chunk), (x1 - x0, y1 - y0), "RGB565")
        scaled = pygame.transform.scale(patch, ((x1 - x0) * self.scale, (y1 - y0) * self.scale))
        self._screen.blit(scaled, (x0 * self.scale, y0 * self.scale))

    def _scaled(self) -> tuple:
        return (self._size[0] * self.scale, self._size[1] * self.scale)

    def pump(self) -> bool:
        for event in self._pygame.event.get():
            if event.type == self._pygame.QUIT:
                return False
            if event.type == self._pygame.KEYDOWN and event.key == self._pygame.K_ESCAPE:
                return False
        return True

    def close(self) -> None:
        try:
            self._pygame.quit()
        except Exception:  # noqa: BLE001 - shutting down must never raise
            pass
        super().close()


VIEWERS: Dict[str, Any] = {
    "headless": HeadlessViewer,
    "pygame": PygameViewer,
    "none": None,
}


def create_viewer(kind: str = "headless", **options: Any) -> Optional[Viewer]:
    """Build a viewer by name; ``none`` means the firmware just runs."""
    kind = (kind or "headless").lower()
    if kind not in VIEWERS:
        raise XwInvalidParameter(f"Unknown viewer {kind!r}; available: {', '.join(sorted(VIEWERS))}")
    factory = VIEWERS[kind]
    return None if factory is None else factory(**options)
