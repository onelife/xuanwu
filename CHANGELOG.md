# Changelog

All notable changes to this project are documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Changed — host serial integration

- The SAM UART/SPI models no longer hard-depend on `socat`. The host side is now
  a pluggable bridge (`xuanwu.backends`):
  - `socat` — a pty pair, the default on POSIX (previous behaviour);
  - `tcp` — the simulator listens on `tcp://host:port` and the peer connects;
  - `loopback` — in-memory, for tests, uses no host resources.
  `bridge: auto` (the chip YAML default) picks `socat` when it is installed and
  TCP otherwise, so `sam3x8e.yaml` now works on Windows.
- The bridge can be selected per chip (`bridge:` in the chip YAML) or per run:
  `XuanWu(chip, code, hardware_options={"bridge": "loopback", "baudrate": 9600})`.
- `ArmSamUart.peer_hint` / `ArmSamSpi.peer_hint` replace `tty_device`; the value
  is a pty path for socat and `tcp://host:port` for TCP.
- `requirements.txt`: `unicorn ~= 2.0.0` → `unicorn >= 2.1` and
  `capstone ~= 4.0.2` → `capstone >= 5.0`. **Both** old versions import
  `distutils`/`pkg_resources`, which no longer exist in modern Python/setuptools,
  so on Python 3.12 the project could not be imported at all without pinning
  `setuptools<81`. That pin is gone from the Dockerfile and the CI workflow.

### Changed — repository layout (this release is not source-compatible with 0.0.1)

- Adopted the **src layout**: the package moved from `xuanwu/` to `src/xuanwu/`,
  so the source tree can no longer shadow an installed copy.
- **Chip descriptions and GDB target descriptions moved inside the package**
  (`src/xuanwu/data/chips`, `src/xuanwu/data/gdb`). They used to live at the
  repository root and were addressed as `../chip` / `../gdb`, which meant the
  built wheel contained none of them and an installed xuanwu could not load any
  chip. The wheel now ships 2 chip descriptions and 11 target description files.
- Test firmware moved out of the package into `tests/firmware/`.
- `Dockerfile` and `docker-compose.yml` moved into `docker/`; the image now also
  installs `socat` (required by the SAM UART/SPI models) and `gdb-multiarch`,
  and pins `setuptools<81` so it can actually import the project.
- `test.py` replaced by `examples/blinky.py`.
- Removed dead code and build leftovers from the repository: `dev/spi.py` (a
  byte-for-byte copy of `memory.py` that was never imported), `q`,
  `xuanwu.log`, `build/`, `xuanwu.egg-info/`.
- `XuanWu(chip=...)` now accepts a **chip name** (`"stm32f411"`) as well as a
  path; `xuanwu.chips.list_chips()` lists what is bundled.

### Changed — module structure

- `arch/armv7m.py` (1266 lines) split into `arch/base.py` plus
  `arch/cortex_m/{controller,constants,scid,systick,nvic,scb,cp,dbg,dwt}.py`.
- `arch/atmel_sam.py` (1697 lines) split into `arch/vendor/atmel/`
  (`pmc, pdc, pio, adc, uart, spi, pwm, uotghs, common`) and `arch/stm_stm.py`
  into `arch/vendor/st/` (`rcc, gpio`).
- The host-dependent serial bridge moved to `backends/serial_bridge.py`, so the
  simulation core no longer starts subprocesses itself.
- The old module paths (`xuanwu.arch.armv7m`, `xuanwu.arch.atmel_sam`,
  `xuanwu.arch.stm_stm`) remain importable as deprecated aliases that emit a
  `DeprecationWarning`; they will be removed in a later release.

### Fixed — GDB stub (it used to be unusable)

- **The stub crashed on any packet it did not implement.** `process_packet()`
  returns `None` for unknown packets, and `write_data()` only accepted `str`, so
  the encoder raised `TypeError` and killed the whole accept loop. `None` now
  encodes to a valid empty reply, and a failing packet is logged and answered
  with `E01` instead of taking the server down.
- **Reading a single register (`p<n>`) crashed the stub.** Those replies are raw
  `bytes`; `write_data()` now hex-encodes them instead of raising
  `cannot use a string pattern on a bytes-like object`.
- **`qXfer:features:read` was neither advertised nor implemented**, so GDB never
  learned the register layout and fell back to its default (much larger) register
  set, reporting `Truncated register 16 in remote 'g' packet`. The feature is now
  advertised and `target.xml` is served as a proper `<target>` document —
  a bare `<feature>` element is silently ignored by GDB.
- Run-length encoding produced an unescaped `$`/`*`/`}` inside the packet body
  for runs of exactly 14 or 97 bytes, corrupting the frame.

### Fixed — core abstractions

- `ArmHardwareBase.read()`/`write()` checked an **uninitialised** `data_orig`
  instead of the initialised `data`/`value`, so an access past the end of a
  peripheral block raised `UnboundLocalError` instead of `XwInvalidMemoryAddress`.
- `ArmHardwareNvic.fix_after_read()`/`fix_before_write()` performed
  `name_[1] = "S"` on a `str`, so reading `ICER`/`ICPR` always raised
  `TypeError`.
- `ArmHardwareNvic.set_pending()`/`set_active()` set the `ISPR`/`IABR` bit in
  the wrong direction (pending/active read back as *not* pending/active).
- `ArmSamGpio` registered its read hook as `_fix_afte5r_read` (typo), so the
  `ISR` read-to-clear behaviour never ran.
- `ArmSamSpi` referenced the non-existent enum member `SPI_MR.SPI_MR`.

### Fixed — packaging

- **Importing the library created `./xuanwu.log` in the current working
  directory**, so `import xuanwu` failed outright in any read-only, container or
  CI working directory. Logging now goes to the console by default, and to
  `$XDG_STATE_HOME/xuanwu/` only when `DEBUG=1`.
- `setup.py` imported the package to read `__version__`, which required
  unicorn/capstone at build time. The version now lives in `xuanwu/_version.py`
  and is read as text, so `pip install .` works in an isolated build
  environment.
- Removed the call-stack introspection hack from `xuanwu/__init__.py` that tried
  to detect `setup.py`/`build_meta.py` frames.

### Fixed — diagnostics

- Six `raise` sites were missing the `f` prefix and printed literal `{name}`
  placeholders instead of the offending value.
- `SystemExit`-style silent failures replaced by `XwSerialBridgeError` when the
  SAM serial bridge cannot start (e.g. `socat` missing), with the socat error
  text attached.
- `ArmSamUart`/`ArmSamSpi.__del__` raised `AttributeError` when construction had
  failed part-way; cleanup is now safe on half-built objects.

### Changed

- The SAM serial bridge waits for the pty symlinks to appear instead of sleeping
  for a fixed second per peripheral, removing a mandatory ~2 s from every
  simulation that maps UART/SPI.
- `SYSTICK.CALIB` is configurable per chip (`calib:` in the YAML) instead of
  being hard-coded to the STM32F4 value, and `SCB.CPUID` is taken from the chip
  description instead of assuming a Cortex-M4.
- `SCB.SCR` resets to `0` instead of `0xFA050000` (an AIRCR leftover).

### Added

- `tests/` suite: unit regression tests for every fix above, chip-description
  conformance tests, and integration tests for firmware smoke runs and the GDB
  stub (78 tests).
- `tests/conformance/test_chip_descriptions.py` validates newly written chip
  YAML in seconds: unknown arch/mode, overlapping mappings, `core` peripherals
  outside a mapped region, and peripheral names that resolve to no model.
- GitHub Actions CI running the suite on Linux and Windows across Python
  3.10–3.13, plus an isolated-wheel packaging job.
- `requirements-dev.txt`.

[Unreleased]: https://github.com/onelife/xuanwu/commits/master
