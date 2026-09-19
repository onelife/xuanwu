# Changelog

All notable changes to this project are documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added — floating-point support

- The Cortex-M FP extension is modelled, so an Armv7E-M core runs VFP code and
  the exception engine stacks the extended frame:
  - `arch/cortex_m/fpu.py` (`ArmHardwareFpu`) answers the FP system registers a
    Cortex-M4F firmware reads while starting up: `FPCCR`, `FPCAR`, `FPDSCR` and
    `MVFR0/1/2`. `FPCCR.LSPACT` always reads as zero, which is what a model
    without lazy stacking has to report.
  - `push_context`/`pop_context` stack and restore the floating-point frame —
    `S0`-`S15`, `FPSCR` and the reserved word, 0x48 bytes *below* the 0x20-byte
    integer frame — whenever `CONTROL.FPCA` is set, and clear `EXC_RETURN` bit 4
    so the return path pops it again.
  - `arm_core_registers` gained `s0`-`s31`, `d0`-`d15` and `q0`-`q15`.
  - `stm32f411.yaml` declares the block:
    `- FPU: {type: core, base: 0xE000EF30, size: 0x1C}`. A chip that does not
    declare it keeps the old behaviour.
- The GDB stub describes the FP register file when the core has one: it serves
  `arm-m-profile.xml` followed by `arm-vfpv2.xml` and lays the `g` packet out in
  that order (17 core registers, then `d0`-`d15` and `fpscr`). Verified against a
  real `gdb-multiarch`, which now reports `d0`/`fpscr` instead of falling back to
  its own default register set.
- `tests/firmware/stm32f411/fpu_test/` — a bare-metal Cortex-M4F firmware that
  holds four values in `S0`-`S3` across three SysTick interrupts whose handler
  clobbers those very registers. The values can only survive if the FP frame is
  stacked and restored, and a negative control run disables the stacking and
  asserts the values are destroyed, so the test cannot pass vacuously.

### Fixed — `EXC_RETURN` from an FP frame was rejected

- The exception-return validator required bits 31:4 to be all ones, which
  rejected every `EXC_RETURN` that reports a stacked FP frame (`0xFFFFFFE1`,
  `0xFFFFFFE9`, `0xFFFFFFED`) with `RuntimeError: UNPREDICTABLE`. It now requires
  bits 31:5 and one of the four valid mode nibbles.

### Fixed — the semihosting trap left the core in Arm state

- Stepping the PC over `BKPT 0xAB` wrote an even address to the program counter.
  Unicorn reads bit 0 of a PC write as the instruction-set selector, so the next
  instruction was fetched in Arm state and the run died with
  `UC_ERR_INSN_INVALID`. The write now goes through a new
  `RegisterController.pc_t` setter, which keeps the bit set.

### Fixed — `run(count=...)` counted milliseconds

- `XuanWu.run()` passed its instruction budget as the third positional argument
  of `uc_emu_start()`, which is the *timeout*. Every `run(count=N)` therefore ran
  for N milliseconds instead of N instructions: the small counts used by tests
  were timing-dependent, and the documented "execute 500k instructions" was
  wrong. Both limits are now passed by keyword.

### Added — external device layer

- `device.py` is no longer an empty stub. A chip description can now declare
  what is wired to the MCU, and those devices are built and attached once the
  peripherals exist:
  ```yaml
  devices:
    - name: LED
      type: led
      port: GPIOB
      pin: 27
    - name: FLASH
      type: spi_flash
      port: SPI
      cs: {port: GPIOC, pin: 26, active_low: true}
  ```
  `device.dev["LED"]`, `device.dev.names()` and iteration over `device.dev`
  expose them.
- Two device models ship: `led` (watches a GPIO pin through
  `ArmSamGpio.add_hook`, which had no callers until now) and `spi_flash` (a NOR
  flash that answers JEDEC ID, read status, write enable/disable, read data,
  page program and the three erase commands, framed by its chip-select pin).
- A device can take over a peripheral's byte stream by assigning
  `peripheral.bridge = itself`, so the SPI bus is driven in-process instead of
  through `socat` or TCP. `create_bridge()` accepts a `SerialBridge` instance
  for that, and a new `NullBridge` (`bridge: none`) provides a peripheral with
  no host end at all.
- `sam3x8e.yaml` declares the two devices and sets `bridge: none` on SPI.

### Fixed — SPI status register

- `ArmSamSpi.fix_after_read` computed `RDRF`/`TXEMPTY`/`OVRES` into a local and
  then returned the unmodified value, so `SR.RDRF` was permanently clear and any
  firmware polling it hung. The derived bits are now merged into the returned
  value and written back.

### Fixed — properties on peripheral models

- `ArmHardwareBase.__setattr__` wrote straight into `__dict__`, bypassing data
  descriptors, so a property setter on a model silently never ran (it is how
  `peripheral.bridge = device` used to leave the old bridge in place). It now
  honours descriptors.

### Added — semihosting

- Arm semihosting is served. Firmware built with the semihosting spec files now
  gets its `printf` on the host console (or into any stream you pass) with no
  UART or serial bridge involved:
  `XuanWu(chip, code, semihosting=SemiHosting(output=stream))`.
  Implemented: `SYS_WRITEC`, `SYS_WRITE0`, `SYS_WRITE`, `SYS_READC`, `SYS_ISTTY`,
  `SYS_CLOCK`, `SYS_TIME`, `SYS_ERRNO`, `SYS_ISERROR`, `SYS_FLEN`, `SYS_SEEK`,
  `SYS_TICKFREQ`, `SYS_ELAPSED`, `SYS_GET_CMDLINE`, `SYS_HEAPINFO`, `SYS_EXIT`
  and `SYS_EXIT_EXTENDED`. Unsupported calls are logged and return `-1`.
- Unicorn reports `BKPT 0xAB` as an ordinary `BKPT` and leaves the PC on the
  instruction (it does not expose QEMU's semihosting switch), so the interrupt
  handler recognises the trap by its immediate, services it and steps over it.
  A `BKPT` with any other immediate still faults, and `semihosting=False`
  disables the service.

### Added — unclaimed-MMIO report

- `MemoryController` now counts every MMIO access that no peripheral model
  claimed, across the plain and bit-band paths, and exposes it through
  `unclaimed_accesses()` and `show_unclaimed()`. Booting a firmware and reading
  that list is the quickest way to find out which peripheral to implement next.

### Added — documentation

- `README.md` rewritten: what the project is, supported chips, install, quick
  start, GDB, semihosting, serial bridges, testing and layout.
- `docs/architecture.md` — how the layers fit together: chip descriptions,
  three-level memory routing, the peripheral framework and its hooks, name
  resolution, the interrupt engine, the GDB stub and the host backends.
- `docs/add-a-chip.md` — a step-by-step guide to supporting a new MCU, with the
  YAML field reference, a model template, the hook patterns and a checklist.
- `docs/debugging.md` — logging, state inspection, GDB, serial bridges,
  semihosting and a table of common failures.

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
