# Changelog

All notable changes to this project are documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added — the ILI9341 model, and a real Adafruit library drawing on it

- `src/xuanwu/peripherals/display/`: the panel side of a TFT, with no transport in it.
  `DisplaySurface` is an RGB565 framebuffer with dirty-rectangle tracking, a hash, a
  histogram, a bounding box and a PNG writer (pure Python -- no `Pillow` needed to look
  at what a firmware painted).  `Ili9341` is the controller: command set, address
  windows with wrap-around, `RAMWR`/`RAMWRC`, the GRAM, `MADCTL` rotation applied to
  both the address counter and the image a viewer sees, `COLMOD`, sleep/display/
  inversion state, the `0xD9` index-register protocol `readcommand8()` uses, and pixel
  read-back.
- `devices/display/`: `Ili9341Device` wires that to a shared SPI bus and a D/C pin and
  keeps a viewer up to date; `Viewer` has a `headless` implementation (default, used by
  the tests) and a `pygame` one behind the optional extra `xuanwu[gui]`.
- `SpiBusSelector` (`peripherals/bus/spi.py`): several devices on one SPI controller,
  each with its own GPIO chip select, which is what the Adafruit TFT shield needs -- it
  carries an ILI9341 on D10 *and* a microSD socket on D4 on the same SPI header.  Bytes
  go to whoever is selected and to nobody when none is; a device is told when its
  transaction starts and ends, because that is where a flash resets its command latch
  and a card ends its frame.  `SpiFlash` now joins the bus instead of taking the whole
  controller over, so two devices can share one.
- Chip descriptions can build on another one with `include:`, and the shield is
  `sam3x8e_tft.yaml`: a handful of lines over `sam3x8e.yaml` instead of a copy of the
  part.  The board's Arduino pin numbers are translated there (D9 = PC21, D10 = PC29,
  D4 = PC26 -- the Due's numbering is *not* the port bit, and getting it wrong is
  silent).
- `tests/firmware/graphicstest_due_tft/` is Adafruit's own `graphicstest` built for the
  Due against pinned library versions (`Adafruit ILI9341 1.6.0`, `Adafruit GFX 1.12.6`,
  `Adafruit BusIO 1.17.4`, installed by `docker/Dockerfile` with `--no-deps` so the
  touch controllers this board does not have are left out).  It runs to `Done!` and
  prints the benchmark table, and its final frame is pinned as a golden digest with a
  reviewed PNG in `docs/images/due_tft_graphicstest.png`.
- `tests/firmware/Tft_smoke_m3/` uses the same libraries on a few known shapes so the
  whole path can be checked pixel by pixel in seconds: a filled rectangle, an outline,
  a fast line, a single pixel, text, and a marker drawn in landscape.  The drawing path
  is verified against the coordinates the sketch used, not against itself -- including
  that the filled rectangle is exactly 30x40 pixels and the outline exactly its
  perimeter.
- New tests: `tests/unit/test_display_surface.py`, `tests/unit/test_ili9341.py`,
  `tests/unit/test_spi_selector.py`, `tests/integration/test_display_layer.py` (the
  shield driven through memory-mapped registers), `tests/integration/test_due_tft.py`
  and `tests/integration/test_due_tft_milestone.py` (the full run, marked `milestone`
  and deselected by default: it takes about three minutes).

### Fixed — guest memory cost a Python callback per access on the SAM3X8E

- The SAM3X8E maps SRAM0 twice: at `0x20000000`, and remapped at `0x20070000`, which is
  where the Due's linker script puts the stack and heap.  The alias was implemented with
  forwarding callbacks, so *every* stack access in *every* SAM firmware went through
  Python: about a microsecond each, which pinned the whole simulator at 8.5 M
  instructions/s and made the TFT work impossible.
- `type: memory` regions are now backed by host memory (`uc_mem_map_ptr`) and a `remap`
  maps the *same* host buffer at the second address, which is what the hardware does.
  Nothing on the host is involved in either view, and writes through one are visible
  through the other.  Measured on `Blink_m3`: **8.5 → 189 M instructions/s (22x)**;
  `Blink_uart_m3` 157 M/s, and the graphicstest firmware 12 M/s (the rest is SPI
  traffic, see below).

### Fixed — an SPI transfer that nothing answers still shifts a byte in

- `SPI.transfer()` in the SAM core polls the receive flag after *every* byte it sends,
  so a controller that only answers when a device has something to say hangs the first
  write to a write-only part.  With nothing driving the data line the master reads the
  idle level (`0xFF`), and that is what the model returns now.
- The byte kept for a read is the one that was on the line *during* the transfer, i.e.
  the device's answer to the *previous* byte -- a shift register cannot answer a byte
  while it is still receiving it.  Reading `n` bytes therefore takes `n + 1` clocks,
  which is why drivers clock a dummy byte first; the tests say so explicitly instead of
  hiding it.

### Changed — the SAM UART is polled once per character time, not every 512 instructions

- Every poll of the host side ends an execution slice, so a firmware with `Serial`
  open was paying Python work thousands of times per simulated millisecond for input
  that could not have arrived yet.  The interval is now derived from the peripheral
  clock and the baud rate (about 7300 instructions at 115200 baud, one character),
  which is as prompt as the line can be, and it is what the chip description's
  `clock:`/`baudrate:` are for.
- The chip-select level a shared SPI bus reads is also sampled per byte rather than
  trusted from the last edge, because firmware routinely writes a chip select to the
  level it already had -- which raises no edge at all.

### Added — chip descriptions can include another one

- `include:` merges a second description underneath the current file (scalars from the
  including file win; `peripherals` and `devices` are concatenated), relative to the
  file or by bundled chip name, with cycles reported as an error.  The conformance
  checks load descriptions the same way the simulator does, so an overlay that failed
  to inherit the part cannot pass vacuously.

### Added — the SAM3X8E address map is audited against the datasheet

- `tests/conformance/test_sam3x8e_map.py` checks every peripheral base and size in
  `sam3x8e.yaml` against the values taken from the datasheet, plus the peripherals
  that are deliberately *not* modelled (HSMCI, SSC, SPI1, TC0-2, USART0-3, EMAC,
  CAN0/1, TRNG, DMAC, DACC, SMC, SDRAMC, MATRIX, CHIPID, RSTC, SUPC, RTTC, WDT, RTT,
  GPBR) so a missing model stays a decision instead of an oversight. It also records
  why `UOTGHS` is the *device* block at `0x400AC800` and not the general block at
  `0x400AC000`: a booted firmware polls the device block for `CLKUSABLE`.
- `ArmSamPmc`'s offsets were checked in the process: `PCSR0` is at `0x18`, `SR` at
  `0x68` and `PCER1`/`PCSR1` at `0x100`/`0x108`.



- `ArmSamTwi` (`arch/vendor/atmel/twi.py`) over the vendor-neutral `I2cController`:
  `CR`/`MMR`/`SMR`/`IADR`/`CWGR`/`SR`/`IER`/`IDR`/`IMR`/`RHR`/`THR`, with `SR` built
  from the core (`TXCOMP|TXRDY` when idle, `RXRDY` after a byte arrives, `NACK` when
  nobody answers, `SVREAD`/`SVACC`/`EOSACC` during a transaction) and `PW_*` reported
  as always ready because this model never stretches the clock.
- TWI0 (`0x4008C000`) and TWI1 (`0x40090000`) are in the chip description with their
  IRQ numbers (22/23), which is what `Wire1` and `Wire` address on a Due.
- The ASF sequence the Arduino `Wire` library drives -- write `MMR`, write `IADR`, the
  first `THR` write *starts* the transfer, poll `SR.TXRDY` per byte, `CR.STOP`, poll
  `SR.TXCOMP`; for a read, `MMR|MREAD`, `CR.START`, poll `SR.RXRDY`, read `RHR` --
  is covered byte by byte in `tests/unit/test_twi.py`.
- `ArmSamEfc` covers `FMR`/`FCR`/`FSR`/`FRR`/`FVR` for both banks, so the flash wait
  states the Arduino core sets no longer land on an unclaimed address. `FSR` always
  reports `FRDY`.

### Changed — `SpiBus` is a protocol, not a base class

- A bus and a device on it now only have to have the same *shape*
  (`write`/`read`/`in_waiting`), which is what the serial bridges in `backends`
  already had. Declaring it `@runtime_checkable` means a flash chip and a host pty
  are the same thing to the controller, and neither inherits from the other.

### Fixed — the PMC reported PLLA as locked by the wrong field

- `LOCKA` was derived from the multiplier field of `CKGR_PLLAR`, so a valid
  `ONE | DIVA=1 | MULA=0` write (divide by one) looked unlocked while a write that
  leaves the PLL switched off looked locked. It now follows `DIVA != 0` and is only
  touched by a write with the `ONE` bit set -- which is also what makes selecting
  PLLA as the master clock before bringing it up correctly *never* report `MCKRDY`
  (a real firmware would hang there too). Found while writing the clock-switch test.

### Fixed — `SCB.set_active()` wrote the opposite bit

- `state=True` took the clear-this-bit branch, so `SHCSR.SYSTICKACT` and
  `ICSR.PENDSTSET` read back inverted and the `pend` term of the Arduino SAM
  `micros()` was computed from the wrong value. Three regression tests pin the bit
  directions down; the `micros()` tests had to use a self-spinning program rather
  than a firmware, because a firmware configures SysTick itself and hides the bug.

### Fixed — a software reset on SPI returned to master mode

- `CR.SWRST` clears `MR`, including `MSTR`, but the adapter wrote `MR` directly,
  bypassing the hook that keeps the core in step, so the controller still believed it
  was the master. It now tells the core as well.

### Added — vendor-neutral behaviour cores

- New `src/xuanwu/peripherals/` package: the protocol lives there, the register
  layout stays in `arch/vendor/`. A model is now a register table plus the mapping
  from its bits onto a core, so supporting another vendor means writing an adapter
  instead of another copy of the protocol.
  - `GpioPort` — direction, selection, driven levels, **levels a device drives on an
    input pin** (so `digitalRead`/`PDSR` see them), pull-ups, and three hook
    granularities: the classic `(on_high, on_low)` pair, `add_edge_hook(pin, fn)`
    with the level, and `add_port_hook(fn)` reporting every change as
    `(pin, level)` — which is what a parallel bus needs to latch eight data lines.
  - `SpiController`/`SpiBus` — one byte out with the answer kept until read
    (including the overrun case), chip-select mask, and the variant-independent
    status the adapter maps onto its own bits.
  - `I2cController`/`I2cBus`/`I2cDevice`/`RegisterDevice` — the START/address/ACK/
    STOP sequence, the internal-address register emitted the way the hardware does
    it (pointer bytes, then a repeated START for a read), and a base class for the
    common "first written byte selects a register" part. Used by the SAM TWI adapter
    in M6.2; the core and its tests land here.
- `ArmSamGpio` and `ArmSamSpi` are adapters over those cores now. `PDSR` is derived
  from the port, so a device-driven input is visible to the guest.
- 26 unit tests in `tests/unit/test_peripheral_cores.py` cover the cores with no
  chip in sight: no addresses, no register names, no vendor IRQ numbers.

### Fixed — `PSR` is not the pin levels

- The first version of the GPIO adapter derived `PSR` from the port's levels. In the
  SAM PIO, `PSR` is the **mirror of `PER`** (which pins the PIO peripheral controls)
  and `PDSR` is the pin data; the Arduino core reads `PSR` during `setup()`, took a
  different branch, and the firmware never reached its loop — the LED stopped
  toggling. Found by the device-layer test that runs the real firmware, which is
  exactly what it is for.

### Changed — execution is sliced instead of hooked per instruction

- **Throughput went from 0.85 M to 138.6 M instructions/s** (163x) for the same
  firmware; Unicorn alone, with no hooks at all, reaches 176 M/s. The whole suite
  went from 70 s to 23 s. Measured on `stm32x411` + `Blink_m4.ino.elf`.
- The interrupt engine no longer dispatches from a `UC_HOOK_CODE` callback. A model
  can only change through an MMIO access, a timer deadline or external input, and
  all three end an execution slice, so `XuanWu.run()` now takes pending exceptions
  and then runs exactly `ArmHardwareController.next_slice()` instructions -- the
  smallest of `max_slice` (10 000 by default), every timed model's next deadline and
  what is left of the budget. An interrupt pended by a register write is therefore
  taken at the next boundary, within `max_slice` instructions (0.12 ms of simulated
  time at 84 MHz); a deadline-driven one (SysTick) is not delayed at all.
- Models observe time through a new `advance(instructions)` / `next_deadline()`
  pair instead of a per-instruction callback. SysTick and the PWM were migrated;
  the SAM UART now samples its host side once per slice (and on `SR` reads), and
  `next_deadline()` reports a poll interval so external input latency stays bounded.
- Slicing can end inside a Thumb IT block, and Unicorn implements the end of an IT
  block as a store into its cached IT state that an early exit skips -- the next
  slice then resumed with a stale "still in an IT block" state and rejected the
  following instruction as invalid. `repair_stale_it_state()` looks for a real `IT`
  encoding in the previous eight bytes and, if there is none, clears the leftover.
- `run(until=...)` keeps a counting hook for the duration of that call, because
  Unicorn cannot report how many instructions it executed and the time base has to
  stay exact.

### Fixed — `SPREALIGN` was written into the Thumb IT state

- `push_context` computed the forced-alignment flag as the *mask* (`0` or `4`) and
  then shifted it by 9, writing `0x800` into the stacked xPSR. Bit 9 is SPREALIGN;
  bit 11 is `IT[3]`. Every exception therefore corrupted the IT state of the
  interrupted context, and a conditional branch after the return was rejected as an
  invalid instruction. `pop_context` reads bit 9, so the alignment bit itself was
  never read back correctly either. Found by running the RT-Thread firmware under
  sliced execution, which moved the boundary onto the corrupted path.
- `ArmHardwareBase.next_deadline()` has a default of "never", so a model that only
  implements `advance()` cannot be caught out by the scheduler.

### Changed — SPI register reads cost less

- `ArmSamSpi` fetches the byte a transfer returned when `TDR` is written, instead of
  leaving it in the device until `RDR` is read. Firmware polls `SR` before *and*
  after every byte, and `SR.RDRF` was derived from the device each time; it is now
  derived from the prefetched byte, so the hot path no longer calls into the device
  at all. An unread byte is kept, which is the overrun case.

### Added — the SAM3X8E + TFT shield plan

- `docs/plan-sam3x8e-tft.md`: bringing up the Arduino Due with the Adafruit 2.8"
  TFT Touch Shield v2 (ILI9341 over SPI, FT6206 over I2C, microSD over SPI), the
  peripheral work it needs, the sub-projects that simulate the display/touch/storage,
  the firmware ladder and the measured numbers behind the ordering.

### Added — the test firmware is built from sketches

- **Arduino CLI replaces the system Arm compiler.** `docker/Dockerfile` installs the
  Arduino CLI and the cores for the boards the test firmware targets, and each core
  brings its own toolchain, so no distribution `gcc-arm-none-eabi` is needed. Four
  images now build from a sketch in the repository:
  `stm32f411` (Generic F411CEUx), `sam3x8e` (Arduino Due), `mkrzero` (Arduino MKR
  Zero, SAMD21/Cortex-M0+) and `nucleo_f767zi` (ST Nucleo-F767ZI,
  STM32F767/Cortex-M7). The first two are run by the suite; the other two are
  built and committed as the physical entry point for supporting those parts.
- `tests/firmware/board.yaml` is the matrix (sketch → board FQBN → chip
  description) and `tests/firmware/build.py` drives `arduino-cli` from it:
  `python tests/firmware/build.py [board...]`, `--list` to see it without building.
- `tests/firmware/Blink_uart_m3/Blink_uart_m3.ino` is the Due sketch that used to
  sit loose in `sam3x8e/`; the Arduino CLI only builds `X/X.ino`, so it moved into
  a sketch directory. `Blink_m4.ino` and `Blink_f767zi.ino`/`Blink_mkrzero.ino` are
  new — the STM32F411 image's source had been lost, so it can be rebuilt now.
- The image is chosen by `board.yaml`, not by the load address: the STM32F411 and
  the STM32F767 both run from `0x08000000`, so an address match would have run the
  Cortex-M7 firmware against the Cortex-M4 description. The address stays as a
  fallback for hand-written firmware, and
  `tests/conformance/test_firmware_matrix.py` checks that every bundled `.elf`
  lives under a declared output directory, that the sketches exist, and that the
  boards which do have a chip description are the ones the suite parametrises.
- `fpu_test/build.sh` finds its toolchain in `$CC`, on `PATH`, or in the Arduino
  STM32 core (asking `arduino-cli` where its data directory is), so the hand-written
  FP firmware rebuilds in the image too. Verified with no distribution
  `arm-none-eabi-gcc` present.
- `.map` files and `*.with_bootloader.*` images are ignored; the `.elf`/`.bin`/`.hex`
  stay committed so running the tests needs no toolchain.
- `requirements-dev.txt` now lists what the suite actually needs to run: `pyflakes`
  and, importantly, `setuptools`/`wheel`. A fresh `python:3.12` image has neither,
  because `pip install .` builds in an isolated environment — so
  `test_setup_py_runs_without_the_runtime_dependencies` failed in a clean container
  and passed in the hand-maintained one. Found by building the image.
- `.dockerignore`: the build context was the whole repository, including several
  megabytes of prebuilt firmware. The image only copies `README.md`, `MANIFEST.in`,
  `setup.py`, `requirements*.txt`, `src/` and `docs/`.
- The updated image was built and verified from scratch: the three cores are
  present, there is no distribution Arm compiler, `fpu_test/build.sh` finds the
  Arduino toolchain, all four sketches rebuild, and the full suite passes (276).

### Added — a Chinese manual and runnable examples

- `docs/manual.zh-CN.md` (中文使用手册): install, first run, the core API, serial
  bridges, GDB, semihosting, floating point and the time base, external devices,
  building test firmware with the Arduino CLI, the test suite, adding a chip and a
  troubleshooting FAQ — with the real output of every command in it.
- Five new scripts under `examples/`, each also a test
  (`tests/integration/test_examples.py`, 9 cases): `uart_bridge.py` (read what the
  firmware prints on its UART over the TCP bridge), `semihosting.py` (a `BKPT 0xAB`
  trap serviced and captured), `fpu.py` (the FP frame with its negative control),
  `devices.py` (the LED and the SPI flash driven from Python) and `unclaimed_io.py`
  (the work list for an unimplemented chip).
- `tests/unit/test_docs.py` keeps the documentation honest: relative links resolve,
  paths in backticks exist, every example is mentioned in the manual, and the
  manual's table of contents matches its headings.

### Changed — history cleanup

- `arch/base.py` no longer does `from unicorn.arm_const import *`: the register
  table uses the `uc_arm` alias it already imported, which removes 31 pyflakes
  warnings and one silent source of "works until the name is misspelled".
- `config/__init__.py` imports `LOGGING_CONFIG` by name instead of relying on a
  star import, so a typo would be an error rather than a `NameError` at import.
- Removed unused imports from `chips.py` (typing.Optional), `loader.py`
  (config.logger) and `arch/base.py` (typing.Dict/Tuple). What is left is the 14
  pyflakes lines for the five deliberate re-export modules
  (`arch/__init__.py`, `arch/armv7m.py`, `arch/atmel_sam.py`, `arch/stm_stm.py`,
  `config/__init__.py`), which pair a star import with an explicit `__all__`.

### Changed — the SysTick time base is explicit instead of a magic step

- SysTick's counter used to be decremented by a bare per-chip `step` (128 on the
  STM32F411, unset elsewhere) with no stated meaning. It now has a documented
  model: the counter advances once per executed instruction by
  `cycles_per_instruction` (default 1, "one cycle per instruction"), a period is
  `RVR + 1` cycles, and the elapsed periods are counted in one step so a
  fractional factor keeps the long-run rate exact. `step` is still accepted as an
  alias.
- New per-chip `clock` field (STM32F411: 100 MHz, SAM3X8E: 84 MHz). The
  `CALIB.TENMS` value is derived from it instead of being a hard-coded 0x2904 that
  corresponded to no real clock, and can still be overridden per chip with
  `calib`.
- `SysTick.cycles`, `SysTick.ticks` and `SysTick.elapsed_ms` expose the simulated
  time base, so tests can assert on simulated time instead of wall-clock time.
- `CSR.COUNTFLAG` is no longer writable by the guest, `CVR` reads back as the
  running down-count, and writing `CVR` restarts the period.

### Fixed — `ArmHardwareNvic.REGISTERS` was a class attribute

- The register table depends on `interrupt_lines`/`priority_bits` and was assigned
  to the *class* from `__init__`, so an already-created controller reported
  whichever table was built last as soon as a second one existed. It is now an
  instance attribute; nothing is cached on the class.
- While building that table the priority mask was computed into `mask`, which the
  following `for ... in REGISTERS_TEMPLATE` loop immediately reused as its loop
  variable: `priority_bits` had no effect on the `IPR` registers. The mask is now
  applied, so an unimplemented priority bit cannot be written (the SAM3X8E and
  STM32F411 both use four implemented bits).

### Fixed — the conformance check for DMA windows was wrong

- It required a `dma_base`/`dma_size` window to lie *inside* the peripheral's own
  block, which cannot hold: on the SAM3X the PDC sits 0x100 bytes after the
  peripheral's registers (the UART's is at 0x400E0900 while the UART block the
  model implements ends at 0x400E0824), and an overlapping IO window is rejected
  by the memory controller at access time. The suite now checks that the window is
  mapped, non-empty, and does not overlap the main block — and the recorded
  expected failure is gone.

### Added — line-ending policy

- `.gitattributes` pins `* text=auto eol=lf` and marks the firmware images as
  binary. The repository is edited on Windows and tested in a Linux container, and
  without it every checkout from one side showed up as a whole-file diff on the
  other.

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
