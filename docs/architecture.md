# Architecture

How xuanwu turns a chip description plus a firmware image into a running
simulation, and where to hook in when something is missing.

---

## 1. Layers

```
                       XuanWu  (src/xuanwu/xuanwu.py)
        reads the chip YAML, builds everything below, loads the firmware
   ┌──────────────┬───────────────┬──────────────┬──────────────────┐
   │ Register     │ Memory        │ Program      │ RemoteSerial     │
   │ Controller   │ Controller    │ Loader       │ Protocol         │
   │ register.py  │ memory.py     │ loader.py    │ rsp.py           │
   └──────┬───────┴───────┬───────┴──────┬───────┴────────┬─────────┘
          │               │              │                │
          │        Unicorn Uc  (mem_map / mmio_map / hooks)
          │               │
          │        ArmHardwareController  (arch/cortex_m/controller.py)
          │        ├── exception entry/exit, priorities, dispatch
          │        ├── core peripherals: SCID, SysTick, NVIC, SCB, CP, DBG, DWT
          │        └── vendor peripherals, by family
          │             ├── vendor/st/      RCC, GPIO
          │             └── vendor/atmel/   PMC, PDC, PIO, ADC, UART, SPI, PWM, UOTGHS
          │
   peripherals/ vendor-neutral behaviour cores (GPIO port, SPI/I2C master, displays)
   backends/    host-dependent code: serial bridges, semihosting
   devices/     external device models (LED, SPI flash, ILI9341) + registry
   data/        chip descriptions (YAML) and GDB target descriptions (XML)
```

Everything above `backends/` is host independent; `backends/` is the only place
that opens sockets, spawns processes or touches pty devices.

## 2. Startup

`XuanWu.__init__` does this, in order:

1. `chips.resolve_chip()` turns a chip name or a path into a YAML path.
2. `yaml.safe_load` the description; `arch` + `mode` are mapped through
   `ARCH_MAPPING` / `MODE_MAPPING` onto Unicorn and Capstone constants
   (currently `arm` + `cortex_m` → `UC_ARCH_ARM`, `UC_MODE_MCLASS|UC_MODE_THUMB`).
3. Build the `Uc`, the Capstone disassembler, the register/memory/hardware
   controllers and the (still empty) `DeviceController`.
4. `mem.map_memory(chip)` — memory regions, aliases and bit-band windows.
5. `hw.map_memory(chip)` — instantiate `type: core` peripherals, register their
   MMIO windows, and attach their clock callbacks.
6. `ProgramLoader(code).load(mem)` — write the firmware image.
7. Optionally start `RemoteSerialProtocol` on a port.

`reset()` is separate and is what you call before running: it resets every
peripheral, sets `lr = 0xFFFFFFFF`, and takes the initial `msp`/`pc` from the
vector table at `0x0` / `0x4`.

## 3. Chip descriptions

A chip YAML has a single `chip` root with `name`, `arch`, `mode`, an optional
`boot` address and a list of `peripherals`. Each entry is a one-key mapping whose
key becomes the region name (it shows up in logs and in `mem.show_map()`).

| `type` | Meaning | What the memory controller does |
|---|---|---|
| `memory` | RAM / Flash | `mem_map(base, size)` |
| `remap` | address alias (e.g. boot remapped to 0) | `mmio_map` on `alias`, reads/writes forwarded to `base` |
| `bitband_memory` | Cortex-M bit-band alias window | `mmio_map` on `alias`, bit extracted/inserted on the fly |
| `peripheral` | generic MMIO window (the private peripheral bus) | callback + byte buffer fallback |
| `bitband_peripheral` | peripheral bit-band window | both of the above, chained |
| `core` | an actual peripheral model | instantiated from the vendor registry inside an enclosing `peripheral` window |

Common fields: `base`, `size`; plus `alias` for the three alias types. `core`
entries take model-specific extras which are passed to the model's constructor
as keyword arguments (`baudrate`, `bridge`, `dma_base`/`dma_size`, `cpuid`,
`interrupt_lines`, `priority_bits`, `clock`, `cycles_per_instruction`, `calib`,
...). A `dma_base`/`dma_size` pair declares a second register window for the
peripheral's DMA block, which on the SAM3X sits 0x100 bytes after the
peripheral's own registers.

Bit-band windows never allocate real memory: the alias callback computes which
bit of which word is addressed and performs a read-modify-write through the
underlying region, so it works for RAM and for MMIO alike.

`ArmHardwareController.map_memory` additionally remaps the chip's `boot` address
to `0x0`, which is why the vector table is fetchable at address zero.

## 4. Memory routing

Three levels, in `memory.py`:

1. **`mem_map`** — plain RAM/Flash, backed by a host buffer (`uc_mem_map_ptr`). The
   buffer is what makes `remap` free: a chip that shows the same RAM at two addresses
   (the SAM3X8E does — SRAM0 at `0x20000000` and remapped at `0x20070000`, where the
   Due's linker script puts the stack) gets *one* host buffer mapped twice, so neither
   address costs a callback and writes through one are visible through the other.
   Forwarding callbacks are the fallback for a region that is not host-backed.
2. **`mmio_map`** — a callback region with a backing `bytearray`. Used for aliases,
   bit-band windows and peripheral windows.
3. **`register_io`** — inside a callback region, a second, finer registry maps
   `(address, size)` to a peripheral model's `read`/`write`. The callback looks
   the address up there first and only falls back to the byte buffer when
   nothing claims it.

That third level is what lets a single 64 KiB `peripheral` window host a dozen
peripherals that each know their own register layout.

The first level matters more than it looks: a hardware remap implemented with callbacks
costs a Python call per guest access (a microsecond each), which pinned the SAM3X8E at
8.5 M instructions/s — a fifth of the STM32F411's rate on the same host — until the
alias shared the host buffer instead.

## 5. Peripheral models

A peripheral model is a *register table* plus the mapping from register bits onto a
*vendor-neutral behaviour core*. The core owns the behaviour -- how a GPIO port
tracks levels and notifies devices, how an SPI master moves a byte, how an I2C
master runs a transaction -- and the adapter in `arch/vendor/` owns the layout, the
write-one-to-set/clear pairs, the write-protection keys and the status-bit meanings.
Supporting another vendor is then a matter of writing an adapter rather than
another copy of the protocol.

`peripherals/` holds those cores and nothing else: no base addresses, no register
names, no vendor IRQ numbers.

| Core | What it owns | What the adapter adds |
|---|---|---|
| `GpioPort` | direction, selection, driven levels, levels a device drives, the three hook granularities | `PER`/`OER`/`SODR`/`CODR`/`PDSR` and the write-protection key |
| `SpiController` | one byte out, one answer kept, overrun, chip-select mask | `CR`/`MR`/`TDR`/`RDR`/`SR`/`CSR0-3`, the NPCS decoding |
| `I2cController` | START/STOP, address and direction, ACK/NACK, internal address | the TWI register block (M6.2) |
| `I2cBus` / `I2cDevice` / `RegisterDevice` | addressing, routing, the register-pointer protocol | nothing: devices implement it |

The device layer subscribes through the cores: `GpioPort.add_hook(pin, (on_high,
on_low))` is the classic pair, `add_edge_hook(pin, fn)` passes the level, and
`add_port_hook(fn)` reports every change as `(pin, level)` -- which is what a
parallel bus needs to latch eight data lines on a strobe. `GpioPort.drive_input()`
and `release_input()` are how a device pulls an input pin, and that is what
`digitalRead` (i.e. `PDSR`) then reports.

One naming trap is worth recording, because it cost a debugging session: in the SAM
PIO, **`PSR` is the mirror of `PER`** -- which pins the PIO peripheral controls --
while **`PDSR` is the pin levels**. Deriving `PSR` from the port's levels makes the
Arduino core take a wrong branch during `setup()` and the firmware never reaches its
loop; the firmware tests caught it immediately.

`arch/base.py` provides the whole framework in about 190 lines.

```python
class ArmHardwareScb(ArmHardwareBase):
    NAME = "SCB"
    REGISTERS = (
        ("CPUID", "I", 0x00000000),     # (name, struct format, write mask)
        ("ICSR",  "I", 0x9E000000),
        ...
    )
```

`REGISTERS` is turned into an ordered mapping of `Register(format, offset, mask)`
and a flat `bytearray` of values. `read(address, size)` / `write(address, size,
data)` translate an MMIO access into the right register, honouring the write mask
and byte offsets; `read_register(name)` / `write_register(name, value)` are the
model-internal equivalents (and bypass the mask, which is how a model maintains
bits the guest cannot write).

Two hooks carry the hardware semantics:

```python
def fix_after_read(self, name, register, data) -> int
    """Adjust the value the guest reads. Read-to-clear lives here."""

def fix_before_write(self, name, register, data, data_orig) -> int
    """Adjust or veto the value being written. Write-one-to-clear, interlocks,
       write protection and status-bit coupling live here."""
```

Both receive the register name, the `Register` record and the raw value, and
return the value to use. Returning `data_orig` vetoes a write; returning `0`
makes the register appear write-only.

Common patterns from the existing models:

- **write-one-to-clear** — `NVIC.ICER`: clear the bits in the mirrored `ISER`.
- **read-only mirror** — `NVIC.fix_after_read` maps `ICER`/`ICPR` reads onto
  `ISER`/`ISPR`.
- **write protection** — a `WPMR` register whose `WPKEY` field must match, else
  the write is ignored (`PMC`, `PIO`, `ADC`, `SPI`, `PWM`).
- **status coupling** — `RCC.CR` mirrors "enable" bits into "ready" bits so
  firmware polling for a ready flag makes progress.
- **command registers** — `PMC.PCR` is a read/write command register, not
  storage.

Models that need to observe time implement `advance(instructions)` (and, if they
have a deadline of their own, `next_deadline()`); the controller calls them once
per execution slice. SysTick, the UART (which polls its host side) and the PWM use
it to advance counters. The older `system_clock_callback` is still attached as a
`UC_HOOK_CODE` callback when a model defines it, but it costs a Python call per
instruction and is deprecated.

## 6. Name resolution

Two implicit conventions are worth knowing, because both are load-bearing:

- **Vendor family** comes from `chip["name"][:3]` — `stm32f411` → `stm`,
  `sam3x8e` → `sam`. The family selects `arch/vendor/<family>/BUILDIN`.
- **Model lookup** lower-cases the region name and looks it up in the registry;
  if that misses, the trailing letter is dropped and it retries. That is how
  `GPIOA` ... `GPIOF` all resolve to the single `gpio` model, and `UART` resolves
  to `uart`.

The conformance tests exercise both rules for every bundled description, so a
typo in a chip YAML fails in seconds instead of at simulation time.

## 7. Interrupt engine

`arch/cortex_m/controller.py` implements the Armv7-M exception model on top of
Unicorn's `UC_HOOK_INTR` (for exception entry and exit) and an execution-slice
scheduler (see §7.2).

- **Entry** happens either from a peripheral asking for it
  (`set_irq_pending`) or from the guest writing `NVIC.ISPR` / `SCB.ICSR`.
  `get_next_irq()` picks the highest-priority pending interrupt that is not
  masked by `PRIMASK`, `FAULTMASK` or `BASEPRI`, taking the priority-grouping
  split into account. It is taken at the next slice boundary, which is the only
  place the state can have changed.
- **Stacking** (`push_context`) writes the eight-word exception frame, honours
  the `CCR.STKALIGN` forced alignment, records the alignment in `xpsr` **bit 9**
  and builds the matching `EXC_RETURN` value in `lr`.
- **Floating-point frame.** When the interrupted context had used the FPU
  (`CONTROL.FPCA`, which Unicorn sets by itself after a VFP instruction) the FP
  extension stacks a further 0x48 bytes *below* the integer frame: `S0`-`S15`,
  `FPSCR` and a reserved word, for a 0x68-byte frame in total. `push_context`
  writes the FP part first and starts the integer words above it; `pop_context`
  consumes it in the same order. `EXC_RETURN` bit 4 is cleared to say that a
  frame was stacked — and the validator deliberately only requires bits 31:5 to
  be ones, because requiring bit 4 to be set rejects every FP return.
- **Return** is detected as `UC_HOOK_INTR` with `intno == EXCP.EXCEPTION_EXIT`
  (the guest wrote `PC = 0xFFFFFFFx`). The controller validates the value,
  un-stacks, restores `apsr`/`ipsr`/`epsr`, updates `CONTROL.SPSEL` and thread
  mode, and **tail-chains** straight into the next pending handler when there is
  one.
- Misbehaviour that real hardware faults on is turned into a UsageFault with
  `CFSR.INVPC` set, rather than silently continuing.

Two fidelity gaps in the FP model are deliberate. Unicorn executes VFP
instructions regardless of `CPACR` and regardless of the configured CPU model
(an M0 will happily run VFP), and it exposes no VFP instruction hook, so a
`NOCP` UsageFault cannot be raised without decoding every instruction. Firmware
that relies on the FPU being *disabled* by `CPACR` will therefore run where real
hardware would fault.

Interrupt sources are registered as an `IrqOp` namedtuple
(`is_enabled`, `set_pending`, `set_active`, `get_priority`). The NVIC registers
external IRQs `0..239`; the SCB registers the system exceptions at negative
indices (`SysTick` is `-1`, `PendSV` `-2`, `NMI` `-14`, ...), which is why the
controller can treat both uniformly.

### 7.1 The time base

`arch/cortex_m/systick.py` is the only clock in the model, and it is driven by the
scheduler: `advance(instructions)` moves the counter arithmetically by a whole
slice, and `next_deadline()` reports how many instructions may still run before the
counter wraps, which is what bounds the slice. The chip YAML's `clock` gives the
core frequency the time base is derived from, and `CALIB.TENMS` follows it. A
period is `RVR + 1` cycles and the periods that elapsed are counted in one step, so
the long-run rate is exact even when `cycles_per_instruction` is fractional.

Unicorn does not report instruction costs, so `cycles_per_instruction` is an
approximation, not a measurement: the default of 1 means "one cycle per
instruction". `SysTick.cycles`, `.ticks` and `.elapsed_ms` expose the simulated
time base, which is what tests assert on instead of wall-clock time.

### 7.2 Execution slices

`XuanWu.run()` does not hand the whole budget to Unicorn. It repeatedly takes any
pending exception and then runs exactly as many instructions as
`ArmHardwareController.next_slice()` allows: the smallest of the configured
maximum slice (`max_slice`, 10 000 instructions by default), every timed model's
next deadline, and what is left of the caller's budget. The instruction count of a
slice is exact, because Unicorn's `count` is exact, so the time base never drifts.

The point is throughput. Advancing models from a `UC_HOOK_CODE` callback means one
Python call per instruction, which measured **0.85 M instructions/s**; running to
the next deadline instead measured **138 M instructions/s** for the same firmware
(Unicorn alone reaches 176 M/s). What makes that legitimate is an invariant:

> A model's state can only change through an MMIO access, a timer deadline or
> external input. All three end a slice, so between slices nothing can change and
> no exception can become pending.

The consequences worth knowing:

- An interrupt pended by an MMIO write is taken at the next slice boundary, so its
  latency is bounded by `max_slice` instructions (0.12 ms of simulated time at
  84 MHz). A deadline (SysTick, a UART polling its host) ends the slice exactly
  when it is due, so those are not delayed at all.
- `run(until=...)` needs the exact instruction count, and Unicorn cannot report how
  many it executed, so that path registers a counting hook for the duration of the
  call. It is the rare, explicit path.
- A slice may end inside a Thumb IT block. Unicorn implements the end of an IT
  block as a store into its cached IT state, which an early exit skips, so the next
  slice resumes with a stale "still in an IT block" state and rejects the following
  instruction as invalid. `repair_stale_it_state()` detects that by looking for a
  real `IT` encoding in the previous eight bytes and clears it.

## 8. GDB stub

`rsp.py` is a TCP server speaking GDB's remote serial protocol. It hooks
`UC_HOOK_CODE` for breakpoints and `UC_HOOK_MEM_READ`/`WRITE` for watchpoints and
calls `emu_stop()` when one fires.

The register layout is not hard-coded: the stub parses the bundled binutils
target descriptions into `{name: (regnum, size)}` and serves the same XML
through `qXfer:features:read:target.xml`. That is what makes a bare
`target remote` work — GDB learns the Cortex-M register set from the stub
instead of falling back to its much larger default set.

A core with an FPU reports two features: `arm-m-profile.xml` followed by
`arm-vfpv2.xml` (`d0`-`d15` and `fpscr`). GDB numbers the registers of a target
description in the order the features are listed, and expects the `g` packet in
that same order, so the parser carries an implicit counter forward from any
explicit `regnum` and the encoder emits the registers sorted by it. Whether the
feature is served is decided from the chip description: `XuanWu` looks for a core
peripheral named `fpu` and passes that to the stub.

Packet codec notes: replies may be `str` or `bytes` (single-register reads are
raw), escaping uses `}` + `c ^ 0x20`, and run-length encoding uses `*` followed
by `chr(28 + count)`. The encoder keeps that count character out of the RSP
metacharacters.

## 9. Host backends

`backends/serial_bridge.py` defines `SerialBridge` — `peer_hint`, `in_waiting`,
`read`, `write`, `reset_*_buffer`, `close` — and three implementations:

- `SocatBridge` — creates two temp files, lets `socat` turn them into a linked
  pty pair, and opens one end with pyserial. Polls for the symlink instead of
  sleeping a fixed second, and surfaces socat's own error text.
- `TcpBridge` — listens on an ephemeral port; bytes written before a peer
  connects are buffered (bounded) so short runs are not lost.
- `LoopbackBridge` — two in-memory buffers with `feed()`/`drain()` helpers.

`create_bridge("auto")` chooses `socat` when it is on `PATH` and `tcp`
otherwise. Peripherals receive the bridge name through their constructor
keyword arguments, so it can come from the chip YAML or from
`XuanWu(..., hardware_options={...})`.

`backends/semihost.py` serves Arm semihosting calls. Unicorn reports
`BKPT 0xAB` as an ordinary `BKPT` and leaves the PC on the instruction — it does
not expose QEMU's semihosting switch — so the interrupt callback recognises the
trap by its immediate, calls `SemiHosting.handle`, and advances the PC by two
bytes itself. A `BKPT` with any other immediate is left to the debugger.

## 9b. Knowing what is missing

`MemoryController` counts every MMIO access that no `register_io` record
claimed, in all four paths (plain IO, bit-band, read and write).
`unclaimed_accesses()` and `show_unclaimed()` expose it. Booting an unfamiliar
firmware and reading that list is the intended way to decide which peripheral
model to write next.

## 10. Devices

`devices/` holds models of things wired to the chip, built from the `devices:`
list in the chip description and attached right after the peripherals exist:

```python
self.dev.load(self._chip, self.box, self.reg, self.mem, self.hw)
```

Each entry needs a `type` (the registry key) and a `name`; everything else is
passed to the model's constructor. A device gets a
:class:`~xuanwu.devices.base.DeviceContext` and can:

- **watch a pin** — `ctx.gpio("GPIOB").add_hook(pin, (on_high, on_low))` (or
  `add_edge_hook` / `add_port_hook` when the level matters more than the edge). The
  GPIO model calls the first hook when a pin is driven high and the second when it is
  driven low. This is what `Led` and a chip select use.
- **take over a byte stream** — assign `peripheral.bridge = self` on a UART or
  SPI model; the model then reads and writes through the device instead of
  through `socat` or TCP. `SpiFlash` implements `SerialBridge` for exactly this.
- **share a byte stream** — `ctx.spi_bus("SPI")` returns the `SpiBusSelector` for that
  controller, and `bus.add(name, device, gpio=port, pin=cs_pin, active_low=True)` puts
  the device on it. Every byte goes to whichever device has its chip select asserted
  (sampled per byte, because firmware often writes a chip select to the level it
  already had, which raises no edge), the device is told when its transaction starts and
  ends, and bytes nobody is selected for reach the fallback bus the controller used
  before (a host bridge, usually) instead of being invented. This is what two devices
  on one controller need — the Adafruit TFT shield has an ILI9341 and a microSD socket
  on the same SPI header.

`SpiFlash` is also a worked example of a state machine driven by bus traffic: a
chip-select edge starts a transaction, the first byte after it selects a command,
address bytes follow for the commands that take one, and responses are queued for
the receive register. A real part behaves the same way, including clearing the
write-enable latch when the transaction ends.

An SPI transfer always shifts a byte in, even when nothing on the bus is driving the
data line: the master reads the idle level (`0xFF`), and the byte it keeps is the one
that was on the line *during* the transfer — the device's answer to the previous byte,
because a shift register cannot answer a byte while it is still receiving it. Reading
`n` bytes therefore costs `n + 1` clocks, which is why drivers clock a dummy byte
first, and why the SPI core tests index into the exchange instead of expecting the
interesting byte first.

Because a device is a `SerialBridge`, `create_bridge()` accepts one directly, so
a peripheral can also be handed a device explicitly:

```python
XuanWu(chip, code, hardware_options={"bridge": my_bridge})
```

Note that peripheral models override `__setattr__` to route register names to
`write_register`. It honours data descriptors, so plain properties on a model
behave normally — worth knowing before adding one.

## 11. Extension points

| Goal | Where to work |
|---|---|
| Support a new part from an existing family | a new YAML in `data/chips/<arch>/<core>/` |
| Add a peripheral model | a new module in `arch/vendor/<family>/`, registered in that package's `BUILDIN` |
| Add a whole vendor family | `arch/vendor/<family>/` with a `BUILDIN`, plus a branch in `ArmHardwareController.get_buildin` |
| Add an architecture or core | `ARCH_MAPPING`/`MODE_MAPPING` in `xuanwu.py`, a new `arch/<core>/` package, and a GDB target description XML |
| Attach something to a pin | a `Device` in `devices/`, declared in the chip YAML's `devices:` list |
| A new host transport | a `SerialBridge` subclass registered in `BRIDGE_KINDS` |

See [`add-a-chip.md`](add-a-chip.md) for the step-by-step version.
