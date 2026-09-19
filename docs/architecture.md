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
   backends/    host-dependent code: serial bridges, semihosting
   devices/     external device models (LED, SPI flash) + registry
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
`interrupt_lines`, `priority_bits`, `step`, `calib`, ...).

Bit-band windows never allocate real memory: the alias callback computes which
bit of which word is addressed and performs a read-modify-write through the
underlying region, so it works for RAM and for MMIO alike.

`ArmHardwareController.map_memory` additionally remaps the chip's `boot` address
to `0x0`, which is why the vector table is fetchable at address zero.

## 4. Memory routing

Three levels, in `memory.py`:

1. **`mem_map`** — plain RAM/Flash. `MemoryController` keeps a registry of
   `(start, end, perms, buffer, desc)` so it can answer "what is mapped here".
2. **`mmio_map`** — a callback region with a backing `bytearray`. Used for
   aliases, bit-band windows and peripheral windows.
3. **`register_io`** — inside a callback region, a second, finer registry maps
   `(address, size)` to a peripheral model's `read`/`write`. The callback looks
   the address up there first and only falls back to the byte buffer when
   nothing claims it.

That third level is what lets a single 64 KiB `peripheral` window host a dozen
peripherals that each know their own register layout.

## 5. Peripheral models

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

Models that need to observe time pass `system_clock_callback`, which the
hardware controller attaches as a `UC_HOOK_CODE` callback; SysTick and the PWM
use it to advance counters.

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
Unicorn's `UC_HOOK_INTR` and `UC_HOOK_CODE`.

- **Entry** happens either from a peripheral asking for it
  (`set_irq_pending`) or from the guest writing `NVIC.ISPR` / `SCB.ICSR`.
  `get_next_irq()` picks the highest-priority pending interrupt that is not
  masked by `PRIMASK`, `FAULTMASK` or `BASEPRI`, taking the priority-grouping
  split into account.
- **Stacking** (`push_context`) writes the eight-word exception frame, honours
  the `CCR.STKALIGN` forced alignment, records the alignment in `xpsr` bit 9 and
  builds the matching `EXC_RETURN` value in `lr`.
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

- **watch a pin** — `ctx.peripheral("GPIOB").add_hook(pin, (on_high, on_low))`.
  The GPIO model calls the first hook when a pin is driven high and the second
  when it is driven low. This is what `Led` and the SPI flash's chip select use.
- **take over a byte stream** — assign `peripheral.bridge = self` on a UART or
  SPI model; the model then reads and writes through the device instead of
  through `socat` or TCP. `SpiFlash` implements `SerialBridge` for exactly this.

`SpiFlash` is also a worked example of a state machine driven by bus traffic: a
chip-select edge starts a transaction, the first byte after it selects a command,
address bytes follow for the commands that take one, and responses are queued for
the receive register. A real part behaves the same way, including clearing the
write-enable latch when the transaction ends.

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
