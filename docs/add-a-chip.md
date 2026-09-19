# Adding a chip

Supporting a new MCU is three steps: describe it in YAML, implement whatever
peripheral models it needs, and add a firmware fixture. This guide walks through
all three, plus the conventions that make the existing models work.

Read [`architecture.md`](architecture.md) first if you have not; sections 3–7
there explain the machinery this guide uses.

---

## 0. Decide the scope

| Situation | Work |
|---|---|
| A new part in an existing family (e.g. another STM32F4) | YAML only, if the peripherals you need already have models |
| A part needing peripherals that do not exist yet | YAML + one module per new peripheral |
| A new silicon vendor | the above, plus `arch/vendor/<family>/` and a branch in `ArmHardwareController.get_buildin` |
| A new architecture or core | the above, plus mappings in `xuanwu.py`, an `arch/<core>/` package and a GDB target description |

The rest of this document assumes the first or second case.

## 1. Write the chip description

Create `src/xuanwu/data/chips/<arch>/<core>/<name>.yaml`. The file name (without
`.yaml`) is what users pass as the chip name.

```yaml
chip:
  name: stm32f103
  arch: arm
  mode: cortex_m
  boot: 0x08000000
  peripherals:
    - "Flash":
        type: memory
        base: 0x08000000
        size: 0x10000
    - SRAM:
        type: memory
        base: 0x20000000
        size: 0x5000
    - PPB:                      # the private peripheral bus must be mapped once
        type: peripheral
        base: 0xE0000000
        size: 0x10000
    - SCID:
        type: core
        base: 0xE000E000
        size: 0x10
        interrupt_lines: 7
    - SYSTICK:
        type: core
        base: 0xE000E010
        size: 0x10
    - NVIC:
        type: core
        base: 0xE000E100
        size: 0x3F0
        interrupt_lines: 7
        priority_bits: 4
    - SCB:
        type: core
        base: 0xE000ED00
        size: 0x40
        cpuid: 0x410FC231
    - FPU:                      # only for a core with the FP extension
        type: core
        base: 0xE000EF30
        size: 0x1C
    - GPIOA:
        type: core
        base: 0x40010800
        size: 0x1C
```

Things that trip people up:

- **`name` drives the vendor family.** `chip["name"][:3]` selects the
  registry, so `stm32f103` → `stm`, `sam3x8e` → `sam`. Get that wrong and every
  `core` peripheral fails to resolve.
- **Declare the FPU block to get floating point.** With `- FPU: {type: core, ...}`
  the core stacks and restores the extended exception frame and the GDB stub
  describes `d0`-`d15`/`fpscr`. Without it the FP register file still exists and
  VFP instructions still execute (Unicorn does not gate them), but the frame is
  not stacked, so values held in `S0`-`S15` are lost across an interrupt. The
  base address and size above are the architectural ones; the name has to be
  `FPU`, which is what `XuanWu` looks for.
- **`core` peripherals must sit inside a `peripheral` (or
  `bitband_peripheral`) window.** They register a sub-range of it; the
  conformance tests check this.
- **Regions must not overlap**, with the exception of alias types, whose
  `alias` is what actually gets mapped.
- **`size` must be at least as large as the register file the model defines.**
  A model with nine 32-bit registers needs `size >= 0x24`.
- **`interrupt_lines`** is `(number of NVIC ISER registers) - 1`; `7` gives the
  usual 8 registers / 240 IRQs. `priority_bits` is the implemented priority
  width (4 on most parts).

Supported `type` values and their fields are tabulated in
[`architecture.md` §3](architecture.md#3-chip-descriptions).

## 2. Validate the description

Before writing any Python:

```bash
pytest tests/conformance -v
```

These tests check, for every bundled YAML: known `arch`/`mode`, a resolvable
vendor family, known device types with the required fields, no overlapping
mapped regions, `core` peripherals inside a mapped window, a mapped `boot`
address, and that every `core` name resolves to a model. They take under a
second and catch most mistakes.

Then try to construct it:

```python
from xuanwu import XuanWu
device = XuanWu("stm32f103", "firmware.elf")
device.mem.show_map()
```

## 3. Implement missing peripheral models

A model lives in `arch/vendor/<family>/<peripheral>.py`. The smallest useful one
is a register table plus a reset:

```python
# -*- coding: utf-8 -*-

"""Serial peripheral interface."""

from typing import Any

from ....config import logger
from ...base import ArmHardwareBase, Register

__all__ = ["ArmStmUsart"]


class ArmStmUsart(ArmHardwareBase):
    """Universal synchronous asynchronous receiver transmitter"""

    NAME = "USART"
    REGISTERS = (
        ("SR",   "I", 0xFFFFFFFF),
        ("DR",   "I", 0xFFFFFFFF),
        ("BRR",  "I", 0x0000FFFF),
        ("CR1",  "I", 0xFFFFFFFF),
        ("CR2",  "I", 0x0000FFFF),
        ("CR3",  "I", 0xFFFFFFFF),
        ("GTPR", "I", 0x0000FFFF),
    )

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._irq = kwargs.get("irq", -1)
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("SR", 0x00C0_0000)   # TXE and TC set
        self.write_register("CR1", 0x0)
        ...
```

### 3.1 The register table

`REGISTERS` is a tuple of `(name, format, write_mask)`:

- **name** — used by `read_register`/`write_register` and by the hooks. Use
  `RESERVEDn` for gaps; the format's repeat count covers them (`"2I"` is two
  words).
- **format** — a `struct` format code: `B`, `H`, `I` (and repeats such as
  `"38I"`). Offsets are computed from the previous entries, so the tuple order
  must match the datasheet layout exactly.
- **write_mask** — the bits the guest is allowed to affect. `0xFFFFFFFF` means
  "all bits writable"; `0x00000000` makes the register effectively read-only
  through MMIO. This is the mask, not the reset value.

### 3.2 Reset values

`reset()` must write every register that does not reset to zero, and is also
where a model initialises its private state:

```python
    def reset(self):
        super().reset()
        self.write_register("CPUID", self._cpuid)
        self._lock_seq = 0
```

`ArmHardwareBase.reset()` only logs the register-file size, so always call
`super().reset()` first.

### 3.3 Bit positions

Do not scatter magic numbers. Put the bit positions in an `IntEnum` next to the
model and use names:

```python
class UART_CR(IntEnum):
    RXEN = 4
    TXEN = 6
...
if cr & (1 << UART_CR.RXEN):
```

### 3.4 The fix hooks

Only assign a hook if the model needs one — the base class calls them on every
access, and an unused hook is pure overhead.

```python
    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name == "SR":
            # read-to-clear
            self.write_register("SR", data & ~(1 << USART_SR.RXNE))
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        if name == "CR1":
            self.write_register("CR1", data)          # keep private state in step
        elif name == "WPMR":
            if (data & 0xFFFFFF00) != 0x55534500:     # wrong key
                return data_orig                      # veto the write
            data &= 0x01
        return data
```

Return the value you want stored. `data_orig` is what is currently in the
register, which is what makes "write ignored" and "preserve masked bits"
possible.

For a worked, real example of a status-coupling model, read
[`src/xuanwu/arch/vendor/st/rcc.py`](../src/xuanwu/arch/vendor/st/rcc.py): it
mirrors the clock-enable bits in `CR` onto the ready bits firmware polls for,
and does the same for the clock-source selection in `CFGR`. Without that,
firmware hangs in its startup loop.

### 3.5 Interrupts

A model that raises interrupts reads its IRQ number from the constructor
arguments (so it comes from the YAML) and asks the controller to pend it:

```python
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._irq = kwargs.get("irq", -1)

    def advance(self, instructions):
        ...
        if self._imr & status:
            self._ctl.set_irq_pending(self._irq)
```

and the chip YAML carries the number:

```yaml
    - USART1:
        type: core
        base: 0x40011000
        size: 0x1C
        irq: 37
```

Any IRQ in `0..239` works, because the NVIC registers an `IrqOp` for all of them
at construction. Defining `advance(instructions)` is what makes the controller
hand your model the length of every execution slice — that is your "time passes"
hook, and it costs one call per slice instead of one per instruction. If the model
also has to be serviced at a known time, define `next_deadline()` and return how
many instructions may still run before then; the scheduler will end the slice
there. Both are documented in `docs/architecture.md` §7.1/§7.2.

### 3.6 Host-facing peripherals

If the peripheral talks to the outside world, keep the host dependency behind a
bridge rather than calling `socket`/`subprocess` directly:

```python
from ....backends import create_bridge

        self._bridge = create_bridge(
            kwargs.get("bridge", "auto"),
            baudrate=kwargs.get("baudrate", 115200),
            prefix="usart",
        )
```

and expose `peer_hint` so users can find the other end. See
[`debugging.md`](debugging.md#3-serial-ports).

### 3.7 Register the model

Add it to the family's registry:

```python
# arch/vendor/st/__init__.py
from .usart import ArmStmUsart

BUILDIN = {
    "rcc": ArmStmRcc,
    "gpio": ArmStmGpio,
    "usart": ArmStmUsart,
}
```

The key is the lower-cased peripheral name from the YAML. The lookup also tries
the name with its trailing letter removed, which is how `GPIOA` ... `GPIOF` all
land on `gpio`; use that only for genuinely identical instances.

## 4. Add a firmware fixture and a test

Firmware is built from Arduino sketches, and `tests/firmware/board.yaml` is the
matrix that says which sketch builds which board and which chip description runs
it:

```yaml
  - name: stm32f103
    board: Generic STM32F1 series
    fqbn: STMicroelectronics:stm32:GenF1:pnum=GENERIC_F103C8TX
    chip: stm32f103            # the description you just wrote
    sketch: Blink_f103
    output: stm32f103
```

Write the sketch as `tests/firmware/Blink_f103/Blink_f103.ino` (directory and file
name must match), then:

```bash
python tests/firmware/build.py stm32f103   # builds it into tests/firmware/stm32f103/
```

Commit the `.elf` next to the other boards so the suite needs no toolchain. The
integration suite then picks it up automatically, and
`tests/conformance/test_firmware_matrix.py` checks that the sketch, the FQBN, the
chip name and the output directory all agree.

`board.yaml` is the authority rather than the image's load address, because two
unrelated parts can share one: the STM32F411 and the STM32F767 both run from
`0x08000000`. The load address is only a fallback, via `CHIP_BY_LOAD_ADDRESS` in
`tests/conftest.py`, for hand-written firmware such as
`tests/firmware/stm32f411/fpu_test/`.

Then:

```bash
pytest tests/integration -v
```

Every firmware gets a smoke test (construct, reset, run 200k instructions) for
free. Add targeted tests for new behaviour next to the existing ones:

- a register-level test if the model has interesting semantics;
- an end-to-end test if it talks to the host, following
  `tests/integration/test_uart_e2e.py`, which uses the loopback and TCP bridges
  so it runs without `socat`.

## 5. Declare external devices

If the board has something wired to the chip, say so in the `devices:` list —
`device.py` builds them once the peripherals exist:

```yaml
chip:
  ...
  devices:
    - name: LED
      type: led
      port: GPIOB
      pin: 27
    - name: FLASH
      type: spi_flash
      port: SPI
      cs: {port: GPIOC, pin: 26, active_low: true}
      size: 0x100000
```

| `type` | Fields | What it does |
|---|---|---|
| `led` | `port`, `pin`, `active_low` | records the level of one GPIO pin (`state`, `transitions`) |
| `spi_flash` | `port`, `cs` (`port`/`pin`/`active_low`), `size` | answers SPI NOR commands on a port, framed by the `cs` pin |

Every entry needs `type` and `name`; the rest is passed to the model's
constructor, so a model can take extra options without touching the loader.

A device attaches through one of two mechanisms (see
[`architecture.md` §10](architecture.md#10-devices)):

```python
# 1. watch a pin
ctx.peripheral(self.port).add_hook(self.pin, (self._on_high, self._on_low))

# 2. take over a peripheral's byte stream
ctx.peripheral(self.port).bridge = self      # self is a SerialBridge
```

To add a **new device model**: subclass `devices.base.Device`, set `type`, and
register it in `devices/__init__.py`'s `BUILDIN`. If it talks over a serial port,
also inherit `SerialBridge` from `xuanwu.backends` — that is the only interface
the UART and SPI models know about.

## 6. Checklist

- [ ] YAML under `data/chips/<arch>/<core>/`, file name == chip name
- [ ] `name[:3]` selects an existing vendor family
- [ ] `pytest tests/conformance -v` passes
- [ ] `XuanWu("<name>", firmware)` constructs and `mem.show_map()` looks right
- [ ] `pytest tests/integration -v` passes
- [ ] new models registered in the family's `BUILDIN`
- [ ] external devices listed under `devices:` with a `type` and a `name`
- [ ] `- FPU: {type: core, base: 0xE000EF30, size: 0x1C}` if the core has an FPU
- [ ] bit positions in an `IntEnum`, not inline
- [ ] any known approximation noted in a comment or in the CHANGELOG

## Known rough edges

Worth knowing before you rely on a particular area:

- **SysTick counts instructions, not real cycles.** Unicorn does not report what an
  instruction cost, so the counter advances once per executed instruction, scaled by
  `cycles_per_instruction` (default 1, the "one cycle per instruction" approximation).
  `millis()` therefore tracks *simulated* time, and its ratio to wall-clock time
  depends on how fast the host executes. A chip that wants a firmware's millisecond
  tick to cost fewer emulated instructions raises the factor; see the `clock` and
  `cycles_per_instruction` fields in `stm32f411.yaml`.
- **`CPACR` does not gate the FPU.** Unicorn executes VFP instructions whatever
  `CPACR` says and whatever CPU model is configured, and it offers no VFP
  instruction hook, so a `NOCP` UsageFault cannot be raised. Declaring the `FPU`
  block gets you the FP exception frame and the debugger description; it does not
  make the FPU switchable.
- **The device layer has two models** (`led`, `spi_flash`) and no I2C bus, so a
  board whose sensors sit on I2C needs that peripheral and device first.
- **`dma_base` support is SAM-specific** (`PDC`). A peripheral's DMA block is a
  second register window declared with `dma_base`/`dma_size`; on the SAM3X it sits
  0x100 bytes after the peripheral's own registers, which is why the conformance
  suite checks that it is mapped and does not overlap the main block rather than
  that it fits inside it.
