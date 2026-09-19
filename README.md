# xuanwu

A micro-controller simulator: run firmware built for real MCUs on your PC, with a
GDB stub and a bridged serial port.

xuanwu uses [Unicorn](https://github.com/unicorn-engine/unicorn) as the CPU core
and [Capstone](https://github.com/capstone-engine/capstone) for disassembly, then
adds the parts a bare instruction emulator does not have: a memory map, peripheral
models, a Cortex-M exception/interrupt engine, a debug stub and a serial bridge.

Chips are described in YAML rather than coded in, so supporting a new part is
mostly a matter of writing a description and, where needed, one peripheral model.

---

## Features

- **YAML chip descriptions** — memory map, aliases and peripheral blocks live in
  `src/xuanwu/data/chips/`, not in code.
- **Cortex-M interrupt engine** — exception entry/exit, tail-chaining, nested
  preemption, `EXC_RETURN` validation, `PRIMASK`/`FAULTMASK`/`BASEPRI` masking.
- **Floating point** — VFP code runs and the extended exception frame is stacked
  and restored, so values held in S0-S15 survive an interrupt. The FP system
  registers (`FPCCR`, `FPCAR`, `FPDSCR`, `MVFR0/1/2`) are modelled.
- **Table-driven peripherals** — a register table plus per-access fix hooks
  reproduce write-one-to-clear, read-to-clear, write protection, interlock keys
  and status-bit coupling.
- **GDB stub** — remote serial protocol over TCP, serving a real target
  description, so `gdb-multiarch` can step, breakpoint and watch memory.
- **Semihosting** — `BKPT 0xAB` traps are served by the simulator, so a
  `printf` from bare-metal firmware reaches your terminal with no UART involved.
- **Unclaimed-MMIO report** — the simulator lists the peripheral registers the
  firmware touched that no model claims, which is the work list for bringing up
  a new chip.
- **Serial bridges** — the simulated UART/SPI is exposed to the host as a
  `socat` pty pair on POSIX, as a TCP port anywhere (Windows included), or as an
  in-memory loopback for tests.
- **External devices** — LEDs on GPIO pins and an SPI NOR flash, declared in the
  chip description. A device can watch a pin or take over a peripheral's byte
  stream.
- **Firmware loading** — ELF (with LMA/VMA copies), Intel HEX and raw binary.

## Supported chips

| Chip | Core | Vendor family | Modelled peripherals |
|---|---|---|---|
| `stm32f411` | Cortex-M4F | `st` | SCID, SysTick, NVIC, SCB, CP, DBG, DWT, FPU, RCC, GPIOA-F |
| `sam3x8e` | Cortex-M3 | `atmel` | SCID, SysTick, NVIC, SCB, DBG, DWT, PMC, PDC (DMA), PIOA-F, ADC, UART, SPI, PWM, UOTGHS |

Both descriptions have plenty of room: `stm32f411.yaml` carries a commented
catalogue of the remaining STM32F4 peripherals, and the SAM models list which
register semantics are still approximate.

## Install

```bash
pip install .
```

Requires Python 3.9+. Runtime dependencies:

| Package | Used for |
|---|---|
| `unicorn >= 2.1` | CPU emulation |
| `capstone >= 5.0` | disassembly (`show_inst`, register access info) |
| `pyyaml` | chip descriptions |
| `pyelftools` | ELF loading |
| `pyserial` | the `socat` pty bridge only |
| `colorlog` | log formatting |
| `click` | reserved for the planned CLI; unused today |

Older pins do not work: `unicorn` 2.0.x and `capstone` 4.x import
`distutils`/`pkg_resources`, which no longer exist on modern Python and
setuptools.

## Quick start

```python
from xuanwu import XuanWu

# The chip may be a bundled name or a path to a YAML description.
device = XuanWu("stm32f411", "tests/firmware/stm32f411/Blink_m4.ino.elf")
device.reset()
print(hex(device.reg.pc), hex(device.reg.msp))

device.mem.show_map()          # what is mapped where
device.run(count=500_000)      # execute 500k instructions
print(hex(device.reg.pc))
```

There is a runnable version of this in `examples/blinky.py`:

```bash
python examples/blinky.py              # STM32F411, no host dependencies
python examples/blinky.py sampled      # SAM3X8E (uses the socat bridge on Linux)
```

The object model you interact with:

| Attribute | What it is |
|---|---|
| `device.reg` | register access — `reg.pc`, `reg.r0`, `reg.read("msp")`, `reg.write("pc", addr)` |
| `device.mem` | memory map and access — `map()`, `remap()`, `read()`, `write()`, `show_map()` |
| `device.hw.perif` | peripheral models, keyed by lower-cased name (`"uart"`, `"nvic"`, ...) |
| `device.hw` | the interrupt controller and the `UC_HOOK_INTR` handler |
| `device.dev` | external devices by name — `device.dev["LED"]`, `device.dev["FLASH"]` |
| `device.box` | the underlying Unicorn instance, if you need it |
| `device.dasm` | the Capstone disassembler |

## Debugging with GDB

Pass `rsp=True` (or a port) to start the remote stub:

```python
device = XuanWu("stm32f411", "firmware.elf", rsp=6666)
device.reset()
device.run()          # blocks, serving GDB on 127.0.0.1:6666
```

```console
$ gdb-multiarch -q firmware.elf
(gdb) target remote 127.0.0.1:6666
(gdb) info registers pc sp
(gdb) break *0x08001e00
(gdb) continue
```

The stub advertises `qXfer:features:read+` and serves the Cortex-M target
description from the package, so a bare `target remote` works without telling
GDB the architecture first. Loading the ELF anyway gives you symbols. When the
chip declares an FPU the description also carries `d0`-`d15` and `fpscr`:

```console
(gdb) info registers d0
d0             3.0000000075995922  (raw 0x404000003fc00000)
(gdb) p $fpscr
$1 = 0
```

Supported packets: `g`/`p`/`P` (registers), `m`/`X` (memory), `s`/`c`
(step/continue), `Z0`/`z0` (software breakpoints), `Z2`/`Z3` (write/read
watchpoints), `QStartNoAckMode`, `D`, `vKill`.

## Semihosting

Firmware built with the semihosting spec files gets its console for free:

```console
$ arm-none-eabi-gcc -specs=rdimon.specs ...
```

```python
import io
from xuanwu import XuanWu
from xuanwu.backends import SemiHosting

stream = io.StringIO()
device = XuanWu("stm32f411", "firmware.elf", semihosting=SemiHosting(output=stream))
device.reset()
device.run(count=1_000_000)
print(stream.getvalue())
```

`SYS_WRITE0`, `SYS_WRITEC`, `SYS_WRITE`, `SYS_READC`, `SYS_CLOCK`, `SYS_TIME`,
`SYS_GET_CMDLINE`, `SYS_HEAPINFO` and both exit calls are implemented; the rest
are reported and return `-1`. Pass `semihosting=False` to leave `BKPT 0xAB` to
the debugger, or the default `True` to send output to stdout.

Under the hood Unicorn reports `BKPT 0xAB` as an ordinary `BKPT` and does not
advance the PC (it does not expose QEMU's semihosting switch), so xuanwu
recognises the trap by its immediate, services it and steps over it. A plain
`BKPT` is still a fault.

## External devices

A chip description can declare what is wired to the chip:

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
      size: 0x100000
```

```python
device.dev["LED"].state          # True / False / None, plus .transitions
device.dev["FLASH"].read_memory(0, 16)
device.dev.names()               # ['LED', 'FLASH']
```

The LED watches a pin (`ArmSamGpio.add_hook`), and the flash *becomes* the SPI
peripheral's byte stream, so bytes the firmware writes to the transmit register
are executed as flash commands and the responses queue up for the receive
register. `cs` names the GPIO pin that delimits transactions.

`led` and `spi_flash` ship today; `docs/architecture.md` explains how to add
another.

## Finding what is missing

The simulator counts every MMIO access that no peripheral model claimed:

```python
device.run(count=500_000)
device.mem.show_unclaimed()
```

```text
Address      Size    Count
0x40023c00   4       12
1 unclaimed address(es)
```

That address is the STM32F4 flash interface. Reading this list after booting an
unfamiliar firmware tells you which peripherals to implement next.

## Serial ports

The SAM UART and SPI models hand their byte stream to the host through a bridge
(`xuanwu.backends`):

| Bridge | How a host program attaches | Notes |
|---|---|---|
| `socat` | open `perif["uart"].peer_hint` as a tty | POSIX only; the default when `socat` is installed |
| `tcp` | connect to `tcp://host:port` | works everywhere, no extra packages |
| `loopback` | `bridge.feed()` / `bridge.drain()` | tests only, no host resources |

`bridge: auto` (the default in the chip YAML) picks `socat` when available and
TCP otherwise. Select one per chip in the YAML, or per run:

```python
device = XuanWu("sam3x8e", "firmware.elf", hardware_options={"bridge": "tcp", "baudrate": 115200})
print(device.hw.perif["uart"].peer_hint)     # tcp://127.0.0.1:41237
```

## Testing

```bash
pip install -r requirements-dev.txt
pytest tests/unit tests/conformance -v    # fast, no firmware needed
pytest tests/integration -v               # boots real firmware images
```

`tests/firmware/` holds prebuilt images for the two supported chips; the
integration suite also drives the GDB stub over a socket and checks that a
firmware's `Serial.print` output really arrives on the bridged port. One test
class runs a real `gdb-multiarch` against the stub when it is installed (it is
skipped otherwise), and `tests/firmware/stm32f411/fpu_test/` is a Cortex-M4F
image that only produces the right answer if the floating-point exception frame
is stacked and restored — the file next to it is the build script for it.

CI runs the same suite on Linux and Windows across Python 3.10–3.13, plus a
packaging job that builds the wheel in isolation and asserts it ships the chip
and GDB descriptions.

## Repository layout

```
src/xuanwu/
├── xuanwu.py            XuanWu facade: memory map, peripherals, loader, stub
├── chips.py             chip name/path resolution
├── memory.py            memory map, MMIO routing, bit-band aliases
├── register.py          register name -> Unicorn register number
├── loader.py            ELF / Intel HEX / binary
├── rsp.py               GDB remote serial protocol stub
├── device.py            external device layer
├── devices/             built-in device models (led, spi_flash)
├── arch/
│   ├── base.py          register table + ArmHardwareBase
│   ├── cortex_m/        core peripherals (incl. FPU) and the interrupt engine
│   └── vendor/          st/ and atmel/ peripheral families
├── backends/            host-dependent code: serial bridges, semihosting
├── config/              paths, logging, architectural constants
└── data/                chip descriptions and GDB target descriptions
tests/                   unit, conformance and integration suites
docker/                  container definition
examples/                runnable examples
```

## Documentation

- [`docs/architecture.md`](docs/architecture.md) — how the pieces fit together
- [`docs/add-a-chip.md`](docs/add-a-chip.md) — supporting a new MCU
- [`docs/debugging.md`](docs/debugging.md) — logging, GDB, serial bridges
- [`CHANGELOG.md`](CHANGELOG.md) — what changed and why

## Status

A working prototype, not a product. Both bundled chips boot real firmware and run
it; the STM32F411 path is the more complete one. Known gaps:

- the floating point model has two deliberate gaps: `CPACR` does not gate VFP
  execution and Unicorn's CPU model does not either, so a `NOCP` fault cannot be
  raised; lazy stacking is not modelled and `FPCCR.LSPACT` always reads as zero;
- SysTick advances per basic block rather than per real cycle;
- PWM raises no interrupts, the ADC returns preset constants, SPI is byte-wide;
- the device layer has two models (`led`, `spi_flash`) and no I2C bus yet.

## License

LGPL-2.1. See [LICENSE](LICENSE).
