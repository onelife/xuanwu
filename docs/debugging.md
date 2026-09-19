# Debugging

How to see what the simulation is doing, attach a debugger to it, and talk to a
simulated serial port.

---

## 1. Logging

The library logs under the `xuanwu` logger at `INFO` by default, to the console.

```bash
DEBUG=1 python firmware.py
```

`DEBUG=1` switches the level to `DEBUG` (every MMIO access, every interrupt
entry/exit) and additionally writes a rotating file log to
`$XDG_STATE_HOME/xuanwu/xuanwu.log`, or `~/.local/state/xuanwu/xuanwu.log` when
`XDG_STATE_HOME` is unset. If neither location is writable it falls back to the
console only.

The log file is never written to the current working directory — importing the
library with a read-only cwd used to fail because of exactly that.

To silence it from your own program:

```python
import logging
logging.getLogger("xuanwu").setLevel(logging.WARNING)
logging.disable(logging.CRITICAL)   # before importing xuanwu, in tests
```

## 2. Inspecting a running simulation

```python
device = XuanWu("stm32f411", "firmware.elf")
device.reset()

device.mem.show_map()                 # regions, permissions, registered IO
print(hex(device.reg.pc), hex(device.reg.sp))
print(hex(device.reg.read("xpsr")))

device.run(count=1000)
device.show_inst(-2, 2)               # disassemble around the PC
```

`show_inst(start_offset, end_offset)` disassembles instructions relative to the
PC and prints the registers each one reads and writes, so a single call shows
both where you are and what the instruction does to the register file.

Peripheral state is reachable through the models themselves:

```python
uart = device.hw.perif["uart"]
print(hex(uart.read_register("SR")))
print(hex(uart.read(0x400E0818, 4)))   # the same thing through the MMIO path
```

When execution dies, `run()` catches the Unicorn error, dumps the instructions
around the PC (and the first registers for a fetch fault) and then re-raises.
Unmapped accesses inside a mapped region show up as warnings instead:

```text
WARNING  xuanwu:memory.py:252 Unmapped bitband (R): 0x40023C00 => 0x00000000
```

That particular address is the STM32F4 flash interface, which is not modelled —
the firmware writing to it is harmless, and the warning is the simulator telling
you which peripheral it does not know about yet.

The same information is collected and can be printed as a table, which is the
quickest way to find out what a new firmware needs:

```python
device.run(count=500_000)
device.mem.show_unclaimed()
```

```text
Address      Size    Count
0x40023c00   4       12
1 unclaimed address(es)
```

`device.mem.unclaimed_accesses()` returns the same data as
`[(address, size, count)]` if you want to post-process it.

## 3. Serial ports

The SAM UART and SPI models publish their host end as `peer_hint`:

```python
uart = device.hw.perif["uart"]
print(uart.peer_hint)      # /tmp/uart_peer_xxxx   or   tcp://127.0.0.1:41237
```

### socat bridge (default on POSIX)

`peer_hint` is a pty path. Any terminal program can open it:

```console
$ socat - /tmp/uart_peer_xxxx,raw,echo=0
$ picocom -b 115200 /tmp/uart_peer_xxxx
```

This is what the `auto` setting picks when `socat` is on `PATH`. If it is not
installed you get `XwSerialBridgeError` naming the missing binary — install
`socat`, or switch bridges.

### TCP bridge (portable)

`peer_hint` is a listen address; connect with anything:

```python
import socket
host, port = device.hw.perif["uart"].peer_hint[len("tcp://"):].rsplit(":", 1)
with socket.create_connection((host, int(port))) as peer:
    print(peer.recv(4096))
```

Bytes the firmware transmits before anyone connects are buffered (bounded), so a
short run is not lost. `auto` falls back to this bridge when `socat` is
unavailable, which is what makes the SAM chips usable on Windows.

To force it:

```python
device = XuanWu("sam3x8e", "firmware.elf", hardware_options={"bridge": "tcp"})
```

### Loopback bridge (tests)

```python
device = XuanWu("sam3x8e", "firmware.elf", hardware_options={"bridge": "loopback"})
bridge = device.hw.perif["uart"]._bridge
bridge.feed(b"hello\n")       # to the simulated device
...
print(bridge.drain())         # what the device transmitted
```

No host resources at all, so it works in any environment. `tests/integration/`
uses it, plus the TCP bridge for the full end-to-end check.

## 4. GDB

Start the stub by passing a port (or `True` for the default 6666):

```python
device = XuanWu("stm32f411", "firmware.elf", rsp=6666)
device.reset()
device.run()        # blocks, serving GDB
```

Then, from another terminal:

```console
$ gdb-multiarch -q firmware.elf
(gdb) target remote 127.0.0.1:6666
Remote debugging using 127.0.0.1:6666
0x08001de8 in Reset_Handler ()
(gdb) info registers pc sp xpsr
(gdb) break *0x08001e00
(gdb) continue
(gdb) x/8i $pc
```

The stub serves its own target description over
`qXfer:features:read:target.xml`, so `set architecture` is not required — a bare
`target remote` gets the Cortex-M register set. Loading the ELF first still
helps, because that is where GDB gets symbols.

Working packets: `g`/`p`/`P`, `m`/`X`, `s`/`c`, `Z0`/`z0` software breakpoints,
`Z2` write watchpoints, `Z3` read watchpoints, `QStartNoAckMode`, `D`, `vKill`.
Anything else is answered with an empty reply, which GDB treats as unsupported —
it will not take the stub down.

Watchpoints are implemented with Unicorn memory hooks, so they fire on the
instruction *after* the access; `Z2`/`Z3` report the address and size.

The stub logs the whole dialogue at `DEBUG`:

```console
$ DEBUG=1 python firmware.py        # RSP RX / RSP TX0 lines
```

## 5. Semihosting

If the firmware was linked with the semihosting spec files
(`--specs=rdimon.specs`, `--specs=rdimon-v2m.specs`), its `printf` goes through
`BKPT 0xAB` rather than a UART, and no serial bridge is involved. Semihosting is
enabled by default and writes to stdout:

```python
device = XuanWu("stm32f411", "firmware.elf")            # semihosting=True
device.run()
```

To capture the output, or to feed the guest's `SYS_READC`:

```python
import io
from xuanwu.backends import SemiHosting

out, inp = io.StringIO(), io.StringIO("y\n")
host = SemiHosting(output=out, input=inp, argv=["firmware.elf", "--verbose"])
device = XuanWu("stm32f411", "firmware.elf", semihosting=host)
device.reset()
device.run()
print(out.getvalue())
print("guest exited:", host.exited, host.exit_code)
```

Implemented: `SYS_WRITEC`, `SYS_WRITE0`, `SYS_WRITE`, `SYS_READC`, `SYS_ISTTY`,
`SYS_CLOCK`, `SYS_TIME`, `SYS_ERRNO`, `SYS_ISERROR`, `SYS_FLEN`, `SYS_SEEK`,
`SYS_TICKFREQ`, `SYS_ELAPSED`, `SYS_GET_CMDLINE`, `SYS_HEAPINFO`, `SYS_EXIT`,
`SYS_EXIT_EXTENDED`. Anything else is logged, recorded in `host.unsupported` and
returns `-1`; file operations are deliberately not implemented.

Because xuanwu recognises the trap by the `0xAB` immediate (Unicorn reports it as
a plain `BKPT` and leaves the PC on the instruction), a debugger `BKPT` with any
other immediate still faults as before. Pass `semihosting=False` to disable the
service entirely.

## 6. Common failures

| Symptom | Cause and fix |
|---|---|
| `XwInvalidParameter: Unknown chip 'x'. ... available: ...` | wrong chip name; the message lists what is bundled (`xuanwu.chips.list_chips()`) |
| `XwUnknownHardware: Unknown core peripheral: X` | the YAML region name resolves to no model — check `BUILDIN` and the family prefix of `chip.name` |
| `XwSerialBridgeError: 'socat' was not found on PATH` | install `socat`, or use `bridge: tcp` / `auto` |
| `unicorn.UcError: Invalid memory ...` | the firmware touches something unmapped; `run()` prints the instructions around the PC first |
| `WARNING ... Unmapped bitband (R/W)` | a peripheral is not modelled yet; harmless, but it tells you what to implement next |
| Tests skip with "needs the socat serial bridge" | `apt-get install socat`, or set the bridge to `tcp`/`loopback` |
| `ModuleNotFoundError: distutils` / `pkg_resources` | an outdated `unicorn`/`capstone`; use `unicorn >= 2.1`, `capstone >= 5.0` |

## 7. Test selection

```bash
pytest tests/unit tests/conformance -q     # fast, no host dependencies
pytest tests/integration -q                # boots firmware, needs a serial bridge
pytest -m integration -q                   # same, by marker
pytest tests -q -k gdb                     # only the GDB stub tests
```

Markers are registered in `pyproject.toml`: `integration`, `conformance`, `slow`.
