# -*- coding: utf-8 -*-

"""Record a parity trace with the Python implementation.

The Python version of xuanwu is the reference the Go port is checked against.  This
script runs one firmware for a fixed number of instructions and writes down what is
observable from outside the machine -- the program counter, the core registers, a
digest of every memory region the chip description declares, and a digest of the
serial output -- in the format `xuanwu_go/internal/parity` reads.

    python tools/parity_dump.py \
        --chip sam3x8e \
        --firmware tests/firmware/sam3x8e/Blink_m3.ino.elf \
        --count 0 \
        --out ../xuanwu_go/testdata/parity/sam3x8e-blink-reset.json

The serial byte stream is what the firmware transmitted, which is only observable when
the bridge keeps it in memory: the default is therefore ``--bridge loopback``, and a
trace records the digest of what the guest sent (the Go side drains the same bridge).

Nothing in here is part of the xuanwu package: it is a development tool, and the
traces it writes are committed so that a Go-only CI run can check parity without a
Python interpreter.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

# Imported after sys.path is set up, because the tests run from a checkout.
from xuanwu import XuanWu  # noqa: E402

FORMAT = "xuanwu-parity"
VERSION = 1

# The same register set the Go side records; kept here as a literal so that the two
# lists can be compared by eye.
REGISTERS = [
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12",
    "sp", "lr", "pc", "xpsr", "msp", "psp", "primask", "faultmask", "basepri", "control",
]


def load_chip(document_path: Path, seen=None) -> dict:
    """Read a chip description, following its ``include:`` chain.

    A board file (``sam3x8e_tft``) carries only what the shield adds; its memory map and
    its peripherals come from the part it includes.  The region list below has to see the
    merged description, or a trace records no memory regions at all for such a chip --
    which is exactly the sort of quiet gap this tool exists to avoid.
    """
    seen = seen or []
    if document_path in seen:
        raise SystemExit(f"chip description include cycle at {document_path}")
    document = yaml.safe_load(document_path.read_text(encoding="utf-8"))["chip"]
    include = document.get("include")
    if not include:
        return document
    target = Path(include)
    if not target.is_file():
        target = document_path.parent / include
    if not target.is_file():
        candidate = REPO_ROOT / "src" / "xuanwu" / "data" / "chips"
        matches = list(candidate.rglob(f"{Path(include).stem}.yaml"))
        if not matches:
            raise SystemExit(f"cannot resolve include {include!r} of {document_path}")
        target = matches[0]
    base = load_chip(target, seen + [document_path])
    merged = dict(base)
    for key, value in document.items():
        if key in ("peripherals", "devices"):
            continue
        merged[key] = value
    merged["peripherals"] = list(base.get("peripherals", [])) + list(document.get("peripherals", []))
    merged["devices"] = list(base.get("devices", [])) + list(document.get("devices", []))
    return merged


def memory_regions(chip_path: Path):
    """The regions a trace digests: the ones the description declares as memory."""
    chip = load_chip(chip_path)
    regions = []
    for entry in chip.get("peripherals", []):
        for name, spec in entry.items():
            if spec.get("type") == "memory":
                regions.append((name, spec["base"], spec["size"]))
    return regions


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def unclaimed_accesses(device) -> dict:
    """The MMIO histogram: accesses no peripheral model claimed, address/size -> count.

    This is the "what have I not modelled yet" report, and recording it in the trace
    makes it a comparison rather than a report: a firmware that polls a register one
    implementation models and the other does not runs on both, and only this field
    shows the difference.
    """
    return {
        f"0x{address:08x}/{size}": count
        for address, size, count in device.mem.unclaimed_accesses()
    }


def serial_output(device) -> bytes:
    """Everything the firmware transmitted, when the bridge keeps it in memory.

    A bridge with a host end (a pty, a TCP port) has already delivered the bytes to
    the outside world, and contributes nothing here; the in-memory loopback bridge is
    what makes the serial stream comparable between the two implementations.
    """
    out = bytearray()
    for _name, model in getattr(device.hw, "perif", {}).items():
        bridge = getattr(model, "bridge", None)
        drain = getattr(bridge, "drain", None)
        if callable(drain):
            out.extend(drain())
    return bytes(out)


def display_frames(device) -> bytes:
    """The frame buffer of every panel on the board, in the order the devices were declared.

    ``surface`` is the image as the user sees it -- the panel memory read back through
    the orientation the firmware drew in -- so this digest says what the firmware
    painted, not how it was addressed.
    """
    out = bytearray()
    for part in getattr(device, "dev", []):
        surface = getattr(part, "surface", None)
        if surface is not None:
            out.extend(bytes(surface.pixels))
    return bytes(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--chip", required=True, help="chip name or path to a .yaml file")
    parser.add_argument("--firmware", required=True, help="firmware image (.elf/.hex/.bin)")
    parser.add_argument("--count", type=int, default=0, help="instructions to run before recording")
    parser.add_argument("--out", help="where to write the trace (default: stdout)")
    parser.add_argument("--note", default="", help="a note to keep in the trace")
    parser.add_argument(
        "--bridge",
        default="loopback",
        help="serial bridge for the run: loopback (the default, so the byte stream can be "
             "recorded), none, tcp or socat",
    )
    args = parser.parse_args()

    firmware = Path(args.firmware)
    chip_path = Path(args.chip)
    if not chip_path.is_file():
        # A bundled chip name: the descriptions live inside the package.
        candidate = REPO_ROOT / "src" / "xuanwu" / "data" / "chips"
        matches = list(candidate.rglob(f"{args.chip}.yaml"))
        if not matches:
            print(f"error: no chip description for {args.chip!r}", file=sys.stderr)
            return 2
        chip_path = matches[0]

    device = XuanWu(str(chip_path), str(firmware), hardware_options={"bridge": args.bridge})
    device.reset()
    if args.count:
        device.run(count=args.count)

    registers = {}
    for name in REGISTERS:
        try:
            registers[name] = f"0x{device.reg.read(name) & 0xFFFFFFFF:08X}"
        except Exception:  # noqa: BLE001 - a register the core does not have is not fatal
            registers[name] = "0x00000000"
    registers["pc"] = f"0x{device.reg.pc:08X}"

    memory = {}
    for name, base, size in memory_regions(chip_path):
        memory[name] = digest(bytes(device.mem.read(base, size)))

    serial = serial_output(device)
    frames = display_frames(device)

    trace = {
        "format": FORMAT,
        "version": VERSION,
        "chip": chip_path.stem,
        "firmware": str(firmware),
        "firmware_sha256": digest(firmware.read_bytes()),
        "instructions": args.count,
        "pc": f"0x{device.reg.pc:08X}",
        "registers": registers,
        "memory": memory,
        # What the firmware transmitted, as a digest and as a length: the digest is what
        # the two implementations compare, and the length is what makes a mismatch
        # readable.
        "uart_sha256": digest(serial),
        "uart_bytes": len(serial),
        # And what it painted on any panel on the board, the same way.
        "display_sha256": digest(frames) if frames else "",
        "display_bytes": len(frames),
        # Every MMIO access no model claimed, with how often it was touched.
        "unclaimed": unclaimed_accesses(device),
    }
    if args.note:
        trace["note"] = args.note
    text = json.dumps(trace, indent=2, sort_keys=True) + "\n"
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
        print(f"wrote {args.out} ({args.count} instructions, pc {trace['pc']})")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
