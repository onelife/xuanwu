# -*- coding: utf-8 -*-

import re
import socket
from os import path
from typing import Optional, Any, Union, List

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ

from .config import logger, RESOURCE
from .register import RegisterController
from .memory import MemoryController
from .exception import XwInvalidChipInformation


__all__ = ["RemoteSerialProtocol"]

# ref: binutils-gdb/include/gdb/signals.def
GDB_SIGNAL_INT = 2  # "SIGINT", "Interrupt"
GDB_SIGNAL_ILL = 4  # "SIGILL", "Illegal instruction"
GDB_SIGNAL_TRAP = 5  # "SIGTRAP", "Trace/breakpoint trap"
GDB_SIGNAL_BUS = 10  # "SIGBUS", "Bus error"
GDB_SIGNAL_SEGV = 11  # "SIGSEGV", "Segmentation fault"

# rom (rx)    : ORIGIN = 0x00080000, LENGTH = 0x00080000 /* Flash, 512K */
# sram0 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00010000 /* sram0, 64K */
# sram1 (rwx) : ORIGIN = 0x20080000, LENGTH = 0x00008000 /* sram1, 32K */
# ram (rwx)   : ORIGIN = 0x20070000, LENGTH = 0x00018000 /* sram, 96K */


class RemoteSerialProtocol(object):
    """GDB remote serial protocol"""

    def __init__(
        self, box: Uc, mem: MemoryController, reg: RegisterController, arch: int, mode: int,
        port: int = 6666, fpu: bool = False, **kwargs: Any
    ) -> None:
        super().__init__(**kwargs)
        self._box = box
        self._box.hook_add(UC_HOOK_CODE, self.process_breakpoints)
        self._box.hook_add(UC_HOOK_MEM_WRITE, self.process_memory_write)
        self._box.hook_add(UC_HOOK_MEM_READ, self.process_memory_read)
        self._mem = mem
        self._reg = reg
        self.host = "127.0.0.1"
        self.port = port
        self.fpu = fpu
        self.reg_info = self.get_register_info(arch, mode, fpu)
        self.target_xml = self.get_target_xml(arch, mode, fpu)
        self.bp = set()
        self.mw = dict()
        self.mr = set()
        self.last_mr = None
        self.no_ack = False
        self.kill = False

    @staticmethod
    def _profile_paths(arch: int, mode: int, fpu: bool = False) -> List[str]:
        """Locate the GDB target descriptions for this architecture/mode.

        A core with an FPU reports a second feature: the floating-point register
        file.  GDB reads the register set as the concatenation of the documents
        named by the target description and numbers the registers in that order,
        which is why the FP feature has to come last.
        """
        arch_ = mode_ = ""
        extra: List[str] = []
        if arch == UC_ARCH_ARM:
            arch_ = "arm"
            if mode & UC_MODE_MCLASS:
                mode_ = "arm-m-profile.xml"
                if fpu:
                    # what a Cortex-M4F reports: d0-d15 and fpscr
                    extra.append("arm-vfpv2.xml")

        profiles = [path.join(RESOURCE["gdb"], arch_, name) for name in [mode_, *extra]]
        for profile in profiles:
            if not path.exists(profile):
                raise XwInvalidChipInformation(f"Unsupported architecture: {arch =}, {mode =}, {profile =}")
        return profiles

    @classmethod
    def get_register_info(cls, arch: int, mode: int, fpu: bool = False):
        import xml.etree.ElementTree as ET

        info = {}
        # An explicit ``regnum`` only pushes the implicit counter forward, which
        # is exactly how GDB numbers the registers of a target description.
        next_num = 0
        for profile in cls._profile_paths(arch, mode, fpu):
            root = ET.parse(profile).getroot()
            for reg in root.findall("reg"):
                num = int(reg.attrib["regnum"]) if "regnum" in reg.attrib else next_num
                info[reg.attrib["name"]] = (num, int(reg.attrib["bitsize"]) // 8)
                next_num = num + 1
        return info

    @classmethod
    def get_target_xml(cls, arch: int, mode: int, fpu: bool = False) -> str:
        """Target description served through ``qXfer:features:read``.

        GDB requires a ``<target>`` document; serving the bare ``<feature>``
        element from the binutils file makes GDB silently ignore it and fall back
        to its default (much larger) register set.
        """
        features = []
        for profile in cls._profile_paths(arch, mode, fpu):
            with open(profile, encoding="utf-8") as file:
                text = file.read()
            features.append(text[text.index("<feature") :].rstrip())
        indented = "\n".join("  " + line for line in "\n".join(features).splitlines())
        return (
            '<?xml version="1.0"?>\n'
            '<!DOCTYPE target SYSTEM "gdb-target.dtd">\n'
            "<target>\n"
            "  <architecture>arm</architecture>\n"
            f"{indented}\n"
            "</target>\n"
        )

    def encode_registers(self) -> str:
        """Encode every register for the 'g' packet, in target-description order.

        GDB lays out the 'g' payload sequentially in the order the registers were
        defined by the target description -- the register *number* does not have
        to match its byte offset.  So this is a plain concatenation, ordered by
        regnum for determinism.
        """
        chunks = []
        for reg, (_num, length) in sorted(self.reg_info.items(), key=lambda item: item[1][0]):
            try:
                chunks.append(self._reg.read(reg).to_bytes(length, "little").hex())
            except Exception as err:  # noqa: BLE001 - an unreadable register is 'unavailable', not fatal
                logger.debug(f"Cannot read register {reg}: {type(err).__name__}: {err}")
                chunks.append("xx" * length)
        return "".join(chunks)

    def process_breakpoints(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        if address not in self.bp:
            return
        self._box.emu_stop()
        logger.info(f"[@] BP 0x{address:08x}")

    def process_memory_write(self, box: Uc, access: int, address: int, size: int, value: int, user_data: Any) -> None:
        if (address, size) not in self.mw or self.mw[(address, size)] == value:
            return
        self._box.emu_stop()
        logger.info(f"[W] WP 0x{address:08x}, {size}, 0x{self.mw[(address, size)]:08x} => 0x{value:08x}")
        self.mw[(address, size)] = value

    def process_memory_read(self, box: Uc, access: int, address: int, size: int, value: int, user_data: Any) -> None:
        if (address, size) not in self.mr:
            return
        if self.last_mr == (address, size):
            self.last_mr = None
            return
        self.last_mr = (address, size)
        self._box.emu_stop()
        logger.info(f"[R] WP 0x{address:08x}, {size}")

    def _continue(self, count: int) -> str:
        from unicorn import UcError, UC_ERR_READ_UNMAPPED, UC_ERR_WRITE_UNMAPPED, UC_ERR_INSN_INVALID

        # execute instructions continuously
        reply = None
        try:
            self._box.emu_start(self._reg.pc_t, 0, count=count)
            reply = f"S{GDB_SIGNAL_TRAP:02x}"
        except UcError as err:
            logger.error(f"Emulation error: {type(err)}, {err}")
            if err.errno in [UC_ERR_READ_UNMAPPED, UC_ERR_WRITE_UNMAPPED]:
                reply = f"S{GDB_SIGNAL_SEGV:02x}"
                # raise
            elif err.errno == UC_ERR_INSN_INVALID:
                reply = f"S{GDB_SIGNAL_ILL:02x}"
                raise
            else:
                raise
        except KeyboardInterrupt:
            reply = f"S{GDB_SIGNAL_INT:02x}"
        return reply

    def process_packet(self, packet: bytes) -> str:
        reply = None
        name, *params = packet.split(b":", 1)
        if name == b"?":
            # reply reason of stop
            # reg = ";".join([f"{self.reg_info[r][0]:02x}:{0:08x}" for r in ["r11", "sp", "pc"]])
            reply = f"T{GDB_SIGNAL_TRAP:02x}"

        elif name == b"g":
            # for reg_, _ in self.reg_info.items():
            #     logger.debug(f"{reg_} = 0x{self._reg.read(reg_):02x}")
            # reply register values
            reply = self.encode_registers()

        elif name.startswith(b"m"):
            from unicorn import UcError, UC_ERR_READ_UNMAPPED

            # reply value in memory
            addr, len_ = [int(num, 16) for num in name[1:].split(b",")]
            try:
                reply = self._mem.read(addr, len_).hex()
            except UcError as err:
                logger.error(f"Read memory @0x{addr:08x}: {type(err)}, {err}")
                if err.errno == UC_ERR_READ_UNMAPPED:
                    reply = "E02"
                else:
                    raise

        elif name.startswith(b"p"):
            num = int(name[1:], 16)
            for reg, (num_, len_) in self.reg_info.items():
                if num == num_:
                    reply = (self._reg.read(reg)).to_bytes(len_, "little")
                    break
            else:
                reply = "E01"

        elif name.startswith(b"P"):
            num_str, val_str = name[1:].split(b"=")
            num = int(num_str, 16)
            val = int(val_str, 16)
            for reg, (num_, len_) in self.reg_info.items():
                if num == num_:
                    self._reg.write(reg, val)
                    reply = "OK"
                    break
            else:
                reply = "E01"

        elif name.startswith(b"q"):
            # query packet
            if name == b"qSupported":
                # reply supported features
                logger.info(f'GDB supported features: {b", ".join([p[:-1] for p in params[0].split(b";") if p.endswith(b"+")])}')
                features = [
                    "PacketSize=1000",
                    "QStartNoAckMode+",
                    "swbreak+",
                    "hwbreak+",
                    "qXfer:features:read+",
                ]
                reply = ";".join(features)
            elif name == b"qTStatus":
                # reply trace status
                reply = "T0;tnotrun:0;tframes:0;tcreated:0;tfree:2048;tsize:2048;circular:0;disconn:0"
            elif name in [b"qTfV", b"qTsV", b"qTfP", b"qTsP", b"qfThreadInfo", b"qsThreadInfo"]:
                # reply end of list
                reply = "l"
            elif name == b"qC":
                # reply current thread ID
                reply = ""
            elif name == b"qAttached":
                # reply if attached to an existing process or created a new process
                reply = ""
            elif name == b"qOffsets":
                # reply section offsets
                reply = "Text=0;Data=0;Bss=0"
            elif name == b"qSymbol":
                # reply not need symbol look up
                reply = "OK"
            elif name == b"qXfer":
                # qXfer:<object>:<read|write>:<annex>:<offset>,<length>
                annex = (params[0] if params else b"").split(b":")
                if len(annex) == 4 and annex[0] == b"features" and annex[1] == b"read" and annex[2] == b"target.xml":
                    offset, length = [int(item, 16) for item in annex[3].split(b",")]
                    chunk = self.target_xml[offset : offset + length]
                    # "l" = last chunk, "m" = more data available
                    reply = ("m" if offset + length < len(self.target_xml) else "l") + chunk
                else:
                    # Object is not supported; a bare empty reply would leave GDB guessing.
                    reply = "E00"

        elif name.startswith(b"v"):
            # multi-letter name
            if name in [b"vMustReplyEmpty", b"vCont?"]:
                # reply empty string
                reply = ""
            elif name.startswith(b"vKill"):
                self.kill = True
                reply = "OK"

        elif name.startswith(b"H"):
            # set thread for subsequent operations
            # c, for step and continue; g, for other operations
            # -1, all threads; 0, any thread
            if name[1:2] in [b"c", b"g"]:
                reply = "OK"

        elif name.startswith(b"Q"):
            if name == b"QStartNoAckMode":
                self.no_ack = True
                logger.debug("Enable NoAck mode")
                reply = "OK"

        elif name.startswith(b"X"):
            # write data to memory
            addr, length = [int(item, 16) for item in name[1:].split(b",")]
            info = self._mem.get_map(addr, addr + length)
            if not info:
                reply = "E01"
            else:
                if length == 0:
                    reply = "OK"
                elif length != len(params[0]):
                    logger.debug(f"{length}, {len(params[0])}")
                    reply = "E01"
                else:
                    from unicorn import UC_ERR_WRITE_UNMAPPED

                    try:
                        self._mem.write(addr, params[0])
                        reply = "OK"
                    except UcError as err:
                        logger.error(f"Write memory @0x{addr:08x}: {type(err)}, {err}")
                        if err.errno == UC_ERR_WRITE_UNMAPPED:
                            reply = "E02"
                        else:
                            raise

        elif name.startswith(b"s") or name.startswith(b"S"):
            # execute one instruction and stop
            return self._continue(1)

        elif name.startswith(b"c") or name.startswith(b"C"):
            # execute instructions continuously
            return self._continue(0)

        elif name.startswith(b"Z"):
            # set breakpoint
            type_, addr, *kind_size = [int(item, 16) for item in name[1:].split(b",")]
            if type_ == 0:
                # set software breakpoint
                # ref: https://sourceware.org/gdb/onlinedocs/gdb/ARM-Breakpoint-Kinds.html#ARM-Breakpoint-Kinds
                # kind for arm:
                # 2, 16-bit Thumb mode breakpoint
                # 3, 32-bit Thumb mode (Thumb-2) breakpoint
                # 4, 32-bit ARM mode breakpoint
                self.bp.add(addr)
                logger.info(f"[+] BP: 0x{addr:08x}, {kind_size[0]}")
                reply = "OK"

            elif type_ == 2:
                # set write watchpoint
                value = int.from_bytes(self._mem.read(addr, kind_size[0]), "little")
                self.mw[(addr, kind_size[0])] = value
                logger.info(f"[+] WWP: 0x{addr:08x}, {kind_size[0]}; {value}")
                reply = "OK"

            elif type_ == 3:
                # set read watchpoint
                self.mr.add((addr, kind_size[0]))
                logger.info(f"[+] RWP: 0x{addr:08x}, {kind_size[0]}")
                reply = "OK"

            else:
                reply = ""

        elif name.startswith(b"z"):
            # delete breakpoint
            type_, addr, *kind_size = [int(item, 16) for item in name[1:].split(b",")]
            if type_ == 0:
                self.bp.discard(addr)
                logger.info(f"[-] BP: 0x{addr:08x}, {kind_size[0]}")
                reply = "OK"

            elif type_ == 2:
                # delete write watchpoint
                _ = self.mw.pop((addr, kind_size[0]), None)
                logger.info(f"[-] WWP: 0x{addr:08x}, {kind_size[0]}")
                reply = "OK"

            elif type_ == 3:
                # delete read watchpoint
                self.mr.discard((addr, kind_size[0]))
                logger.info(f"[-] RWP: 0x{addr:08x}, {kind_size[0]}")
                reply = "OK"

            else:
                reply = ""

        elif name.startswith(b"D"):
            # detach
            reply = "OK"

        else:
            pass

        # if reply is not None:
        #     reply = f"+${reply}#{reduce(lambda a, b: (ord(a) if isinstance(a, str) else a) + ord(b), reply, 0) & 0xff:02x}"
        return reply

    def read_data(self, data_in: bytes) -> Optional[bytes]:
        """
        ref: https://sourceware.org/gdb/current/onlinedocs/gdb/Overview.html
        ref: https://rosettacode.org/wiki/Run-length_encoding
        """
        try:
            packet, checksum = re.fullmatch(rb"^\+*\$([^#]+)#([a-zA-Z0-9]{2})", data_in).groups()
        except Exception:
            if data_in == b"+":
                return None
            logger.warning(f"read_data: Unknown data format, {data_in}")
            return None
        # checksum
        if (sum(packet) & 0xff) != int(checksum, 16):
            logger.warning("read_data: Checksum error")
            return None
        # run-length decode
        buffer = re.sub(rb"(.)\*(.)", lambda m: m.group(1) * (ord(m.group(2)) - 28), packet)
        # unescape
        data = re.sub(rb"}.", lambda m: bytes([m.group(0)[1] ^ 0x20]), buffer, flags=re.DOTALL)

        return data

    def write_data(self, data_out: Union[str, bytes, None]) -> bytes:
        """
        ref: https://sourceware.org/gdb/current/onlinedocs/gdb/Overview.html
        ref: https://rosettacode.org/wiki/Run-length_encoding
        """
        if data_out is None:
            data_out = ""
        elif isinstance(data_out, bytes):
            # Single/multi register replies are built as raw bytes; RSP carries them hex encoded.
            data_out = data_out.hex()
        # escape
        # logger.debug(f"{data_out =}")
        buffer = re.sub(r"[}$#*]", lambda m: f"}}{chr(ord(m.group(0)[0]) ^ 0x20)}", data_out)

        # run-length encode
        def encode_run(match) -> str:
            """Encode one long run, keeping the count character printable and non-metacharacter.

            ``chr(28 + length)`` collides with the RSP metacharacters for a few
            lengths (14 -> '*', 97 -> '}'), which used to emit a raw metacharacter
            inside the packet body and corrupt the frame.
            """
            run = match.group(0)
            char = run[0]
            encoded = []
            while len(run) > 6:
                length = min(len(run), 97)
                while length > 6 and chr(28 + length) in "}#$*":
                    length -= 1
                if length <= 6:
                    break
                encoded.append(f"{char}*{chr(28 + length)}")
                run = run[length:]
            encoded.append(run)
            return "".join(encoded)

        buffer_ = ""
        while buffer_ != buffer:
            if buffer_:
                buffer = buffer_
            buffer_ = re.sub(r"(.)\1{8,97}", encode_run, buffer)
        data = re.sub(r"(.)\1{3,5}", lambda m: f"{m.group(1)}*{chr(28 + len(m.group(0)))}", buffer_)
        return f"${data}#{sum([ord(c) for c in data]) & 0xff:02x}".encode("utf-8")

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        logger.info(f"0GDB server is listening on {self.host}:{self.port}")
        sock.bind((self.host, self.port))
        sock.listen(0)
        logger.info(f"GDB server is listening on {self.host}:{self.port}")

        while True:
            conn, addr = sock.accept()
            with conn:
                logger.info(f"GDB server is connected with {addr[0]}:{addr[1]}")
                self.no_ack = False
                self.kill = False
                try:
                    while True:
                        datain = conn.recv(4096)
                        if not datain:
                            # disconnected
                            logger.info(f"GDB server is disconnected with {addr[0]}:{addr[1]}")
                            break
                        data = self.read_data(datain)
                        if not data:
                            continue
                        logger.debug(f"RSP RX: {data}")
                        if not self.no_ack:
                            # logger.info("ACK")
                            _ = conn.sendall(b"-" if data is None else b"+")
                        if data:
                            try:
                                reply = self.process_packet(data)
                                logger.debug(f"RSP TX0: {reply}")
                                dataout = self.write_data(reply)
                            except Exception as err:
                                # A single malformed/unsupported packet must never take down the stub.
                                logger.error(f"RSP packet {data!r} failed: {type(err).__name__}: {err}")
                                dataout = self.write_data("E01")
                            # logger.debug(f"RSP TX: {dataout}")
                            _ = conn.sendall(dataout)
                            # logger.debug(f"sendall: {err}")
                        if self.kill:
                            break
                except KeyboardInterrupt:
                    pass
