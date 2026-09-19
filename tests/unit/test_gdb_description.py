# -*- coding: utf-8 -*-

"""The GDB target description has to describe the floating-point register file.

GDB numbers the registers of a target description in the order the features are
listed, and expects the ``g`` packet to be laid out in that same order.  A
Cortex-M4F therefore reports ``arm-m-profile.xml`` followed by
``arm-vfpv2.xml``; getting the order wrong makes every register after ``xpsr``
read back as a different one.
"""

import xml.etree.ElementTree as ET

import pytest
from unicorn import UC_ARCH_ARM, UC_MODE_MCLASS

from xuanwu.rsp import RemoteSerialProtocol

M_PROFILE = "org.gnu.gdb.arm.m-profile"
VFP = "org.gnu.gdb.arm.vfp"


def target_xml(fpu: bool) -> ET.Element:
    text = RemoteSerialProtocol.get_target_xml(UC_ARCH_ARM, UC_MODE_MCLASS, fpu)
    return ET.fromstring(text[text.index("<target>") :])


def feature_names(fpu: bool) -> list:
    root = target_xml(fpu)
    return [feature.attrib["name"] for feature in root.findall("feature")]


def described_registers(fpu: bool) -> list:
    return [reg.attrib["name"] for reg in target_xml(fpu).iter("reg")]


class TestWithoutFpu:
    def test_only_the_core_feature_is_served(self):
        assert feature_names(False) == [M_PROFILE]

    def test_the_register_file_ends_at_xpsr(self):
        assert described_registers(False)[-1] == "xpsr"
        assert len(RemoteSerialProtocol.get_register_info(UC_ARCH_ARM, UC_MODE_MCLASS, False)) == 17


class TestWithFpu:
    def test_the_vfp_feature_follows_the_core_one(self):
        assert feature_names(True) == [M_PROFILE, VFP]

    def test_the_double_registers_are_described(self):
        assert described_registers(True)[-17:] == [*(f"d{i}" for i in range(16)), "fpscr"]

    def test_the_registers_are_numbered_as_gdb_numbers_them(self):
        info = RemoteSerialProtocol.get_register_info(UC_ARCH_ARM, UC_MODE_MCLASS, True)
        # r0-r12, sp, lr, pc are implicit 0..15; xpsr pins itself to 25 and the
        # floating-point registers continue from there.
        assert info["xpsr"][0] == 25
        assert info["d0"][0] == 26
        assert info["d15"][0] == 41
        assert info["fpscr"][0] == 42
        assert info["d0"][1] == 8, "d0 is a 64-bit register"

    def test_the_g_packet_order_matches_the_description(self):
        info = RemoteSerialProtocol.get_register_info(UC_ARCH_ARM, UC_MODE_MCLASS, True)
        by_number = [name for name, _ in sorted(info.items(), key=lambda item: item[1][0])]
        assert described_registers(True) == by_number

    def test_the_description_is_a_single_target_document(self):
        text = RemoteSerialProtocol.get_target_xml(UC_ARCH_ARM, UC_MODE_MCLASS, True)
        assert text.startswith('<?xml version="1.0"?>')
        assert "<!DOCTYPE target" in text
        assert text.count("<target>") == 1


@pytest.fixture(scope="module")
def stm32f411(stm32f411_path, stm32f411_firmware):
    from xuanwu import XuanWu

    return XuanWu(str(stm32f411_path), str(stm32f411_firmware), rsp=True)


class TestTheDeviceDecides:
    def test_a_chip_with_an_fpu_reports_it(self, stm32f411):
        assert "fpu" in stm32f411.hw.perif
        assert stm32f411.rsp.fpu is True
        assert "d0" in stm32f411.rsp.reg_info

    def test_the_g_packet_grows_by_the_fp_register_file(self, stm32f411):
        # 17 core registers of 4 bytes + d0-d15 (8 bytes each) + fpscr
        payload = stm32f411.rsp.encode_registers()
        assert len(payload) == 2 * (17 * 4 + 16 * 8 + 4) == 400

    def test_the_g_packet_places_the_fp_registers_after_the_core(self, stm32f411):
        stm32f411.reg.write("r0", 0xDEAD_BEEF)
        stm32f411.reg.write("d0", 0x0123_4567_89AB_CDEF)
        payload = stm32f411.rsp.encode_registers()
        assert payload.startswith("efbeadde")
        # 17 core registers of 4 bytes come first
        assert payload[17 * 8 : 17 * 8 + 16] == "efcdab8967452301"
        assert payload.endswith(stm32f411.reg.read("fpscr").to_bytes(4, "little").hex())


class TestWithoutAnFpu:
    def test_a_chip_without_an_fpu_keeps_the_core_description_only(self, sam3x8e_path, sam3x8e_firmware):
        from xuanwu import XuanWu

        device = XuanWu(
            str(sam3x8e_path), str(sam3x8e_firmware), rsp=True, hardware_options={"bridge": "none"}
        )
        assert "fpu" not in device.hw.perif
        assert device.rsp.fpu is False
        assert "d0" not in device.rsp.reg_info
        assert len(device.rsp.encode_registers()) == 2 * 17 * 4
