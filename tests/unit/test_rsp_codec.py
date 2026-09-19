# -*- coding: utf-8 -*-

"""Unit tests for the GDB RSP packet codec.

Both codec helpers are pure functions of their argument (they never touch
instance state), so they are exercised through thin unbound wrappers.
"""

import pytest

from xuanwu.rsp import RemoteSerialProtocol


def write_data(data_out):
    return RemoteSerialProtocol.write_data(None, data_out)


def read_data(data_in):
    return RemoteSerialProtocol.read_data(None, data_in)


def body_of(packet: bytes) -> bytes:
    """Strip the leading '$' and trailing '#xx' checksum."""
    return packet[1 : packet.rindex(b"#")]


class TestWriteData:
    def test_ascii_reply(self):
        assert write_data("OK") == b"$OK#9a"

    def test_none_reply_is_a_valid_empty_packet(self):
        # An unsupported packet used to crash the server with
        # "expected string or bytes-like object, got 'NoneType'".
        assert write_data(None) == b"$#00"

    def test_bytes_reply_is_hex_encoded(self):
        # 'p' replies are raw bytes; they used to crash the server with
        # "cannot use a string pattern on a bytes-like object".
        assert body_of(write_data(b"\x01\x02\xff")) == b"0102ff"

    def test_checksum_is_correct(self):
        packet = write_data("hello")
        body, checksum = packet[1:].split(b"#")
        assert int(checksum, 16) == sum(body) & 0xFF

    @pytest.mark.parametrize("char", ["$", "#", "}", "*"])
    def test_special_characters_are_escaped(self, char):
        body = body_of(write_data(char))
        assert b"}" in body
        assert char.encode() not in body.replace(b"}", b"")

    def test_long_run_is_compressed(self):
        assert len(write_data("a" * 60)) < 40


class TestReadData:
    @pytest.mark.parametrize("text", ["OK", "T05", "deadbeef", "x" * 50])
    def test_write_then_read_round_trip(self, text):
        assert read_data(write_data(text)) == text.encode()

    @pytest.mark.parametrize("length", [4, 6, 9, 13, 14, 15, 50, 96, 97, 98, 200])
    def test_run_length_boundaries_round_trip(self, length):
        # Runs of 14 and 97 used to emit the count byte as '*' / '}' -- an
        # unescaped RSP metacharacter inside the packet body.
        text = "z" * length
        packet = write_data(text)
        assert read_data(packet) == text.encode()
        assert packet.count(b"$") == 1, f"unescaped '$' in {packet!r}"
        assert packet.count(b"#") == 1, f"unescaped '#' in {packet!r}"

    def test_ack_only_input_is_ignored(self):
        assert read_data(b"+") is None

    def test_bad_checksum_is_rejected(self):
        assert read_data(b"$OK#00") is None

    def test_garbage_is_ignored(self):
        assert read_data(b"not a packet") is None
