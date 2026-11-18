#!/usr/bin/env python3
"""Offline protocol tests for linux_rx.Receiver; no USB hardware required."""
import io
import random
import struct
from types import SimpleNamespace

from linux_rx import (CROP_H, CROP_W, FLAG_EOLF, FMT, FMT_PACKED42210, HDR_BYTES,
                      PACKED422_LINE_BYTES, RAW_LINE_BYTES, Receiver, SYNC,
                      TYPE_LINE, TYPE_STATUS, unpack_frame)


def packet(fmt, line, payload=b"", status=0):
    typ = TYPE_STATUS if line == 0xFFFF else TYPE_LINE
    flags = (typ << 5) | (fmt << 2) | (FLAG_EOLF if typ == TYPE_STATUS or line == CROP_H - 1 else 0)
    w2 = flags | SYNC[0] << 8 | SYNC[1] << 16 | SYNC[2] << 24
    return struct.pack("<IIII", 7, line, w2, status << 16) + payload


def stream_for(fmt, missing=None):
    chunks = []
    expected = []
    size = PACKED422_LINE_BYTES if fmt == FMT_PACKED42210 else RAW_LINE_BYTES
    for line in range(CROP_H):
        if line == missing:
            continue
        row = bytes(((line + i + fmt) & 0xFF) for i in range(size))
        chunks.append(packet(fmt, line, row))
        expected.append((line, row))
    chunks.append(packet(fmt, 0xFFFF, status=0x1234))
    return b"".join(chunks), expected


def run_case(fmt, fragmented=True, corrupt_prefix=False, missing=None):
    stream, expected = stream_for(fmt, missing)
    if corrupt_prefix:
        stream = b"not-aligned" + stream
    rx = Receiver()
    rx.setup(SimpleNamespace(urb_lines=4, save=None, dump_frames=0))
    rx.out = io.BytesIO()
    rng = random.Random(100 + fmt)
    off = 0
    while off < len(stream):
        step = rng.choice([1, 3, 17, 272, 1024, 1296, 1616, 2576]) if fragmented else len(stream)
        rx.feed(stream[off:off + step], float(off) / 1e9)
        off += step
    if missing is None:
        assert rx.frames_done == 1, (fmt, rx.frames_done, rx.frames_dropped)
        assert rx.frames_dropped == 0
        raw = rx.out.getvalue()
        hdr = struct.unpack_from("<4sBBHHI2x", raw)
        assert hdr[:6] == (b"BFRA", fmt, FMT[fmt][1], CROP_W, CROP_H, 7)
        frame = raw[16:]
        converted = unpack_frame(b"".join(row for _, row in expected), fmt)
        out_row = CROP_W * FMT[fmt][1]
        assert len(frame) == CROP_H * out_row, (fmt, len(frame), CROP_H * out_row, hdr)
        for line, row in expected:
            start = line * out_row
            assert frame[start:start + out_row] == converted[start:start + out_row]
        assert rx.status_word == 0x1234
        assert not rx.pending
    else:
        assert rx.frames_done == 0
        assert rx.frames_dropped == 1
    if corrupt_prefix:
        assert rx.resyncs > 0
    rx.out = None


def test_unpack_mapping():
    raw = bytearray(RAW_LINE_BYTES * CROP_H)
    slot_a = (0x155 << 20) | (0x2AA << 10) | 0x3FF
    slot_b = (0x155 << 20) | (0x100 << 10)
    struct.pack_into("<II", raw, 0, slot_a, slot_b)
    expected = {
        0: bytes((0x55, 0xAA, 0x55, 0x40)),
        1: bytes((0x55, 0xAA, 0xFF, 0)),
        2: bytes((0xAA, 0xFF, 0x55, 0)),
        3: struct.pack("<I", 0x155 | (0x2AA << 10) | (0x3FF << 20)),
        4: struct.pack("<I", 0x2AA | (0x3FF << 10) | (0x155 << 20)),
    }
    for fmt, prefix in expected.items():
        decoded = unpack_frame(raw, fmt)
        assert decoded[:len(prefix)] == prefix, (fmt, decoded[:8], prefix)

    packed = bytearray(PACKED422_LINE_BYTES * CROP_H)
    pair = 0x3FF | (0x2AA << 10) | (0x100 << 20) | (0x155 << 30)
    packed[:5] = pair.to_bytes(5, "little")
    decoded = unpack_frame(packed, FMT_PACKED42210)
    assert decoded[:8] == struct.pack("<4H", 0x3FF, 0x2AA, 0x155, 0x100)


def test_pure_python_mapping():
    import builtins
    original = builtins.__import__
    def without_numpy(name, *args, **kwargs):
        if name == "numpy":
            raise ImportError("forced fallback test")
        return original(name, *args, **kwargs)
    builtins.__import__ = without_numpy
    try:
        test_unpack_mapping()
    finally:
        builtins.__import__ = original


def main():
    test_unpack_mapping()
    test_pure_python_mapping()
    for fmt in FMT:
        run_case(fmt)
        run_case(fmt, corrupt_prefix=True)
    run_case(0, missing=137)
    print("PASS: 6 formats, packed42210, arbitrary fragmentation, resync, status, incomplete-frame drop")


if __name__ == "__main__":
    main()
