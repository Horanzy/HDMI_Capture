#!/usr/bin/env python3
"""Cycle/bit-exact model of video_stream's packed-422 gearbox.

This is a targeted model, not an RTL simulation.  It checks the exact 80->128
accumulator recurrence, final-line drainage, pin-to-pair mapping and common
resolution centre crop calculations.
"""
import random


def rtl_pair40(word):
    """IT68051 YUV422: A_R=Cb, A_G=Y0, B_R=Cr, B_G=Y1."""
    return (((word >> 50) & 0x3FF) << 30 | ((word >> 40) & 0x3FF) << 20 |
            ((word >> 10) & 0x3FF) << 10 | ((word >> 20) & 0x3FF))


def check_pair_mapping():
    word = ((0x155 << 50) | (0x2AA << 40) | (0x3FF << 30) |
            (0x0AB << 20) | (0x123 << 10) | 0x3CD)
    # Cr=155, Y1=2AA, Y0=123, Cb=0AB; unused slots do not enter pair40.
    assert rtl_pair40(word) == (0x155 << 30 | 0x2AA << 20 | 0x123 << 10 | 0x0AB)


def check_gearbox(seed=68051):
    rng = random.Random(seed)
    pairs = [rng.getrandbits(80) for _ in range(160)]
    pack_acc = 0
    pack_count = 0
    pay_cnt = 0
    emitted = []
    raw_last_seen = False
    for pair in pairs:
        pack_sum = pack_acc | (pair << pack_count)
        pack_emit = pack_count >= 48
        if pack_emit:
            emitted.append(pack_sum & ((1 << 128) - 1))
            pack_acc = pack_sum >> 128
            pack_count -= 48
        else:
            pack_acc = pack_sum
            pack_count += 80
        raw_last = pay_cnt == 99 and pack_emit
        if raw_last:
            assert not raw_last_seen
            raw_last_seen = True
            pack_acc = 0
            pack_count = 0
        elif pack_emit:
            pay_cnt += 1
    assert raw_last_seen
    assert len(emitted) == 100
    assert pay_cnt == 99  # final word closes the packet instead of incrementing
    assert pack_acc == 0 and pack_count == 0
    expected = sum(pair << (80 * i) for i, pair in enumerate(pairs))
    actual = sum(word << (128 * i) for i, word in enumerate(emitted))
    assert actual == expected
    assert len(emitted) * 16 == 1600


def check_centres():
    cases = {
        (2560, 1440): (960, 400),
        (2560, 1600): (960, 480),
        (1920, 1080): (640, 220),
        (1920, 1200): (640, 280),
        (3840, 2160): (1600, 760),
    }
    for (width, height), expected in cases.items():
        x0, y0 = (width - 640) // 2, (height - 640) // 2
        assert (x0, y0) == expected
        assert x0 % 4 == 0 and y0 % 2 == 0


def main():
    check_pair_mapping()
    check_gearbox()
    check_centres()
    print("PASS: YUV422 pin map, 160 PCLK -> 100x128-bit payload words, "
          "zero final remainder, common-resolution centre crop")


if __name__ == "__main__":
    main()
