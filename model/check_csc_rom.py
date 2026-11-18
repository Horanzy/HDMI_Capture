#!/usr/bin/env python3
"""Check every synthesized CSC ROM byte against the authoritative EVB table."""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / "docs" / "refs" / "evb_src" / "iTE6805_CSC_Table.h"
ROM = ROOT / "rtl" / "csc_matrix_rom.v"

NAMES = [
    "bCSCMtx_RGB2YUV_ITU601_16_235",
    "bCSCMtx_RGB2YUV_ITU601_00_255",
    "bCSCMtx_RGB2YUV_ITU709_16_235",
    "bCSCMtx_RGB2YUV_ITU709_00_255",
]


def main():
    source = REFERENCE.read_text(encoding="utf-8", errors="replace")
    rtl = ROM.read_text(encoding="utf-8")
    for sel, name in enumerate(NAMES):
        match = re.search(r"/\*" + re.escape(name) + r"\*/\s*\{(.*?)\}", source, re.S)
        assert match, name
        body = re.sub(r"/\*.*?\*/", "", match.group(1), flags=re.S)
        expected = [int(x, 16) for x in re.findall(r"0x([0-9A-Fa-f]+)", body)]
        start_token = "2'd%d: case (byte_index)" % sel if sel < 3 else "default: case (byte_index)"
        start = rtl.find(start_token)
        assert start >= 0, sel
        if sel < 2:
            end = rtl.find("2'd%d: case (byte_index)" % (sel + 1), start + 1)
        elif sel == 2:
            end = rtl.find("default: case (byte_index)", start + 1)
        else:
            end = rtl.find("\n        endcase", start)
        assert end > start, sel
        actual = [int(x, 16) for x in
                  re.findall(r"\d+:byte_data=8'h([0-9A-Fa-f]+)", rtl[start:end])]
        assert len(expected) == len(actual) == 22, (name, len(expected), len(actual))
        assert actual == expected, (name, actual, expected)
    print("PASS: 4 CSC matrices x 22 bytes exactly match iTE6805_CSC_Table.h")


if __name__ == "__main__":
    main()
