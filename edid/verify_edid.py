# -*- coding: utf-8 -*-
"""verify_edid.py -- independent EDID checker  (run with: python -I verify_edid.py)

Second implementation, written from the DECODER side (Linux drm_edid.c /
include/drm/drm_edid.h semantics + VESA E-EDID 1.4 / CTA-861 tables), kept
deliberately separate from review/scripts/edid_gen_2k144.py so the two cross
validate ("double channel").  It:

  1. walks the 128 B base block + 128 B CTA-861 extension byte by byte,
  2. verifies both block checksums (sum of 128 B == 0 mod 256),
  3. decodes every field (vendor, input, features, chromaticity, established
     bitmap, standard timings, all detailed/monitor descriptors),
  4. re-derives the primary mode physics from first principles and checks the
     frozen requirements:
        2560x1440, refresh ~144 Hz, pixel clock <= 594 MHz,
        HTotal % 4 == 0 (IT68051 dual-pixel DDR), CVT-RB geometry
        (HBlank=160, HFP=48, HSW=32, VFP=3, VSYNC=4, +hsync -vsync),
        clock on the CVT 0.25 MHz grid, TTL PCLK = clock/4 <= 150 MHz,
  5. checks the range-limits descriptor would not reject 580 MHz / 144 Hz,
  6. walks the CTA data blocks: VDB, HDMI VSDB (Max_TMDS=340 MHz byte max),
     HF-VSDB directly after it (SCDC_Present=1, Max_TMDS_Character_Rate),
  7. prints the 256 B hex dump (compare against the generator output).

usage:  python -I verify_edid.py [path/to/edid.bin]
"""
import os
import sys

# ------------------------------------------------------------------- helpers
def fail(msg):
    print("  x FAIL: %s" % msg)
    return 1


def ok(msg):
    print("  . %s" % msg)
    return 0


def signed_ok(cond, msg):
    return ok(msg) if cond else fail(msg)


# ------------------------------------------------------------------ load file
default_bin = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "edid_2k144.bin")
path = sys.argv[1] if len(sys.argv) > 1 else default_bin
edid = open(path, "rb").read()
print("verify_edid: %s (%d bytes)" % (path, len(edid)))
print()

print("256-byte hex dump:")
for o in range(0, len(edid), 16):
    print("%02Xh: %s" % (o, " ".join("%02X" % x for x in edid[o:o + 16])))
print()

err = 0

if len(edid) != 256:
    err += fail("expected 256 bytes (128 base + 128 CTA ext), got %d" % len(edid))
    sys.exit(1)

blk0, blk1 = edid[0:128], edid[128:256]

# ------------------------------------------------------------------- checksum
print("[1] checksums (each 128-byte block must sum to 0 mod 256)")
for idx, b in ((0, blk0), (1, blk1)):
    s = sum(b)
    err += signed_ok(s % 256 == 0,
                     "block %d checksum: sum=%d, checksum byte=0x%02X"
                     % (idx, s, b[127]))
print()

# ------------------------------------------------------------------ base walk
print("[2] base block walk (VESA E-EDID 1.4 / struct edid)")
if blk0[:8] != bytes([0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00]):
    err += fail("fixed header pattern 00 FF..00 missing")
else:
    ok("header 00 FF FF FF FF FF FF 00")

mfg = (blk0[8] << 8) | blk0[9]
letters = "".join(chr(ord('@') + ((mfg >> s) & 0x1F)) for s in (10, 5, 0))
prod = blk0[10] | (blk0[11] << 8)
serial = int.from_bytes(blk0[12:16], "little")
year = 1990 + blk0[17]
ok("manufacturer '%s' (PNP 0x%04X), product 0x%04X, serial %d, week %d year %d"
   % (letters, mfg, prod, serial, blk0[16], year))
err += signed_ok(letters == "ZFW", "vendor placeholder ZFW")
err += signed_ok((mfg >> 15) == 0, "PNP bit15 reserved=0")

err += signed_ok(blk0[18] == 1 and blk0[19] == 4,
                 "EDID version %d.%d (expect 1.4)" % (blk0[18], blk0[19]))

inp = blk0[20]
digital = bool(inp & 0x80)
depth = (inp >> 4) & 0x07
iface = inp & 0x0F
iface_names = {0: "undefined", 1: "DVI", 2: "HDMI-a", 3: "HDMI-b",
               4: "MDDI", 5: "DisplayPort"}
err += signed_ok(digital and depth == 2 and iface == 2,
                 "video input: digital, %d bpc, %s (byte 0x14=0x%02X)"
                 % ({1: 6, 2: 8, 3: 10, 4: 12, 5: 14, 6: 16}.get(depth, "?"),
                    iface_names.get(iface, "?"), inp))

feat = blk0[24]
dtype = (feat >> 3) & 0x03
err += signed_ok(dtype == 3 and feat & 0x04 and feat & 0x02 and feat & 0x01,
                 "features 0x%02X: RGB444+YCrCb444+YCrCb422, sRGB, "
                 "preferred DTD, continuous freq" % feat)
err += signed_ok(blk0[23] == 120, "gamma %.2f" % (blk0[23] / 100.0 + 1.0))

# chromaticity: floor(v*1024); byte 0x19 = Rx,Ry,Gx,Gy 2 LSBs each,
# byte 0x1A = Bx,By,Wx,Wy 2 LSBs each, bytes 0x1B.. = 8-bit MSBs
lo = blk0[0x19]
lo2 = blk0[0x1A]
msb = blk0[0x1B:0x23]
Rx = (((lo >> 6) & 3) << 8) | msb[0]
Ry = (((lo >> 4) & 3) << 8) | msb[1]
Gx = (((lo >> 2) & 3) << 8) | msb[2]
Gy = ((lo & 3) << 8) | msb[3]
Bx = (((lo2 >> 6) & 3) << 8) | msb[4]
By = (((lo2 >> 4) & 3) << 8) | msb[5]
Wx = (((lo2 >> 2) & 3) << 8) | msb[6]
Wy = ((lo2 & 3) << 8) | msb[7]
srgb = {"Rx": 0.640, "Ry": 0.330, "Gx": 0.300, "Gy": 0.600,
        "Bx": 0.150, "By": 0.060, "Wx": 0.3127, "Wy": 0.3290}
got = {"Rx": Rx, "Ry": Ry, "Gx": Gx, "Gy": Gy,
       "Bx": Bx, "By": By, "Wx": Wx, "Wy": Wy}
bad = [k for k, v in got.items() if abs(v / 1024.0 - srgb[k]) > 1.0 / 1024]
err += signed_ok(not bad, "chromaticity = sRGB standard values "
                 + " ".join("%s=%.3f" % (k, got[k] / 1024.0) for k in got)
                 + ("" if not bad else " (mismatch: %s)" % bad))

# established timings bitmap (LSB order per Wikipedia table / kernel list)
est_names = [
    (0, "800x600@60"), (1, "800x600@56"), (2, "640x480@75"),
    (3, "640x480@72"), (4, "640x480@67"), (5, "640x480@60"),
    (6, "720x400@88"), (7, "720x400@70"),
    (8, "800x600@72"), (9, "800x600@75"), (10, "832x624@75"),
    (11, "1024x768@87i"), (12, "1024x768@60"), (13, "1024x768@70"),
    (14, "1024x768@75"), (15, "1280x1024@75"),
]
est = blk0[0x23] | (blk0[0x24] << 8) | ((blk0[0x25] & 0x80) << 9)
est_set = [n for bit, n in est_names if est & (1 << bit)]
if blk0[0x25] & 0x7F:
    est_set.append("1152x870@75")
ok("established timings: %s (bytes 0x23-25 = %02X %02X %02X)"
   % (", ".join(est_set) or "none", blk0[0x23], blk0[0x24], blk0[0x25]))
err += signed_ok("800x600@60" in est_set and "640x480@60" in est_set,
                 "universal 60 Hz fallbacks present")

aspect_names = {0: "16:10", 1: "4:3", 2: "5:4", 3: "16:9"}
aspect_div = {0: 16 / 10.0, 1: 4 / 3.0, 2: 5 / 4.0, 3: 16 / 9.0}
std_list = []
for i in range(8):
    b1, b2 = blk0[38 + 2 * i], blk0[39 + 2 * i]
    if (b1, b2) in ((0x01, 0x01), (0x00, 0x00), (0x20, 0x20)):
        continue
    w = (b1 + 31) * 8
    h = int(w * 9 / 16) if (b2 >> 6) == 3 else \
        int(w * 5 / 4) if (b2 >> 6) == 2 else \
        int(w * 3 / 4) if (b2 >> 6) == 1 else int(w * 10 / 16)
    std_list.append("%dx%d@%d(%s)" % (w, h, (b2 & 0x3F) + 60,
                                      aspect_names[b2 >> 6]))
ok("standard timings: %s" % ", ".join(std_list))
err += signed_ok("1920x1080@60(16:9)" in std_list,
                 "std timing 1920x1080@60 present")
err += signed_ok("1440x900@60(16:10)" in std_list,
                 "std timing 1440x900@60 present")

# detailed timing / monitor descriptor slots
print()
print("[3] detailed timing / monitor descriptors (base block)")
def decode_dtd(b, where):
    """decode per detailed_pixel_timing; return dict or None if not a DTD."""
    if b[0] == 0 and b[1] == 0:
        return None
    d = {}
    d["clock_khz"] = (b[0] | (b[1] << 8)) * 10     # 10 kHz units -> kHz
    d["hactive"] = ((b[4] & 0xF0) << 4) | b[2]
    d["hblank"] = ((b[4] & 0x0F) << 8) | b[3]
    d["vactive"] = ((b[7] & 0xF0) << 4) | b[5]
    d["vblank"] = ((b[7] & 0x0F) << 8) | b[6]
    d["hfp"] = ((b[11] & 0xC0) << 2) | b[8]
    d["hsw"] = ((b[11] & 0x30) << 4) | b[9]
    d["vfp"] = ((b[11] & 0x0C) << 2) | (b[10] >> 4)
    d["vsw"] = ((b[11] & 0x03) << 4) | (b[10] & 0x0F)
    d["wmm"] = b[12] | ((b[14] & 0xF0) << 4)
    d["hmm"] = b[13] | ((b[14] & 0x0F) << 8)
    d["misc"] = b[17]
    d["interlaced"] = bool(b[17] & 0x80)
    d["sep_sync"] = (b[17] & 0x18) == 0x18
    d["hpos"] = bool(b[17] & 0x02)
    d["vpos"] = bool(b[17] & 0x04)
    d["htotal"] = d["hactive"] + d["hblank"]
    d["vtotal"] = d["vactive"] + d["vblank"]
    d["refresh_hz"] = d["clock_khz"] * 1000.0 / (d["htotal"] * d["vtotal"])
    d["hfreq_khz"] = d["clock_khz"] / d["htotal"]
    return d

desc_types = {0xFC: "monitor name", 0xFD: "range limits",
              0xFF: "serial", 0xFE: "text", 0xF8: "CVT 3-byte codes"}

for slot, off in enumerate((54, 72, 90, 108)):
    b = blk0[off:off + 18]
    d = decode_dtd(b, "slot %d" % slot)
    if d is None:
        dtype_b = b[3]
        name = desc_types.get(dtype_b, "type 0x%02X" % dtype_b)
        if dtype_b in (0xFC, 0xFE, 0xFF):
            txt = bytes(b[5:18]).rstrip(b" ").rstrip(b"\n").decode("ascii", "replace")
            ok("slot %d @0x%02X: %s descriptor \"%s\"" % (slot, off, name, txt))
            if slot == 2:
                err += signed_ok(txt == "ZFW-CAP-2K", "monitor name ZFW-CAP-2K")
        elif dtype_b == 0xFD:
            print("  . slot %d @0x%02X: range limits descriptor (checked in [5])"
                  % (slot, off))
        else:
            ok("slot %d @0x%02X: non-pixel descriptor type 0x%02X"
               % (slot, off, dtype_b))
        continue
    print("  . slot %d @0x%02X: DTD %d.%04d MHz  %dx%d %s%s/%s%s, HTot %d VTot %d"
          "  HFP%d HSW%d VFP%d VSW%d  %s%s %s"
          % (slot, off, d["clock_khz"] // 1000, d["clock_khz"] % 1000,
             d["hactive"], d["vactive"], "+" if d["hpos"] else "-",
             "h", "+" if d["vpos"] else "-", "v",
             d["htotal"], d["vtotal"], d["hfp"], d["hsw"], d["vfp"], d["vsw"],
             "sep-sync" if d["sep_sync"] else "SYNC-TYPE?",
             " interlaced" if d["interlaced"] else "",
             "image %dx%dmm" % (d["wmm"], d["hmm"]) if d["wmm"] else ""))
    if slot == 0:
        dtd1 = d
    elif slot == 1:
        dtd2 = d

print()
print("[4] primary timing re-derivation (frozen requirements)")
# CVT-RB constants (verified against official DMT RB entries in drm_dmt_modes)
RB_HBLANK, RB_HFP, RB_HSW = 160, 48, 32
RB_VFP, RB_VSYNC = 3, 4
err += signed_ok(dtd1["hactive"] == 2560 and dtd1["vactive"] == 1440,
                 "active area 2560x1440")
err += signed_ok(dtd1["htotal"] % 4 == 0,
                 "HTotal %d %% 4 == 0 (IT68051 dual-pixel DDR)" % dtd1["htotal"])
err += signed_ok(dtd1["hblank"] == RB_HBLANK and dtd1["hfp"] == RB_HFP and
                 dtd1["hsw"] == RB_HSW and dtd1["vfp"] == RB_VFP and
                 dtd1["vsw"] == RB_VSYNC,
                 "CVT-RB geometry: HBlank=160 HFP=48 HSW=32 VFP=3 VSYNC=4")
err += signed_ok(dtd1["hpos"] and not dtd1["vpos"] and dtd1["sep_sync"],
                 "polarities +hsync -vsync (CVT-RB), separate sync")
err += signed_ok(dtd1["clock_khz"] % 250 == 0,
                 "pixel clock %d.%03d MHz on CVT 0.25 MHz grid"
                 % (dtd1["clock_khz"] // 1000, dtd1["clock_khz"] % 1000))
err += signed_ok(dtd1["clock_khz"] <= 594000,
                 "pixel clock %.3f MHz <= 594 MHz (HDMI 2.0 / 594 Mcsc)"
                 % (dtd1["clock_khz"] / 1000.0))
err += signed_ok(abs(dtd1["refresh_hz"] - 144.0) <= 0.5,
                 "refresh %.3f Hz ~= 144 (hfreq %.3f kHz)"
                 % (dtd1["refresh_hz"], dtd1["hfreq_khz"]))
err += signed_ok(dtd1["clock_khz"] / 4 <= 150000,
                 "IT68051 dual-pixel DDR TTL PCLK = clock/4 = %.1f MHz <= 150"
                 % (dtd1["clock_khz"] / 4000.0))
err += signed_ok(not dtd1["interlaced"], "progressive")

# secondary DTD must be CEA VIC 16 exact
vic16 = dict(clock_khz=148500, hactive=1920, hblank=280, vactive=1080,
             vblank=45, hfp=88, hsw=44, vfp=4, vsw=5)
err += signed_ok(all(dtd2[k] == v for k, v in vic16.items()),
                 "DTD2 = 1920x1080@60 exact (VIC 16 formal timing, 148.5 MHz)")

# ---------------------------------------------------------------- range limits
print()
print("[5] range limits descriptor")
if blk0[108:112] != bytes([0, 0, 0, 0xFD]):
    err += fail("range limits descriptor not at slot 4 (0x6C)")
else:
    r = blk0[108:126]
    off = r[4]
    vmin, vmax = r[5], r[6]
    hmin, hmax = r[7], r[8]
    pclk10 = r[9]
    flag = r[10]
    if off & 0x01:
        vmin += 255
    if off & 0x02:
        vmax += 255
    if off & 0x04:
        hmin += 255
    if off & 0x08:
        hmax += 255
    ok("offsets 0x%02X, v %d-%d Hz, h %d-%d kHz, max pclk %d MHz (byte %d), "
       "flags 0x%02X" % (off, vmin, vmax, hmin, hmax, pclk10 * 10, pclk10, flag))
    err += signed_ok(flag == 0x01,
                     "'range limits only' (0x01): any timing in range accepted")
    err += signed_ok(vmin <= 60 and vmax >= 144,
                     "vertical range covers 60 Hz and 144 Hz")
    err += signed_ok(hmin <= 31 and hmax >= 214,
                     "horizontal range covers 31.5 kHz and 213.2 kHz")
    err += signed_ok(pclk10 * 10 >= 600,
                     "max pixel clock %d MHz >= 600 (580 MHz mode never "
                     "rejected; kernel range_pixel_clock: %d MHz)"
                     % (pclk10 * 10, pclk10 * 10))
    err += signed_ok(bytes(r[11:18]) == bytes([0x0A] + [0x20] * 6),
                     "bytes 11-17 = spec padding 0A 20 20 20 20 20 20")

# ------------------------------------------------------------------- extension
print()
print("[6] CTA-861 extension walk")
err += signed_ok(blk1[0] == 0x02, "extension tag 0x02 (CTA)")
err += signed_ok(blk1[1] == 0x03, "revision 3")
d_off = blk1[2]
err += signed_ok(4 <= d_off <= 127 and d_off % 4 == 0,
                 "DTD start offset %d (>=4, 4-aligned, after all data blocks)" % d_off)
ext_flags = blk1[3]
err += signed_ok(ext_flags & 0x20, "YCbCr 4:4:4 supported (byte3 bit5)")
err += signed_ok(ext_flags & 0x10, "YCbCr 4:2:2 supported (byte3 bit4)")
err += signed_ok(not ext_flags & 0x40, "basic audio NOT claimed (video-only sink)")

off = 4
blocks = []
while off < d_off:
    tag = blk1[off] >> 5
    ln = blk1[off] & 0x1F
    blocks.append((off, tag, ln, blk1[off + 1:off + 1 + ln]))
    off += 1 + ln

for b_off, tag, ln, payload in blocks:
    if tag == 2:
        svds = ["VIC %d%s" % (s & 0x7F,
                              "+native" if (s & 0x80) and (s & 0x7F) <= 64 else "")
                for s in payload]
        ok("@%02d Video Data Block: %s" % (b_off, ", ".join(svds)))
        err += signed_ok(0x90 in payload, "VIC 16 (1080p60) with native bit")
    elif tag == 3:
        oui = payload[0] | (payload[1] << 8) | (payload[2] << 16)
        if oui == 0x000C03:
            ok("@%02d HDMI VSDB (H14b), len %d" % (b_off, ln))
            phys = "A=%d B=%d C=%d D=%d" % (
                payload[3] >> 4, payload[3] & 0xF,
                payload[4] >> 4, payload[4] & 0xF)
            err += signed_ok(payload[3] == 0x10 and payload[4] == 0x00,
                             "physical address 1.0.0.0 (%s)" % phys)
            err += signed_ok(payload[5] == 0x00,
                             "no DVI-dual / no deep-color claims (8 bpc sink)")
            maxtmds = payload[6] * 5
            err += signed_ok(payload[6] == 0x44,
                             "Max_TMDS_Clock = %d MHz (0x44, HDMI 1.4 byte max)"
                             % maxtmds)
            err += signed_ok(payload[7] == 0x20,
                             "HDMI_Video_Present=1 (infoframes accepted)")
            vsdb_off = b_off
        elif oui == 0xC45DD8:
            ok("@%02d HDMI Forum VSDB (HF-VSDB), len %d" % (b_off, ln))
            err += signed_ok(ln >= 7, "payload length %d >= 7" % ln)
            err += signed_ok(payload[3] >= 1, "version %d" % payload[3])
            maxchar = payload[4] * 5
            err += signed_ok(payload[4] == 0x78 and maxchar == 600,
                             "Max_TMDS_Character_Rate = %d MHz (0x78; kernel "
                             "max_tmds_clock=%d kHz since >340)" % (maxchar, maxchar * 1000))
            scdc = payload[5]
            err += signed_ok(bool(scdc & 0x80), "SCDC_Present = 1 (HDMI 2.0 "
                             "signalling above 340 Mcsc)")
            err += signed_ok(not scdc & 0x40, "RR_Capable = 0 (no SCDC polling)")
            frl = payload[6] >> 4
            err += signed_ok(frl == 0, "Max_FRL_Rate = 0 (HDMI 2.0, not 2.1)")
            err += signed_ok(b_off == vsdb_off + 9,
                             "HF-VSDB placed immediately after HDMI VSDB "
                             "(HDMI 2.0 ordering)")
        else:
            ok("@%02d vendor VSDB OUI 0x%06X" % (b_off, oui))
    else:
        ok("@%02d data block tag %d len %d" % (b_off, tag, ln))

print()
print("[7] extension DTD section")
if d_off:
    n = (127 - d_off) // 18
    any_dtd = False
    for i in range(n):
        b = blk1[d_off + 18 * i:d_off + 18 * i + 18]
        d = decode_dtd(b, "ext")
        if d:
            any_dtd = True
            ok("ext DTD %d: %d.%03d MHz %dx%d HTot %d VTot %d (%dx%d@%.0f)"
               % (i, d["clock_khz"] // 1000, d["clock_khz"] % 1000,
                  d["hactive"], d["vactive"], d["htotal"], d["vtotal"],
                  d["hactive"], d["vactive"], d["refresh_hz"]))
            err += signed_ok(d["hactive"] == 1440 and d["vactive"] == 900
                             and abs(d["refresh_hz"] - 60) < 0.5
                             and d["htotal"] % 4 == 0,
                             "ext fallback 1440x900@60, HTotal %d %%4==0"
                             % d["htotal"])
    if not any_dtd:
        ok("no DTDs in extension (allowed)")

# --------------------------------------------------------------------- summary
print()
if err:
    print("RESULT: %d check(s) FAILED" % err)
    sys.exit(1)
print("RESULT: ALL CHECKS PASSED (checksums, structure walk, timing "
      "re-derivation, HDMI 2.0 VSDB/HF-VSDB, range limits)")
