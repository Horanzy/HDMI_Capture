#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Low-latency Linux receiver for the IT68051 / GW5A / FT601 capture card.

The FPGA automatically follows IT68051 input color format; there is no host
format-selection command.  AUTO_FORMAT raw LINE packets have a 16-byte header
and 2560-byte payload.  The default fixed-422 firmware uses format 5 and a
lossless 1600-byte packed 10-bit payload.  STATUS packets are header-only.
Header bytes 9..11 are the fixed sync marker C2 51 A5, so the parser can resync
at arbitrary byte boundaries.  Raw libusb uses a real asynchronous IN-URB
queue; pyusb remains a fallback.
"""

import argparse
import ctypes
import ctypes.util
import struct
import sys
import threading
import time
from collections import deque

HDR_BYTES = 16
CROP_W = CROP_H = 640
RAW_LINE_BYTES = CROP_W * 4
PACKED422_LINE_BYTES = CROP_W * 5 // 2
FMT_PACKED42210 = 5
TYPE_LINE, TYPE_STATUS = 0, 1
FLAG_EOLF, FLAG_OVERRUN = 0x01, 0x02
SYNC = b"\xC2\x51\xA5"
DEF_VID, DEF_PID = 0x0403, 0x601F
FMT = {
    0: ("UYVY8", 2, "uyvy"),
    1: ("RGB888", 4, "rgbx"),
    2: ("YUV4448", 4, "yuvx"),
    3: ("RGB10", 4, "rgb10"),
    4: ("YUV44410", 4, "yuv10"),
    5: ("YUV42210_16LE", 4, "yuv42210"),  # Cb,Y0,Cr,Y1; low 10 bits per u16
}


# ---------------------------------------------------------------------------
# Minimal LP64 libusb-1.0 binding.  Offsets below are from libusb.h; this raw
# path deliberately rejects non-64-bit Python instead of corrupting transfers.
# ---------------------------------------------------------------------------
LIBUSB_TRANSFER_COMPLETED = 0
LIBUSB_TRANSFER_CANCELLED = 3
LIBUSB_TRANSFER_STALL = 4
USB_TRANSFER_TYPE_BULK = 3
OFF_FLAGS, OFF_EP, OFF_TYPE, OFF_TIMEOUT = 8, 9, 10, 12
OFF_STATUS, OFF_LENGTH, OFF_ACTUAL = 16, 20, 24
OFF_CALLBACK, OFF_USER, OFF_BUFFER = 32, 40, 48


class LibusbError(Exception):
    pass


class _EndpointDesc(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8), ("bDescriptorType", ctypes.c_uint8),
        ("bEndpointAddress", ctypes.c_uint8), ("bmAttributes", ctypes.c_uint8),
        ("wMaxPacketSize", ctypes.c_uint16), ("bInterval", ctypes.c_uint8),
        ("bRefresh", ctypes.c_uint8), ("bSynchAddress", ctypes.c_uint8),
        ("extra", ctypes.c_void_p), ("extra_length", ctypes.c_int),
    ]


class _InterfaceDesc(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8), ("bDescriptorType", ctypes.c_uint8),
        ("bInterfaceNumber", ctypes.c_uint8), ("bAlternateSetting", ctypes.c_uint8),
        ("bNumEndpoints", ctypes.c_uint8), ("bInterfaceClass", ctypes.c_uint8),
        ("bInterfaceSubClass", ctypes.c_uint8), ("bInterfaceProtocol", ctypes.c_uint8),
        ("iInterface", ctypes.c_uint8), ("endpoint", ctypes.c_void_p),
        ("extra", ctypes.c_void_p), ("extra_length", ctypes.c_int),
    ]


class _Interface(ctypes.Structure):
    _fields_ = [("num_altsetting", ctypes.c_int), ("altsetting", ctypes.c_void_p)]


class _ConfigDesc(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8), ("bDescriptorType", ctypes.c_uint8),
        ("wTotalLength", ctypes.c_uint16), ("bNumInterfaces", ctypes.c_uint8),
        ("bConfigurationValue", ctypes.c_uint8), ("iConfiguration", ctypes.c_uint8),
        ("bmAttributes", ctypes.c_uint8), ("MaxPower", ctypes.c_uint8),
        ("interface", ctypes.c_void_p), ("extra", ctypes.c_void_p),
        ("extra_length", ctypes.c_int),
    ]


class LibusbAsync:
    def __init__(self):
        if ctypes.sizeof(ctypes.c_void_p) != 8:
            raise LibusbError("raw libusb path requires 64-bit Python")
        path = ctypes.util.find_library("usb-1.0")
        names = ([path] if path else []) + ["libusb-1.0.so.0", "libusb-1.0.so"]
        self.dll = None
        for name in names:
            try:
                self.dll = ctypes.CDLL(name)
                break
            except OSError:
                pass
        if self.dll is None:
            raise LibusbError("libusb-1.0 not found")
        d = self.dll
        d.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        d.libusb_exit.argtypes = [ctypes.c_void_p]
        d.libusb_get_device_list.argtypes = [ctypes.c_void_p,
                                             ctypes.POINTER(ctypes.c_void_p)]
        d.libusb_get_device_list.restype = ctypes.c_ssize_t
        d.libusb_free_device_list.argtypes = [ctypes.c_void_p, ctypes.c_int]
        d.libusb_get_device_descriptor.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        d.libusb_open.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)]
        d.libusb_close.argtypes = [ctypes.c_void_p]
        d.libusb_set_configuration.argtypes = [ctypes.c_void_p, ctypes.c_int]
        d.libusb_claim_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
        d.libusb_release_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
        d.libusb_set_auto_detach_kernel_driver.argtypes = [ctypes.c_void_p, ctypes.c_int]
        d.libusb_get_active_config_descriptor.argtypes = [ctypes.c_void_p,
                                                         ctypes.POINTER(ctypes.c_void_p)]
        d.libusb_free_config_descriptor.argtypes = [ctypes.c_void_p]
        d.libusb_alloc_transfer.argtypes = [ctypes.c_int]
        d.libusb_alloc_transfer.restype = ctypes.c_void_p
        d.libusb_free_transfer.argtypes = [ctypes.c_void_p]
        d.libusb_submit_transfer.argtypes = [ctypes.c_void_p]
        d.libusb_submit_transfer.restype = ctypes.c_int
        d.libusb_cancel_transfer.argtypes = [ctypes.c_void_p]
        d.libusb_clear_halt.argtypes = [ctypes.c_void_p, ctypes.c_ubyte]
        d.libusb_clear_halt.restype = ctypes.c_int
        d.libusb_handle_events_timeout.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        d.libusb_handle_events_timeout.restype = ctypes.c_int
        d.libusb_error_name.argtypes = [ctypes.c_int]
        d.libusb_error_name.restype = ctypes.c_char_p
        self.ctx = ctypes.c_void_p()
        if d.libusb_init(ctypes.byref(self.ctx)) != 0:
            raise LibusbError("libusb_init failed")
        self.handle = None
        self.device = None
        self.ifnum = 0
        self.tv = (ctypes.c_long * 2)(0, 200000)
        self.cb_type = ctypes.CFUNCTYPE(None, ctypes.c_void_p)
        self._cb_keepalive = []
        self.orphans = []  # transfers whose cancellation callback never arrived

    def strerror(self, code):
        name = self.dll.libusb_error_name(code)
        return name.decode() if name else "libusb error %d" % code

    def open_device(self, vid, pid, ifnum=0):
        d = self.dll
        lst = ctypes.c_void_p()
        n = d.libusb_get_device_list(self.ctx, ctypes.byref(lst))
        if n < 0:
            raise LibusbError("get_device_list: " + self.strerror(n))
        dev = None
        try:
            for i in range(n):
                dp = ctypes.c_void_p(lst.value + i * ctypes.sizeof(ctypes.c_void_p))
                dd = (ctypes.c_ubyte * 18)()
                if d.libusb_get_device_descriptor(dp, dd) != 0:
                    continue
                if (dd[8] | dd[9] << 8) == vid and (dd[10] | dd[11] << 8) == pid:
                    dev = dp
                    break
            if dev is None:
                raise LibusbError("device %04x:%04x not found" % (vid, pid))
            h = ctypes.c_void_p()
            r = d.libusb_open(dev, ctypes.byref(h))
            if r != 0:
                raise LibusbError("open: " + self.strerror(r))
            self.handle, self.device, self.ifnum = h, dev, ifnum
            d.libusb_set_auto_detach_kernel_driver(h, 1)
            r = d.libusb_set_configuration(h, 1)
            if r != 0:
                print("note: set_configuration(1):", self.strerror(r))
            r = d.libusb_claim_interface(h, ifnum)
            if r != 0:
                raise LibusbError("claim_interface: " + self.strerror(r))
        finally:
            d.libusb_free_device_list(lst, 1)
        return self.bulk_endpoints()

    def bulk_endpoints(self):
        cfg = ctypes.c_void_p()
        r = self.dll.libusb_get_active_config_descriptor(self.device, ctypes.byref(cfg))
        if r != 0:
            raise LibusbError("get_active_config_descriptor: " + self.strerror(r))
        ep_in = ep_out = None
        mps = 1024
        try:
            c = ctypes.cast(cfg, ctypes.POINTER(_ConfigDesc)).contents
            for i in range(c.bNumInterfaces):
                ip = c.interface + i * ctypes.sizeof(_Interface)
                alt = ctypes.cast(ip, ctypes.POINTER(_Interface)).contents
                for j in range(alt.num_altsetting):
                    ap = alt.altsetting + j * ctypes.sizeof(_InterfaceDesc)
                    iface = ctypes.cast(ap, ctypes.POINTER(_InterfaceDesc)).contents
                    if iface.bInterfaceNumber != self.ifnum:
                        continue
                    for k in range(iface.bNumEndpoints):
                        ep = iface.endpoint + k * ctypes.sizeof(_EndpointDesc)
                        e = ctypes.cast(ep, ctypes.POINTER(_EndpointDesc)).contents
                        if (e.bmAttributes & 3) != 2:
                            continue
                        if e.bEndpointAddress & 0x80:
                            ep_in = ep_in or e.bEndpointAddress
                            mps = e.wMaxPacketSize & 0x7FF
                        else:
                            ep_out = ep_out or e.bEndpointAddress
        finally:
            self.dll.libusb_free_config_descriptor(cfg)
        if ep_in is None:
            raise LibusbError("no bulk IN endpoint in interface %d" % self.ifnum)
        return ep_in, ep_out, mps

    def make_xfer(self, ep, buf, length, callback, user=0):
        t = self.dll.libusb_alloc_transfer(0)
        if not t:
            raise LibusbError("alloc_transfer failed")
        self._cb_keepalive.append(callback)
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint64))[0] = self.handle.value
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint8))[OFF_FLAGS] = 0
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint8))[OFF_EP] = ep & 0xFF
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint8))[OFF_TYPE] = USB_TRANSFER_TYPE_BULK
        struct.pack_into("<I", t, OFF_TIMEOUT, 0)
        struct.pack_into("<i", t, OFF_LENGTH, length)
        struct.pack_into("<Q", t, OFF_USER, user)
        struct.pack_into("<Q", t, OFF_CALLBACK,
                         ctypes.cast(callback, ctypes.c_void_p).value)
        struct.pack_into("<Q", t, OFF_BUFFER, ctypes.addressof(buf))
        return t

    @staticmethod
    def user_of(t):
        return struct.unpack_from("<Q", t, OFF_USER)[0]

    @staticmethod
    def status_of(t):
        return struct.unpack_from("<i", t, OFF_STATUS)[0]

    @staticmethod
    def actual_of(t):
        return struct.unpack_from("<i", t, OFF_ACTUAL)[0]

    def submit(self, t):
        r = self.dll.libusb_submit_transfer(t)
        if r != 0:
            raise LibusbError("submit: " + self.strerror(r))

    def pump(self):
        r = self.dll.libusb_handle_events_timeout(self.ctx, self.tv)
        if r != 0:
            raise LibusbError("handle_events: " + self.strerror(r))

    def clear_halt(self, endpoint):
        return self.dll.libusb_clear_halt(self.handle, endpoint)

    def send_command(self, ep_out, packet):
        if ep_out is None:
            raise LibusbError("device has no bulk OUT endpoint")
        buf = (ctypes.c_ubyte * len(packet)).from_buffer_copy(packet)
        result = []

        @self.cb_type
        def done(t):
            result.append((self.status_of(t), self.actual_of(t)))

        t = self.make_xfer(ep_out, buf, len(packet), done)
        self.submit(t)
        deadline = time.perf_counter() + 1.0
        while not result and time.perf_counter() < deadline:
            self.pump()
        if not result:
            r = self.dll.libusb_cancel_transfer(t)
            if r not in (0, -10):  # LIBUSB_ERROR_NOT_FOUND
                self.orphans.append((t, buf, done))
                raise LibusbError("cancel OUT: " + self.strerror(r))
            while not result and time.perf_counter() < deadline + 1.0:
                self.pump()
        if not result:
            # libusb still owns the transfer until its callback has run.  Never
            # free it here: retaining the allocation is safer than a UAF.
            self.orphans.append((t, buf, done))
            raise LibusbError("OUT cancel callback did not return; transfer retained")
        if result[0][0] == LIBUSB_TRANSFER_STALL:
            r = self.clear_halt(ep_out)
            if r != 0:
                raise LibusbError("clear_halt OUT: " + self.strerror(r))
        self.dll.libusb_free_transfer(t)
        if result[0][0] != LIBUSB_TRANSFER_COMPLETED or result[0][1] != len(packet):
            raise LibusbError("OUT command status=%d actual=%d" % result[0])

    def close(self):
        if self.orphans:
            print("warning: %d transfer(s) still owned by libusb; "
                  "leaving context for process teardown" % len(self.orphans))
            return
        if self.handle:
            self.dll.libusb_release_interface(self.handle, self.ifnum)
            self.dll.libusb_close(self.handle)
            self.handle = None
        if self.ctx:
            self.dll.libusb_exit(self.ctx)
            self.ctx = None


# ---------------------------------------------------------------------------
# PC-side raw-slot conversion.  NumPy is used when available; the fallback is
# intentionally simple and is meant for offline captures, not 144 Hz preview.
# ---------------------------------------------------------------------------
def unpack_frame(raw, fmt):
    """Decode on the PC; fmt 5 keeps every 10-bit sample in u16 containers."""
    if fmt not in FMT:
        raise ValueError("unknown pixel format %r" % (fmt,))
    line_bytes = PACKED422_LINE_BYTES if fmt == FMT_PACKED42210 else RAW_LINE_BYTES
    expected = line_bytes * CROP_H
    if len(raw) != expected:
        raise ValueError("raw frame has %d bytes, expected %d" % (len(raw), expected))
    try:
        import numpy as np
    except ImportError:
        np = None
    if fmt == FMT_PACKED42210:
        if np is not None:
            pairs = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 2, 5)
            packed = np.zeros(pairs.shape[:2], dtype=np.uint64)
            for i in range(5):
                packed |= pairs[:, :, i].astype(np.uint64) << np.uint64(8 * i)
            out = np.empty(packed.shape + (4,), dtype="<u2")
            out[:, :, 0] = packed & 0x3ff
            out[:, :, 1] = (packed >> 10) & 0x3ff
            out[:, :, 2] = (packed >> 30) & 0x3ff
            out[:, :, 3] = (packed >> 20) & 0x3ff
            return out.tobytes()
        out = bytearray()
        for p in range(0, len(raw), 5):
            word = int.from_bytes(raw[p:p + 5], "little")
            out += struct.pack("<4H", word & 0x3ff, (word >> 10) & 0x3ff,
                               (word >> 30) & 0x3ff, (word >> 20) & 0x3ff)
        return bytes(out)
    if np is not None:
        slots = np.frombuffer(raw, dtype="<u4").astype(np.uint32) & 0x3fffffff
        if fmt == 0:
            pairs = slots.reshape(-1, 2)
            out = np.empty((pairs.shape[0], 4), dtype=np.uint8)
            out[:, 0] = ((pairs[:, 0] >> 20) & 0x3ff) >> 2  # A_R = Cb
            out[:, 1] = ((pairs[:, 0] >> 10) & 0x3ff) >> 2  # A_G = Y0
            out[:, 2] = ((pairs[:, 1] >> 20) & 0x3ff) >> 2  # B_R = Cr
            out[:, 3] = ((pairs[:, 1] >> 10) & 0x3ff) >> 2  # B_G = Y1
            return out.tobytes()
        r_cr = (slots >> 20) & 0x3ff
        g_y = (slots >> 10) & 0x3ff
        b_cb = slots & 0x3ff
        if fmt in (1, 3):
            channels = (r_cr, g_y, b_cb)       # R,G,B
        else:
            channels = (g_y, b_cb, r_cr)       # Y,Cb,Cr
        if fmt in (1, 2):
            out = np.zeros((slots.size, 4), dtype=np.uint8)
            for i, channel in enumerate(channels):
                out[:, i] = channel >> 2
            return out.tobytes()
        packed = channels[0] | (channels[1] << 10) | (channels[2] << 20)
        return packed.astype("<u4").tobytes()

    slots = [word & 0x3fffffff for (word,) in struct.iter_unpack("<I", raw)]
    if fmt == 0:
        out = bytearray()
        for a, b in zip(slots[0::2], slots[1::2]):
            out.extend((((a >> 20) & 0x3ff) >> 2, ((a >> 10) & 0x3ff) >> 2,
                        ((b >> 20) & 0x3ff) >> 2, ((b >> 10) & 0x3ff) >> 2))
        return bytes(out)
    from array import array
    dtype = "B" if fmt in (1, 2) else "I"
    out = array(dtype)
    for slot in slots:
        r_cr, g_y, b_cb = (slot >> 20) & 0x3ff, (slot >> 10) & 0x3ff, slot & 0x3ff
        channels = (r_cr, g_y, b_cb) if fmt in (1, 3) else (g_y, b_cb, r_cr)
        if fmt in (1, 2):
            channels = tuple(channel >> 2 for channel in channels)
            out.extend(channels + (0,))
        else:
            out.append(channels[0] | (channels[1] << 10) | (channels[2] << 20))
    if sys.byteorder != "little" and dtype == "I":
        out.byteswap()
    return out.tobytes()


# ---------------------------------------------------------------------------
# Byte-stream parser and frame reassembly
# ---------------------------------------------------------------------------
class Receiver:
    def __init__(self):
        self.pending = bytearray()
        self.frame = None
        self.last_frame = None
        self.last_fmt = None
        self.cur_frame = None
        self.cur_fmt = None
        self.line_seen = bytearray(CROP_H)
        self.expected_line = 0
        self.frame_bad = False
        self.frames_done = 0
        self.frames_dropped = 0
        self.lines = 0
        self.overruns = 0
        self.gaps = 0
        self.resyncs = 0
        self.status_word = None
        self.drop_count = 0
        self.line_hist = deque(maxlen=4096)
        self.frame_hist = deque(maxlen=64)
        self.byte_total = 0
        self.interval_frames = 0
        self.out = None
        self.stop = False

    def setup(self, args, mps=1024):
        self.args = args
        self.buf_bytes = max(args.urb_lines * (HDR_BYTES + CROP_W * 4), mps * 4)
        if args.save:
            self.out = open(args.save, "wb")

    @staticmethod
    def payload_bytes(fmt):
        return PACKED422_LINE_BYTES if fmt == FMT_PACKED42210 else RAW_LINE_BYTES

    def _new_frame(self, frame_idx, fmt):
        if self.frame is not None and self.cur_frame != frame_idx:
            self.frames_dropped += 1
        self.cur_frame, self.cur_fmt = frame_idx, fmt
        self.frame = bytearray(self.payload_bytes(fmt) * CROP_H)
        self.line_seen = bytearray(CROP_H)
        self.expected_line = 0
        self.frame_bad = False

    def _line(self, frame_idx, line_idx, flags, fmt, payload, t_now):
        if self.cur_frame != frame_idx or self.cur_fmt != fmt or self.frame is None:
            self._new_frame(frame_idx, fmt)
        if line_idx != self.expected_line:
            self.gaps += 1
            self.frame_bad = True
        self.expected_line = line_idx + 1
        if flags & FLAG_OVERRUN:
            self.overruns += 1
            self.frame_bad = True
        if self.line_seen[line_idx]:
            self.frame_bad = True
        self.line_seen[line_idx] = 1
        dst = line_idx * self.payload_bytes(fmt)
        self.frame[dst:dst + len(payload)] = payload
        self.lines += 1
        self.line_hist.append((t_now, frame_idx, line_idx))
        if flags & FLAG_EOLF:
            complete = (line_idx == CROP_H - 1 and not self.frame_bad
                        and all(self.line_seen))
            if complete:
                self.frames_done += 1
                self.interval_frames += 1
                self.frame_hist.append((t_now, frame_idx))
                self.last_frame = unpack_frame(self.frame, self.cur_fmt)
                self.last_fmt = self.cur_fmt
                bpp = FMT[self.cur_fmt][1]
                if self.out:
                    self.out.write(struct.pack("<4sBBHHI2x", b"BFRA", self.cur_fmt, bpp,
                                               CROP_W, CROP_H, frame_idx))
                    self.out.write(self.last_frame)
                if self.frames_done <= self.args.dump_frames:
                    self.dump_frame()
            else:
                self.frames_dropped += 1
            self.frame = None
            self.cur_frame = None

    def feed(self, data, t_now):
        self.pending.extend(data)
        self.byte_total += len(data)
        off = 0
        mv = memoryview(self.pending)
        while len(mv) - off >= HDR_BYTES:
            frame_idx, w1, w2, w3 = struct.unpack_from("<IIII", mv, off)
            line_idx = w1 & 0x3FF
            flags, typ, fmt = w2 & 0xFF, (w2 >> 5) & 7, (w2 >> 2) & 7
            valid_line = (typ == TYPE_LINE and (w1 & 0xFFFFFC00) == 0
                          and line_idx < CROP_H
                          and bool(flags & FLAG_EOLF) == (line_idx == CROP_H - 1))
            valid_status = (typ == TYPE_STATUS and w1 == 0x0000FFFF
                            and (flags & (FLAG_EOLF | FLAG_OVERRUN)) == FLAG_EOLF)
            valid = (bytes(mv[off + 9:off + 12]) == SYNC and fmt in FMT
                     and (valid_status or valid_line))
            if not valid:
                self.resyncs += 1
                off += 1
                continue
            pay_len = 0 if typ == TYPE_STATUS else self.payload_bytes(fmt)
            pkt_len = HDR_BYTES + pay_len
            if len(mv) - off < pkt_len:
                break
            self.drop_count = w3 & 0xFFFF
            if typ == TYPE_STATUS:
                self.status_word = (w3 >> 16) & 0xFFFF
            else:
                self._line(frame_idx, line_idx, flags, fmt,
                           mv[off + HDR_BYTES:off + pkt_len], t_now)
            off += pkt_len
        mv.release()
        if off:
            del self.pending[:off]

    def dump_frame(self):
        ext = FMT[self.last_fmt][2]
        fn = "frame_%08d.%s" % (self.cur_frame, ext)
        with open(fn, "wb") as f:
            f.write(self.last_frame)
        print("saved", fn, FMT[self.last_fmt][0])

    def reset_interval(self):
        self.byte_total = 0
        self.lines = 0
        self.interval_frames = 0

    def print_stats(self, t_prev, t_now):
        dt = max(t_now - t_prev, 1e-9)
        intervals = []
        h = self.line_hist
        for i in range(1, min(len(h), 256)):
            t1, f1, l1 = h[-i]
            t2, f2, l2 = h[-i - 1]
            if f1 == f2 and l1 == l2 + 1:
                intervals.append(t1 - t2)
        intervals.sort()
        n = len(intervals)
        med = intervals[n // 2] * 1e6 if n else 0.0
        p95 = intervals[min(n - 1, int(n * 0.95))] * 1e6 if n else 0.0
        fmt_name = FMT[self.cur_fmt][0] if self.cur_fmt in FMT else "between-frames"
        print("[stats] frames=%d dropped=%d fps=%.1f lines/s=%.0f Mbit/s=%.1f "
              "fmt=%s line-delta med=%.0fus p95=%.0fus overruns=%d gaps=%d resyncs=%d" %
              (self.frames_done, self.frames_dropped, self.interval_frames / dt,
               self.lines / dt, self.byte_total * 8 / dt / 1e6, fmt_name,
               med, p95, self.overruns, self.gaps, self.resyncs))
        print("        fpga_dropped_lines=%d" % self.drop_count)
        if self.status_word is not None:
            st = self.status_word
            print("        device=0x%04x init_err=%d i2c_nack=%d stream_ok=%d "
                  "htot_ok=%d stable=%d init_done=%d hpd=%d online=%d i2c_fail=%d busy=%d" %
                  (st, st >> 12, (st >> 8) & 15, (st >> 7) & 1, (st >> 6) & 1,
                   (st >> 5) & 1, (st >> 4) & 1, (st >> 3) & 1, (st >> 2) & 1,
                   (st >> 1) & 1, st & 1))
        if len(self.frame_hist) >= 2:
            (ta, fa), (tb, fb) = self.frame_hist[-2], self.frame_hist[-1]
            print("        frame period=%.3f ms (%d -> %d)" %
                  ((tb - ta) * 1e3, fa, fb))

    def close(self):
        if self.out:
            self.out.close()
            self.out = None


def run_libusb_async(args, rx):
    usb = LibusbAsync()
    transfers = []
    try:
        ep_in, ep_out, mps = usb.open_device(args.vid, args.pid)
        rx.setup(args, mps)
        print("opened raw libusb: IN=0x%02x OUT=%s MPS=%d URBs=%d x %d B" %
              (ep_in, "0x%02x" % ep_out if ep_out is not None else "none",
               mps, args.urbs, rx.buf_bytes))
        bufs = [(ctypes.c_ubyte * rx.buf_bytes)() for _ in range(args.urbs)]
        errors = []
        active = set()

        @usb.cb_type
        def on_done(t):
            idx = usb.user_of(t)
            active.discard(idx)  # libusb no longer owns it until resubmission
            st, actual = usb.status_of(t), usb.actual_of(t)
            if st == LIBUSB_TRANSFER_STALL and not rx.stop:
                r = usb.clear_halt(ep_in)
                if r == 0:
                    try:
                        usb.submit(t)
                        active.add(idx)
                        return
                    except LibusbError as exc:
                        errors.append(str(exc))
                else:
                    errors.append("clear_halt IN: " + usb.strerror(r))
                rx.stop = True
            elif st == LIBUSB_TRANSFER_COMPLETED and actual:
                rx.feed(memoryview(bufs[idx])[:actual], time.perf_counter())
            elif st != LIBUSB_TRANSFER_COMPLETED and not rx.stop:
                errors.append(st)
                rx.stop = True
            if not rx.stop:
                try:
                    usb.submit(t)
                    active.add(idx)
                except LibusbError as exc:
                    errors.append(str(exc))
                    rx.stop = True

        for i, buf in enumerate(bufs):
            t = usb.make_xfer(ep_in, buf, len(buf), on_done, i)
            transfers.append(t)
            usb.submit(t)
            active.add(i)
        t_stats = time.perf_counter()
        while not rx.stop:
            usb.pump()
            now = time.perf_counter()
            if now - t_stats >= args.stats_every:
                rx.print_stats(t_stats, now)
                rx.reset_interval()
                t_stats = now
            if args.frames and rx.frames_done >= args.frames:
                break
            if errors:
                raise LibusbError("IN transfer: %s" % errors[-1])
    finally:
        rx.stop = True
        for i, t in enumerate(transfers):
            if i in active:
                usb.dll.libusb_cancel_transfer(t)
        deadline = time.perf_counter() + 1.0
        while active and time.perf_counter() < deadline:
            try:
                usb.pump()
            except LibusbError:
                break
        for i, t in enumerate(transfers):
            if i in active:
                # A callback may still touch the buffer and transfer.  Retain
                # both instead of calling libusb_free_transfer prematurely.
                usb.orphans.append((t, bufs, on_done))
            else:
                usb.dll.libusb_free_transfer(t)
        usb.close()
        rx.close()
    return 0


def run_pyusb(args, rx):
    import usb.core
    import usb.util
    dev = usb.core.find(idVendor=args.vid, idProduct=args.pid)
    if dev is None:
        raise LibusbError("device not found via pyusb")
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)
    dev.set_configuration()
    usb.util.claim_interface(dev, 0)
    ep_in = ep_out = None
    for ep in dev.get_active_configuration()[(0, 0)]:
        if (ep.bmAttributes & 3) == 2:
            if ep.bEndpointAddress & 0x80:
                ep_in = ep.bEndpointAddress
            else:
                ep_out = ep.bEndpointAddress
    if ep_in is None:
        raise LibusbError("no bulk IN endpoint")
    rx.setup(args)
    print("opened pyusb: IN=0x%02x OUT=%s" %
          (ep_in, "0x%02x" % ep_out if ep_out is not None else "none"))
    q = deque(maxlen=128)
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            try:
                data = bytes(dev.read(ep_in, rx.buf_bytes, timeout=500))
                if q.full():
                    q.popleft()
                q.append((data, time.perf_counter()))
            except Exception as exc:
                if not stop.is_set():
                    q.append((exc, time.perf_counter()))
                    return

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    t_stats = time.perf_counter()
    try:
        while not rx.stop:
            if not q:
                time.sleep(0.0002)
                continue
            data, timestamp = q.popleft()
            if isinstance(data, Exception):
                raise data
            rx.feed(data, timestamp)
            now = time.perf_counter()
            if now - t_stats >= args.stats_every:
                rx.print_stats(t_stats, now)
                rx.reset_interval()
                t_stats = now
            if args.frames and rx.frames_done >= args.frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=1.0)
        usb.util.dispose_resources(dev)
        rx.close()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--vid", type=lambda x: int(x, 0), default=DEF_VID)
    ap.add_argument("--pid", type=lambda x: int(x, 0), default=DEF_PID)
    ap.add_argument("--frames", type=int, default=0, help="stop after N complete frames")
    ap.add_argument("--urbs", type=int, default=8)
    ap.add_argument("--urb-lines", type=int, default=4,
                    help="maximum line payload buffered per URB; short packets may finish earlier")
    ap.add_argument("--stats-every", type=float, default=1.0)
    ap.add_argument("--save", metavar="FILE",
                    help="write BFRA-header + raw payload for each complete frame")
    ap.add_argument("--dump-frames", type=int, default=0)
    ap.add_argument("--pyusb", action="store_true")
    args = ap.parse_args()
    if args.urbs < 2 or args.urb_lines < 1:
        ap.error("--urbs must be >=2 and --urb-lines >=1")
    try:
        if args.pyusb:
            rc = run_pyusb(args, Receiver())
        else:
            try:
                rc = run_libusb_async(args, Receiver())
            except LibusbError as exc:
                print("raw libusb unavailable:", exc)
                print("trying pyusb fallback...")
                rc = run_pyusb(args, Receiver())
    except (LibusbError, OSError) as exc:
        print("receiver error:", exc)
        rc = 2
    except KeyboardInterrupt:
        rc = 0
    sys.exit(rc)


if __name__ == "__main__":
    main()
