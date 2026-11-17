#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
linux_rx.py -- libusb-1.0 async receiver for the GW5A-25 / IT68051 / FT601Q
HDMI capture card.

Stream protocol (device -> host, single SuperSpeed Bulk IN endpoint)
--------------------------------------------------------------------
The device sends a continuous byte stream of 1296-byte packets (one per
captured 640-px line, plus one STATUS packet per frame):

  word0  u32 frame_idx        little endian on the wire
  word1  u16 line_idx         (0xFFFF = STATUS packet), u16 rsv=0
  word2  u8  flags            bit0=EOLF  bit1=OVERRUN  bits[7:5]=type
                              (type 0 = LINE, 1 = STATUS)
         u8  rsv=0, u16 rsv=0
  word3  u16 status           device status word, u16 rsv=0
  payload 1280 B              320 x UYVY pixel pairs (Cb,Y0,Cr,Y1) = 640 px

STATUS packets arrive once per frame after the EOLF line (zero payload).
The FT601 forms USB packets itself (MPS=1024, burst 15): each line written as
one continuous FIFO transaction appears as a 1024 B + a 272 B short packet.
The parser only relies on the byte stream, never on packet boundaries.

Host -> device commands (needs EN_CMD_PATH=1 in the FPGA)
---------------------------------------------------------
16-byte packets written to the interface's Bulk OUT endpoint:
  'BFCP' | cmd(1) | len=8 | x0(u32) | y0(u32) | rsv(2)
  cmd: 1 = SET_CROP (x0 must be a multiple of 4), 2 = SOFT_RESET,
       3 = GET_STATUS (passive: a STATUS packet arrives every frame anyway)

Implementation
--------------
Primary transport: raw libusb-1.0 through ctypes with N queued asynchronous
URBs (true async, matches the "multi-URB queue" requirement; struct layout
per libusb.h on LP64).  Fallback: pyusb with a reader thread (blocking reads)
when raw ctypes cannot claim the device.  Frame reassembly copies payloads
with single slice assignments (no per-byte Python in the hot path).

Latency statistics
------------------
  * line-arrival delta (med / p95) between consecutive lines of a frame -
    the jitter of the pipeline; its floor is the source line period.
  * frame period between EOLF arrivals.
  * OVERRUN flags, line-index gaps (dropped lines), type errors.
True source-to-host latency needs a source-side timestamp; see README.

Usage:
  python3 linux_rx.py [--vid 0x0403] [--pid 0x601f] [--frames N]
      [--urbs 8] [--urb-lines 16] [--stats-every 1.0]
      [--set-crop X Y] [--save out.raw] [--dump-frames N]
"""

import argparse
import ctypes
import ctypes.util
import struct
import sys
import threading
import time
from collections import deque

# ----------------------------------------------------------------- constants
PKT_BYTES = 1296                  # 16 header + 1280 payload
HDR_BYTES = 16
PAY_BYTES = 1280
CROP_W, CROP_H = 640, 640
TYPE_LINE, TYPE_STATUS = 0, 1
FLAG_EOLF, FLAG_OVERRUN = 0x01, 0x02
DEF_VID, DEF_PID = 0x0403, 0x601F     # FTDI FT601 defaults (adjust if needed)


# =============================================================================
# raw libusb-1.0 via ctypes, asynchronous multi-URB
# =============================================================================
LIBUSB_SUCCESS = 0
LIBUSB_TRANSFER_COMPLETED = 0
LIBUSB_TRANSFER_CANCELLED = 3
LIBUSB_CONTEXT = ctypes.c_void_p
LIBUSB_DEVICE_HANDLE = ctypes.c_void_p
LIBUSB_TRANSFER_PTR = ctypes.c_void_p

# struct libusb_transfer (LP64):
#   0  dev_handle (8)   8 flags(1) 9 endpoint(1) 10 type(1) [11 pad]
#   12 timeout(4)  16 status(4)  20 length(4)  24 actual_length(4)
#   32 callback(8) 40 user_data(8) 48 buffer(8) 56 num_iso_packets(4)
OFF_FLAGS, OFF_EP, OFF_TYPE, OFF_TIMEOUT = 8, 9, 10, 12
OFF_STATUS, OFF_LENGTH, OFF_ACTUAL = 16, 20, 24
OFF_CALLBACK, OFF_USER, OFF_BUFFER = 32, 40, 48

USB_TRANSFER_TYPE_BULK = 3


class LibusbError(Exception):
    pass


class LibusbAsync:
    """Minimal libusb-1.0 binding: one interface, N async bulk IN URBs."""

    def __init__(self):
        path = ctypes.util.find_library("usb-1.0")
        names = [path] if path else []
        names += ["libusb-1.0.so.0", "libusb-1.0.so", "libusb-1.0.dylib",
                  "libusb-1.0.dll"]
        self.dll = None
        for n in names:
            try:
                self.dll = ctypes.CDLL(n)
                break
            except OSError:
                continue
        if self.dll is None:
            raise LibusbError("libusb-1.0 not found; apt install libusb-1.0-0 "
                              "or pip install pyusb (fallback path)")
        d = self.dll
        d.libusb_init.argtypes = [ctypes.POINTER(LIBUSB_CONTEXT)]
        d.libusb_exit.argtypes = [LIBUSB_CONTEXT]
        d.libusb_get_device_list.argtypes = [LIBUSB_CONTEXT,
                                             ctypes.POINTER(ctypes.c_void_p)]
        d.libusb_free_device_list.argtypes = [ctypes.c_void_p, ctypes.c_int]
        d.libusb_get_device_descriptor.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        d.libusb_open.argtypes = [ctypes.c_void_p,
                                  ctypes.POINTER(LIBUSB_DEVICE_HANDLE)]
        d.libusb_close.argtypes = [LIBUSB_DEVICE_HANDLE]
        d.libusb_set_configuration.argtypes = [LIBUSB_DEVICE_HANDLE, ctypes.c_int]
        d.libusb_claim_interface.argtypes = [LIBUSB_DEVICE_HANDLE, ctypes.c_int]
        d.libusb_set_auto_detach_kernel_driver.argtypes = [LIBUSB_DEVICE_HANDLE,
                                                           ctypes.c_int]
        d.libusb_alloc_transfer.argtypes = [ctypes.c_int]
        d.libusb_alloc_transfer.restype = LIBUSB_TRANSFER_PTR
        d.libusb_free_transfer.argtypes = [LIBUSB_TRANSFER_PTR]
        d.libusb_submit_transfer.argtypes = [LIBUSB_TRANSFER_PTR]
        d.libusb_submit_transfer.restype = ctypes.c_int
        d.libusb_cancel_transfer.argtypes = [LIBUSB_TRANSFER_PTR]
        d.libusb_handle_events_timeout.argtypes = [LIBUSB_CONTEXT,
                                                   ctypes.c_void_p]
        d.libusb_get_max_packet_size.argtypes = [ctypes.c_void_p, ctypes.c_ubyte]
        d.libusb_get_max_packet_size.restype = ctypes.c_int
        d.libusb_error_name.argtypes = [ctypes.c_int]
        d.libusb_error_name.restype = ctypes.c_char_p

        self.ctx = LIBUSB_CONTEXT()
        if d.libusb_init(ctypes.byref(self.ctx)) != 0:
            raise LibusbError("libusb_init failed")
        self.tv = (ctypes.c_long * 2)(1, 0)      # 1s timeval for handle_events
        self.cb_type = ctypes.CFUNCTYPE(None, LIBUSB_TRANSFER_PTR)
        self._cb_keepalive = []                  # prevent GC of live callbacks

    def strerror(self, code):
        name = self.dll.libusb_error_name(code)
        return (name.decode() if name else "err%d" % code)

    def open_device(self, vid, pid, ifnum=0):
        d = self.dll
        lst = ctypes.c_void_p()
        n = d.libusb_get_device_list(self.ctx, ctypes.byref(lst))
        dev = None
        for i in range(n):
            dp = ctypes.c_void_p(lst.value + i * ctypes.sizeof(ctypes.c_void_p))
            dd = (ctypes.c_ubyte * 18)()
            if d.libusb_get_device_descriptor(dp, dd) != 0:
                continue
            if (dd[8] | (dd[9] << 8)) == vid and (dd[10] | (dd[11] << 8)) == pid:
                dev = dp
                break
        if dev is None:
            d.libusb_free_device_list(lst, 1)
            raise LibusbError("device %04x:%04x not found" % (vid, pid))
        h = LIBUSB_DEVICE_HANDLE()
        if d.libusb_open(dev, ctypes.byref(h)) != 0:
            d.libusb_free_device_list(lst, 1)
            raise LibusbError("libusb_open failed (udev permission? see README)")
        d.libusb_free_device_list(lst, 1)
        d.libusb_set_auto_detach_kernel_driver(h, 1)
        r = d.libusb_set_configuration(h, 1)
        if r != 0:
            print("note: set_configuration(1) ->", self.strerror(r))
        r = d.libusb_claim_interface(h, ifnum)
        if r != 0:
            raise LibusbError("claim_interface(%d): %s (try sudo / udev rule)"
                              % (ifnum, self.strerror(r)))
        self.handle = h
        return h

    def make_xfer(self, ep, buf, length, callback, user=0):
        t = self.dll.libusb_alloc_transfer(0)
        if not t:
            raise LibusbError("alloc_transfer failed")
        self._cb_keepalive.append(callback)
        # dev_handle field holds the libusb_device_handle pointer value
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint64))[0] = self.handle.value
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint8))[OFF_FLAGS] = 0
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint8))[OFF_EP] = ep & 0xFF
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint8))[OFF_TYPE] = USB_TRANSFER_TYPE_BULK
        struct.pack_into("<I", t, OFF_TIMEOUT, 0)
        struct.pack_into("<i", t, OFF_LENGTH, length)
        struct.pack_into("<Q", t, OFF_USER, user)
        cb_addr = ctypes.cast(self.cb_type(callback), ctypes.c_void_p).value
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint64))[OFF_CALLBACK // 8] = cb_addr
        ctypes.cast(t, ctypes.POINTER(ctypes.c_uint64))[OFF_BUFFER // 8] = \
            ctypes.addressof(buf)
        return t

    def user_of(self, t):
        return struct.unpack_from("<Q", t, OFF_USER)[0]

    def submit(self, t):
        r = self.dll.libusb_submit_transfer(t)
        if r != 0:
            raise LibusbError("submit: " + self.strerror(r))

    def cancel(self, t):
        self.dll.libusb_cancel_transfer(t)

    def pump(self):
        self.dll.libusb_handle_events_timeout(self.ctx, self.tv)

    def status_of(self, t):
        return struct.unpack_from("<i", t, OFF_STATUS)[0]

    def actual_of(self, t):
        return struct.unpack_from("<i", t, OFF_ACTUAL)[0]

    def close(self):
        self.dll.libusb_exit(self.ctx)


# =============================================================================
# receiver: protocol parse, frame reassembly, latency statistics
# =============================================================================
class Receiver:
    def __init__(self):
        self.frame = None
        self.cur_frame = -1
        self.frames_done = 0
        self.lines = 0
        self.overruns = 0
        self.gaps = 0
        self.type_errs = 0
        self.status_word = None
        self.status_rx_t = None
        self.synced = False
        self.last_line = -1
        self.need_hdr = True
        self.hdr = None
        self.need_pay = 0
        self.line_hist = deque(maxlen=4096)   # (t, frame, line)
        self.frame_hist = deque(maxlen=64)    # (t, frame) at EOLF
        self.byte_total = 0
        self.out = None
        self.stop = False

    # ------------------------------------------------------------ transport
    def setup(self, args, mps):
        self.args = args
        self.mps = mps
        self.buf_bytes = args.urb_lines * PKT_BYTES
        self.n_urbs = args.urbs
        if args.save:
            self.out = open(args.save, "wb")

    def send_set_crop(self, ep_out_write, x, y):
        pkt = b"BFCP" + bytes([1, 8]) + struct.pack("<II", x & ~3, y) + b"\0\0"
        ep_out_write(pkt)
        print("SET_CROP x0=%d y0=%d sent (takes effect at next VSYNC)"
              % (x & ~3, y))

    # ------------------------------------------------------------ parsing
    def feed(self, data, t_now):
        """data: bytes/memoryview of one URB's received bytes."""
        n = len(data)
        self.byte_total += n
        off = 0
        mv = memoryview(data)
        while off < n:
            if self.need_hdr:
                if n - off < HDR_BYTES:
                    break                       # short tail; device stream is
                                                # 1296-aligned per packet set,
                                                # tolerate and wait
                w0, w1, w2, w3 = struct.unpack_from("<IIII", mv, off)
                frame_idx = w0
                line_idx = w1 & 0xFFFF
                flags = w2 & 0xFF
                typ = (flags >> 5) & 7
                status = (w3 >> 16) & 0xFFFF
                if typ == TYPE_LINE:
                    if self.synced:
                        exp = 0 if self.last_line == CROP_H - 1 else self.last_line + 1
                        if line_idx != exp:
                            self.gaps += 1      # dropped line(s) or resync
                    self.synced = True
                    self.last_line = line_idx
                    self.cur_frame = frame_idx
                    if self.frame is None:
                        self.frame = bytearray(CROP_W * CROP_H * 2)
                    if flags & FLAG_OVERRUN:
                        self.overruns += 1
                    self.hdr = (frame_idx, line_idx, flags)
                    self.need_pay = PAY_BYTES
                    self.line_hist.append((t_now, frame_idx, line_idx))
                elif typ == TYPE_STATUS:
                    self.status_word = status
                    self.status_rx_t = t_now
                    self.need_pay = PAY_BYTES
                    self.hdr = None             # status payload is discarded
                else:
                    self.type_errs += 1
                    self.need_pay = PAY_BYTES   # resync by skipping payload
                    self.hdr = None
                self.need_hdr = False
                off += HDR_BYTES
            else:
                take = min(self.need_pay, n - off)
                if self.hdr is not None and self.frame is not None:
                    _, line_idx, _ = self.hdr
                    if line_idx < CROP_H:
                        # continuation writes must resume where the previous
                        # segment stopped, not restart the row: the device
                        # stream is NOT guaranteed packet-aligned per URB
                        # (272B short packets split every 1296B line), so a
                        # payload regularly spans several feed() calls.
                        dst = line_idx * CROP_W * 2 + (PAY_BYTES
                                                       - self.need_pay)
                        self.frame[dst:dst + take] = mv[off:off + take]
                self.need_pay -= take
                off += take
                if self.need_pay == 0:
                    if self.hdr is not None:
                        _, line_idx, flags = self.hdr
                        self.lines += 1
                        self.hdr = None
                        if flags & FLAG_EOLF:
                            self.frames_done += 1
                            self.frame_hist.append((t_now, self.cur_frame))
                            if self.out:
                                self.out.write(self.frame)
                            if self.frames_done <= self.args.dump_frames:
                                self.dump_frame()
                    self.need_hdr = True

    def dump_frame(self):
        fn = "frame_%08d.uyvy" % self.cur_frame
        try:
            with open(fn, "wb") as f:
                f.write(self.frame)
            print("saved", fn)
        except OSError as e:
            print("frame save failed:", e)

    # ------------------------------------------------------------ statistics
    def print_stats(self, t_prev, t_now):
        dt = t_now - t_prev
        fps = (self.frames_done) / dt if dt > 0 else 0.0
        klines = self.lines / dt / 1000.0 if dt > 0 else 0.0
        mbit = self.byte_total * 8 / dt / 1e6 if dt > 0 else 0.0
        intervals = []
        h = self.line_hist
        for i in range(1, min(len(h), 256)):
            t1, f1, l1 = h[-i]
            t2, f2, l2 = h[-i - 1]
            if f1 == f2 and ((l1 == l2 + 1) or (l1 == 0 and l2 == CROP_H - 1)):
                intervals.append(t1 - t2)
        intervals.sort()
        n = len(intervals)
        med = intervals[n // 2] * 1e6 if n else 0.0
        p95 = intervals[min(n - 1, int(n * 0.95))] * 1e6 if n else 0.0
        st = self.status_word
        print("[stats] frames=%d fps=%.1f lines=%d (%.1f kl/s, %.1f Mbit/s) "
              "line-delta med=%.0fus p95=%.0fus overruns=%d gaps=%d "
              "type_errs=%d" %
              (self.frames_done, fps, self.lines, klines, mbit,
               med, p95, self.overruns, self.gaps, self.type_errs))
        if st is not None:
            print("        device status=0x%04x: init_err=%d i2c_nack=%d "
                  "stream_ok=%d htot_ok=%d stable=%d init_done=%d hpd=%d "
                  "online=%d i2c_fail=%d replay=%d" %
                  (st, (st >> 12) & 0xF, (st >> 8) & 0xF, (st >> 7) & 1,
                   (st >> 6) & 1, (st >> 5) & 1, (st >> 4) & 1,
                   (st >> 3) & 1, (st >> 2) & 1, (st >> 1) & 1, st & 1))
        if len(self.frame_hist) >= 2:
            (ta, fa), (tb, fb) = self.frame_hist[-2], self.frame_hist[-1]
            print("        frame period=%.3f ms (frame %d -> %d)"
                  % ((tb - ta) * 1e3, fa, fb))


# =============================================================================
# transport 1: raw libusb async (primary)
# =============================================================================
def run_libusb_async(args, rx):
    usb = LibusbAsync()
    rx.setup(args, 1024)
    usb.open_device(args.vid, args.pid, 0)
    print("opened %04x:%04x via raw libusb-1.0, async URB queue" %
          (args.vid, args.pid))

    bufs = [(ctypes.c_ubyte * rx.buf_bytes)() for _ in range(rx.n_urbs)]
    transfers = []
    errors = []

    @ctypes.CFUNCTYPE(None, LIBUSB_TRANSFER_PTR)
    def on_done(t):
        st = usb.status_of(t)
        act = usb.actual_of(t)
        idx = usb.user_of(t)
        if st == LIBUSB_TRANSFER_COMPLETED and act > 0:
            rx.feed(memoryview(bufs[idx])[:act], time.perf_counter())
        elif st != LIBUSB_TRANSFER_COMPLETED:
            errors.append(st)
            rx.stop = True
        if not rx.stop:
            usb.submit(t)

    # command sink (bulk OUT, ep 0x01) - only if EN_CMD_PATH=1 in the FPGA
    def cmd_sink(pkt):
        # blocking one-shot write via a temporary transfer
        done = threading.Event()
        b = (ctypes.c_ubyte * len(pkt)).from_buffer_copy(pkt)

        @ctypes.CFUNCTYPE(None, LIBUSB_TRANSFER_PTR)
        def _cb(t):
            done.set()

        t = usb.make_xfer(0x01, b, len(pkt), _cb)
        usb.submit(t)
        done.wait(timeout=1.0)
        if not done.is_set():
            usb.cancel(t)

    if args.set_crop:
        try:
            rx.send_set_crop(cmd_sink, args.set_crop[0], args.set_crop[1])
        except Exception as e:
            print("SET_CROP failed (EN_CMD_PATH=0?):", e)

    for t in transfers:
        usb.submit(t)
    print("URBs=%d x %d lines (%.1f KiB) queued" %
          (rx.n_urbs, args.urb_lines, rx.n_urbs * rx.buf_bytes / 1024))

    t_stats = time.perf_counter()
    try:
        while not rx.stop:
            usb.pump()
            now = time.perf_counter()
            if now - t_stats >= args.stats_every:
                rx.print_stats(t_stats, now)
                t_stats = now
                rx.byte_total = 0
                rx.lines = 0
                rx.frames_done = 0
            if args.frames and rx.frames_done >= args.frames:
                break
            if errors:
                print("transfer error status:", usb.strerror(errors[-1]))
                break
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop = True
        for t in transfers:
            usb.cancel(t)
        for _ in range(20):
            usb.pump()
        usb.close()
    return 0


# =============================================================================
# transport 2: pyusb fallback (blocking reader thread)
# =============================================================================
def run_pyusb(args, rx):
    import usb.core
    import usb.util
    dev = usb.core.find(idVendor=args.vid, idProduct=args.pid)
    if dev is None:
        print("device %04x:%04x not found via pyusb" % (args.vid, args.pid))
        return 2
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)
    try:
        dev.set_configuration()
    except Exception:
        pass
    usb.util.claim_interface(dev, 0)
    cfg = dev.get_active_configuration()
    intf = cfg[(0, 0)]
    ep_in = ep_out = None
    for e in intf:
        if (e.bmAttributes & 3) != 2:
            continue
        if e.bEndpointAddress & 0x80:
            ep_in = e.bEndpointAddress
        else:
            ep_out = e.bEndpointAddress
    print("opened via pyusb: bulk IN 0x%02x (MPS assumed 1024)" % ep_in)
    rx.setup(args, 1024)

    if ep_out is not None and args.set_crop:
        pkt = b"BFCP" + bytes([1, 8]) + struct.pack("<II",
              args.set_crop[0] & ~3, args.set_crop[1]) + b"\0\0"
        try:
            dev.write(ep_out, pkt, 1000)
            print("SET_CROP sent")
        except Exception as e:
            print("SET_CROP failed (EN_CMD_PATH=0?):", e)

    q = deque(maxlen=128)
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            try:
                d = dev.read(ep_in, rx.buf_bytes, timeout=500)
                if q.full():
                    q.popleft()                # never stall on a slow parser
                q.append((bytes(d), time.perf_counter()))
            except Exception as e:
                q.append((e, time.perf_counter()))

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    t_stats = time.perf_counter()
    try:
        while not rx.stop:
            if not q:
                time.sleep(0.0002)
                continue
            d, t_now = q.popleft()
            if isinstance(d, Exception):
                raise d
            rx.feed(memoryview(d), t_now)
            now = time.perf_counter()
            if now - t_stats >= args.stats_every:
                rx.print_stats(t_stats, now)
                t_stats = now
                rx.byte_total = 0
                rx.lines = 0
                rx.frames_done = 0
            if args.frames and rx.frames_done >= args.frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        usb.util.dispose_resources(dev)
    return 0


# ===================================================================== main
def main():
    ap = argparse.ArgumentParser(description="USB HDMI capture receiver "
                                             "(libusb-1.0 async)")
    ap.add_argument("--vid", type=lambda x: int(x, 0), default=DEF_VID)
    ap.add_argument("--pid", type=lambda x: int(x, 0), default=DEF_PID)
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames")
    ap.add_argument("--urbs", type=int, default=8,
                    help="number of queued URBs")
    ap.add_argument("--urb-lines", type=int, default=16,
                    help="line packets buffered per URB")
    ap.add_argument("--stats-every", type=float, default=1.0, help="seconds")
    ap.add_argument("--set-crop", nargs=2, type=int, metavar=("X", "Y"),
                    help="SET_CROP via Bulk OUT (FPGA EN_CMD_PATH=1)")
    ap.add_argument("--save", metavar="FILE", help="append raw UYVY frames")
    ap.add_argument("--dump-frames", type=int, default=0,
                    help="save first N frames as frame_*.uyvy")
    ap.add_argument("--pyusb", action="store_true",
                    help="force the pyusb fallback transport")
    args = ap.parse_args()

    if args.pyusb:
        rc = run_pyusb(args, Receiver())
    else:
        try:
            rc = run_libusb_async(args, Receiver())
        except LibusbError as e:
            print("libusb async path unavailable:", e)
            print("falling back to pyusb...")
            rc = run_pyusb(args, Receiver())
        except OSError as e:
            print("libusb error:", e)
            rc = 2
    sys.exit(rc)


if __name__ == "__main__":
    main()
