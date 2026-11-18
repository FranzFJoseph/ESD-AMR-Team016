#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BLE 版实时点云上位机（HC-04BLE / JDY-08 透传模块）

用法：
    py ble_plot.py --name HC
    py ble_plot.py --addr C4:25:11:06:00:4A

依赖：
    py -m pip install bleak matplotlib numpy

帧格式（与固件一致）：
    AA 55 | TYPE | SEQ | LEN(2, LE) | PAYLOAD | CRC8(0x07)
    TYPE 0x01 IMU   : t_ms(u32) heading_cdeg(u16) gz_ddps(i16) ax_mg(i16) ay_mg(i16)  = 12 B
    TYPE 0x02 SCAN  : n(u16) + n × [ angle_q6(u16) dist_mm(u16) ]                     = 2 + 4n
    TYPE 0x03 STATUS: lidar_bytes(u32) imu_frames(u32) dropped(u16)                    = 10 B
"""

import argparse
import asyncio
import struct
import sys
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    print("[!] 缺少 bleak，请先运行：py -m pip install bleak")
    sys.exit(1)

SOF = b"\xAA\x55"
BINS = 500
MAX_PAYLOAD = 1200


# ------------------------------------------------------------------ CRC8
def crc8(data: bytes, poly: int = 0x07) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


# ------------------------------------------------------------------ 解析
class FrameParser:
    """字节流 -> [(type, seq, payload)]，自动重同步"""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, chunk: bytes):
        self.buf += chunk
        out = []
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < 6:
                break
            typ = self.buf[2]
            seq = self.buf[3]
            ln = self.buf[4] | (self.buf[5] << 8)
            if ln > MAX_PAYLOAD:
                del self.buf[:2]
                self.errors += 1
                continue
            if len(self.buf) < 7 + ln:
                break
            payload = bytes(self.buf[6:6 + ln])
            got = self.buf[6 + ln]
            if crc8(self.buf[:6 + ln]) != got:
                del self.buf[:2]
                self.errors += 1
                continue
            out.append((typ, seq, payload))
            del self.buf[:7 + ln]
        return out


# ------------------------------------------------------------------ 数据
class Model:
    def __init__(self):
        self.lock = threading.Lock()
        self.dist = np.zeros(BINS, dtype=float)   # 每个角度桶的距离（m）
        self.ts   = np.zeros(BINS, dtype=float)   # 该桶最后一次更新的时间
        self.heading = 0.0
        self.gz = 0.0
        self.ax = 0
        self.ay = 0
        self.imu_frames = 0
        self.scan_frames = 0
        self.crc_err = 0
        self.mcu_drop = 0
        self.rx_bytes = 0
        self._t0 = time.time()
        self._last_rx = time.time()
        self._win = []            # [(t, n)] 最近 2 秒

    def on_chunk(self, n):
        now = time.time()
        self.rx_bytes += n
        self._last_rx = now
        self._win.append((now, n))
        while self._win and now - self._win[0][0] > 2.0:
            self._win.pop(0)

    def alive(self, timeout=5.0):
        return (time.time() - self._last_rx) < timeout

    def on_frame(self, typ, seq, p):
        with self.lock:
            if typ == 0x01 and len(p) >= 12:
                _t, hdg, gz, ax, ay = struct.unpack_from("<IHhhh", p, 0)
                self.heading = (hdg / 100.0) % 360.0
                self.gz = gz / 100.0
                self.ax, self.ay = ax, ay
                self.imu_frames += 1

            elif typ == 0x02 and len(p) >= 2:
                n = p[0] | (p[1] << 8)
                need = 2 + 4 * n
                if len(p) < need:
                    return
                for k in range(n):
                    a, d = struct.unpack_from("<HH", p, 2 + 4 * k)
                    if d == 0:
                        continue
                    bin_i = int(a / 64.0 / 360.0 * BINS) % BINS
                    self.dist[bin_i] = d / 1000.0
                    self.ts[bin_i] = time.time()
                self.scan_frames += 1

            elif typ == 0x03 and len(p) >= 10:
                _lb, _imf, drop = struct.unpack_from("<IIH", p, 0)
                self.mcu_drop = drop

    def rate(self):
        now = time.time()
        tot = sum(n for ts, n in self._win if now - ts <= 1.0)
        return tot / 1024.0


# ------------------------------------------------------------------ BLE
async def ble_loop(model: Model, parser: FrameParser, stop_evt, name_sub, addr):
    if addr:
        dev = await BleakScanner.find_device_by_address(addr, timeout=10.0)
    else:
        probe = (name_sub or "HC").lower()
        dev = await BleakScanner.find_device_by_filter(
            lambda d, adv: (d.name or adv.local_name or "").lower().find(probe) >= 0,
            timeout=15.0,
        )
    if dev is None:
        print("[!] 找不到设备")
        stop_evt.set()
        return

    print("[+] 连接 %s (%r) ..." % (dev.address, dev.name))

    def on_notify(_ch, data: bytearray):
        global _dbg_done
        if not _dbg_done and len(data):
            _dbg_done = True
            print('[dbg] first bytes:', ' '.join('%02x' % b for b in data[:32]))
        model.on_chunk(len(data))
        for typ, seq, p in parser.feed(bytes(data)):
            model.on_frame(typ, seq, p)
        model.crc_err = parser.errors

    async with BleakClient(dev) as cli:
        print("[+] 已连接，订阅 ffe1")

        target = None
        for s in cli.services:
            for c in s.characteristics:
                u = c.uuid.lower()
                if "ffe1" in u or "fff1" in u or "ff01" in u:
                    target = c
                    break
            if target:
                break
        if target is None:                       # 退而求其次：第一个 notify
            for s in cli.services:
                for c in s.characteristics:
                    if "notify" in c.properties:
                        target = c
                        break
                if target:
                    break

        if target is None:
            print("[!] 没找到可通知的特征")
            stop_evt.set()
            return

        await cli.start_notify(target, on_notify)
        print("[+] 已订阅 %s" % target.uuid)

        stale = 0
        while not stop_evt.is_set():
            await asyncio.sleep(0.5)
            if not cli.is_connected:
                print("[!] 连接已断开")
                return
            if model.alive(5.0):
                stale = 0
            else:
                stale += 1
                if stale >= 2:          # 连续 1 秒没有数据 -> 判定链路僵死
                    print("[!] 5 秒无数据，重连")
                    return


# ------------------------------------------------------------------ 绘图
_dbg_done = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="HC", help="设备名关键字")
    ap.add_argument("--addr", default=None, help="设备地址")
    ap.add_argument("--span", type=float, default=6.0, help="显示半径（米）")
    args = ap.parse_args()

    model = Model()
    parser = FrameParser()
    stop_evt = threading.Event()

    def worker():
        while not stop_evt.is_set():
            try:
                asyncio.run(ble_loop(model, parser, stop_evt, args.name, args.addr))
            except Exception as e:
                print("[!] BLE 异常: %s" % e)
            if stop_evt.is_set():
                break
            print("[*] 2 秒后重连 ...")
            time.sleep(2.0)

    th = threading.Thread(target=worker, daemon=True)
    th.start()

    fig, ax = plt.subplots(figsize=(9, 7))
    fig.canvas.manager.set_window_title("Block3 - LiDAR over BLE")
    scat = ax.scatter([], [], s=8)
    arrow = ax.quiver([0], [0], [0], [0], angles="xy", scale_units="xy",
                      scale=1.0 / 1.5, color="crimson", width=0.012)
    hud = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                  family="monospace", fontsize=9,
                  bbox=dict(fc="white", ec="0.8", alpha=0.9))

    ax.set_xlim(-args.span, args.span)
    ax.set_ylim(-args.span, args.span)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("X (m) - right")
    ax.set_ylabel("Y (m) - forward")
    ax.set_title("RPLIDAR C1 + MPU6500 over BLE")

    def update(_):
        with model.lock:
            now = time.time()
            expired = (now - model.ts) > 2.0        # 超过 2 秒没更新的桶清掉
            model.dist[expired] = 0.0
            dist = model.dist.copy()
            hdg = model.heading
            hud_txt = (
                "heading : %6.2f deg\n"
                "gz      : %6.2f deg/s\n"
                "accel   : %4d %4d mg\n"
                "scan fr : %6d  (crc err %d)\n"
                "imu fr  : %6d\n"
                "rx      : %5.1f kB/s\n"
                "mcu drop: %6d\n"
                "link    : %s"
                % (hdg, model.gz, model.ax, model.ay,
                   model.scan_frames, model.crc_err,
                   model.imu_frames, model.rate(), model.mcu_drop,
                   "OK" if model.alive(3.0) else "STALE (reconnecting)")
            )

        idx = np.nonzero(dist)[0]
        if len(idx):
            ang = np.radians(idx * 360.0 / BINS)
            r = dist[idx]
            x = r * np.cos(ang)
            y = -r * np.sin(ang)          # 屏幕坐标：雷达顺时针 -> 视觉不镜像
            scat.set_offsets(np.column_stack([x, y]))
            scat.set_sizes(np.full(len(x), 8.0))
        else:
            scat.set_offsets(np.empty((0, 2)))

        h = np.radians(hdg)
        arrow.set_UVC(np.cos(h), -np.sin(h))
        hud.set_text(hud_txt)
        return scat, arrow, hud

    ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()


if __name__ == "__main__":
    main()
