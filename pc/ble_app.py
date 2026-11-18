#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BLE 版上位机探查工具（用于 HC-04BLE / JDY-08 这类透传模块）

用法：
    py ble_app.py --scan                     # 只扫描
    py ble_app.py --name HC                  # 连接并监听数据
    py ble_app.py --name HC --send "AT"      # 连接后向模块写一条命令（AT over BLE）
    py ble_app.py --name HC --send "AT+BAUD?" --seconds 6

依赖：
    py -m pip install bleak
"""

import argparse
import asyncio
import sys
import time

try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    print("[!] 缺少 bleak，请先运行：py -m pip install bleak")
    sys.exit(1)


# ---------------------------------------------------------------- 扫描
async def do_scan(timeout=8.0):
    print("扫描中（%d 秒）..." % int(timeout))
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    if not found:
        print("没发现任何 BLE 设备")
        return

    rows = [(adv.rssi, addr, dev.name or adv.local_name or "")
            for addr, (dev, adv) in found.items()]
    rows.sort(reverse=True)

    print("%-18s %-28s %s" % ("地址", "名称", "RSSI"))
    print("-" * 60)
    for rssi, addr, name in rows:
        print("%-18s %-28s %d" % (addr, name, rssi))


# ---------------------------------------------------------------- 连接 + 探查
class Stats:
    def __init__(self):
        self.frames = 0
        self.bytes = 0
        self.t0 = time.time()

    def on_data(self, label, data):
        self.frames += 1
        self.bytes += len(data)
        hexs = data[:48].hex(" ")
        if len(data) > 48:
            hexs += " ..."
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in data[:48])
        print("[RX %-8s] %3d B | %-100s | %s" % (label, len(data), hexs, asc))

    def report(self):
        dt = max(time.time() - self.t0, 1e-6)
        print("\n[统计] 帧=%d  字节=%d  速率=%.1f B/s"
              % (self.frames, self.bytes, self.bytes / dt))


def _short(uuid):
    u = uuid.lower()
    return u[4:8] if u.startswith("0000") else u[:8]


async def do_connect(name_sub=None, addr=None, seconds=20.0, send=None):
    dev = None
    if addr:
        print("按地址查找 %s ..." % addr)
        dev = await BleakScanner.find_device_by_address(addr, timeout=10.0)
    else:
        probe = (name_sub or "HC").lower()
        print("查找名字含 %r 的设备 ..." % probe)
        dev = await BleakScanner.find_device_by_filter(
            lambda d, adv: (d.name or adv.local_name or "").lower().find(probe) >= 0,
            timeout=12.0,
        )

    if dev is None:
        print("没找到设备。先跑 --scan 看看名字，再用 --name 指定关键字。")
        return

    print("连接 %s (%r) ..." % (dev.address, dev.name))

    stats = Stats()
    async with BleakClient(dev) as cli:
        print("已连接：%s\n" % cli.address)

        print("==== GATT 服务 ====")
        notif = []      # (char, uuid)
        writable = []   # (char, uuid)
        for s in cli.services:
            print("service %s" % s.uuid)
            for c in s.characteristics:
                print("    char %s  [%s]" % (c.uuid, ",".join(c.properties)))
                if "notify" in c.properties or "indicate" in c.properties:
                    notif.append((c, _short(c.uuid)))
                if "write" in c.properties or "write-without-response" in c.properties:
                    writable.append((c, _short(c.uuid)))

        if not notif:
            print("\n[!] 没有 notify 特征，收不到数据。")
            return

        print("\n==== 订阅 notify（%d 个）====" % len(notif))
        for c, tag in notif:
            try:
                await cli.start_notify(c, lambda ch, d, t=tag: stats.on_data(t, d))
                print("  subscribed %s (%s)" % (c.uuid, tag))
            except Exception as e:
                print("  订阅失败 %s : %s" % (c.uuid, e))

        # ---- 需要写命令时：优先 ffe1 / ff11，其次第一个可写特征 ----
        if send is not None:
            target = None
            for pref in ("ffe1", "ff11", "ffe2"):
                for c, tag in writable:
                    if tag == pref:
                        target = c
                        break
                if target:
                    break
            if target is None and writable:
                target = writable[0][0]

            if target is None:
                print("\n[!] 没有可写特征，无法发送命令。")
            else:
                payload = send.encode("ascii", "ignore") + b"\r\n"
                print("\n>>>> 写入 %s : %r" % (_short(target.uuid), payload))
                use_rsp = "write" in target.properties
                await cli.write_gatt_char(target, payload, response=use_rsp)

        print("\n等待数据 %d 秒（Ctrl+C 提前结束）...\n" % int(seconds))
        try:
            await asyncio.sleep(seconds)
        except asyncio.CancelledError:
            pass
        stats.report()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan", action="store_true", help="只扫描，不连接")
    ap.add_argument("--name", default=None, help="名字关键字，如 HC")
    ap.add_argument("--addr", default=None, help="设备地址")
    ap.add_argument("--seconds", type=float, default=20.0, help="监听时长")
    ap.add_argument("--send", default=None, help="连接后向模块写入一条命令（自动补 \\r\\n）")
    args = ap.parse_args()

    if args.scan:
        asyncio.run(do_scan())
    else:
        asyncio.run(do_connect(args.name, args.addr, args.seconds, args.send))


if __name__ == "__main__":
    main()
