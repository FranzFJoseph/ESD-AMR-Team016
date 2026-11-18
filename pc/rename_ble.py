#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rename_ble.py —— 给 HC-04 / JDY-08 这类 BLE 透传模块改名字（AT over BLE）

背景：这块模块的 UART 侧 AT 走不通（PA3 被 ST-Link 占用、UART4 也试过），
      但大多数 JDY/HC 系列支持**通过 BLE 写特征**发 AT 命令。

本脚本会自动尝试常见的几种组合：
    写特征：ffe1 / ff12
    行尾  ：\\r\\n  /  \\r  /  无
哪种组合能收到回复（OK / OK+... ），就用它把名字改掉。

用法：
    py rename_ble.py --name Group1                 # 改成 Group1
    py rename_ble.py --name Group1 --addr C4:25:01:17:05:AD
    py rename_ble.py --ask                         # 只查询当前名字，不改

改完记得：模块会重启广播，Windows 里要重新配对（旧名字还在就删掉）。
"""

import argparse
import asyncio
import sys
import time

try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    print("[!] 缺少 bleak，请先安装：py -m pip install bleak")
    sys.exit(1)

WRITE_CHARS = [
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    "0000ff12-0000-1000-8000-00805f9b34fb",
    "0000ff11-0000-1000-8000-00805f9b34fb",
]
NOTIFY_CHARS = [
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    "0000ff11-0000-1000-8000-00805f9b34fb",
    "0000ff12-0000-1000-8000-00805f9b34fb",
]
ENDINGS = [b"\r\n", b"\r", b""]


class Session:
    def __init__(self, cli):
        self.cli = cli
        self.rx = bytearray()
        self.evt = asyncio.Event()

    async def subscribe(self):
        ok = 0
        for uuid in NOTIFY_CHARS:
            try:
                await self.cli.start_notify(uuid, self._on)
                ok += 1
            except Exception:
                pass
        return ok

    def _on(self, _ch, data: bytearray):
        if data:
            self.rx += data
            self.evt.set()

    def clear(self):
        self.rx.clear()
        self.evt.clear()

    async def send(self, uuid: str, payload: bytes, wait_s: float = 1.2) -> bytes:
        self.clear()
        try:
            await self.cli.write_gatt_char(uuid, payload, response=False)
        except Exception:
            try:
                await self.cli.write_gatt_char(uuid, payload, response=True)
            except Exception as e:
                return b"<write failed: %s>" % str(e)[:40].encode()
        deadline = time.time() + wait_s
        while time.time() < deadline:
            if self.evt.is_set():
                await asyncio.sleep(0.15)
                break
            await asyncio.sleep(0.05)
        return bytes(self.rx)


def show(b: bytes) -> str:
    if not b:
        return "(无回复)"
    txt = b.decode("utf-8", "replace").replace("\r", "").replace("\n", " | ")
    return txt[:80]


async def find_device(args):
    if args.addr:
        print("[*] 直接使用地址 %s" % args.addr)
        return args.addr

    print("[*] 扫描 6 秒找名字含 '%s' 的设备 ..." % args.key)
    found = await BleakScanner.discover(timeout=6.0)
    cands = [d for d in found if args.key.lower() in (d.name or "").lower()]
    if not cands:
        print("[!] 没找到设备。确认模块上电、没被手机连着。")
        for d in found:
            print("      %-18s %s" % (d.address, d.name or "(无名)"))
        return None
    if len(cands) > 1:
        print("[!] 找到 %d 块同名设备，先都列出（建议下次用 --addr 指定）：" % len(cands))
        for d in cands:
            print("      %-18s %s" % (d.address, d.name or "(无名)"))
    return cands[0].address


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=None, help="要设置的新名字，例如 Group1")
    ap.add_argument("--ask", action="store_true", help="只查询当前名字")
    ap.add_argument("--addr", default=None, help="直接指定 MAC 地址")
    ap.add_argument("--key", default="HC", help="设备名关键字，默认 HC")
    args = ap.parse_args()

    if not args.name and not args.ask:
        print("[!] 至少给一个 --name 或 --ask")
        return 1

    addr = await find_device(args)
    if not addr:
        return 2

    print("[*] 连接 %s ..." % addr)
    async with BleakClient(addr, timeout=15.0) as cli:
        s = Session(cli)
        n = await s.subscribe()
        print("[+] 已订阅 %d 个通知特征" % n)

        # ---- 第一步：找出哪种 (特征, 行尾) 组合能得到回复 ----
        print("\n[*] 正在试探 AT 通道（最多 9 种组合）...")
        good = None
        for uuid in WRITE_CHARS:
            for end in ENDINGS:
                reply = await s.send(uuid, b"AT" + end)
                tag = "%s + %s" % (uuid[4:8], {b"\r\n": "CRLF", b"\r": "CR", b"": "无行尾"}[end])
                print("    %-12s -> %s" % (tag, show(reply)))
                if reply and not reply.startswith(b"<write failed"):
                    if (b"OK" in reply) or (b"AT+" in reply) or (b"ERROR" in reply):
                        good = (uuid, end)
                        print("    ^^^ 这个组合有戏\n")
                        break
                    else:
                        # 收到的不是 ASCII 文本，多半是模块把 UART 上的遥测流转发过来了
                        printable = sum(32 <= c < 127 for c in reply[:32])
                        if printable < 20:
                            print("    ^^^ 非 ASCII 数据流（UART 遥测），不是 AT 回复")
            if good:
                break

        if not good:
            print("\n[!] 所有组合都没有回复。可能：")
            print("    1) 模块不支持 AT over BLE（只能走 UART）")
            print("    2) 模块已被别的设备连着（手机关蓝牙）")
            print("    3) 需要先断开 BLE 再上电进入 AT 模式")
            print("\n    退路：用 py find_module.py 记录 MAC，不改名字。")
            return 3

        uuid, end = good

        # ---- 第二步：查询 / 设置名字 ----
        r = await s.send(uuid, b"AT+NAME?" + end)
        print("[查询] AT+NAME?  -> %s" % show(r))

        if args.name:
            new = args.name.encode()
            print("[设置] 发送 AT+NAME=%s ..." % args.name)
            r = await s.send(uuid, b"AT+NAME=" + new + end, wait_s=2.0)
            print("[设置] 回复 -> %s" % show(r))
            await asyncio.sleep(0.5)
            r = await s.send(uuid, b"AT+NAME?" + end)
            print("[复核] AT+NAME?  -> %s" % show(r))
            print("\n[+] 如果复核显示新名字，就成功了。")
            print("    模块可能自动重启，Windows 里删掉旧的配对记录再重新搜索。")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        print("\n[*] 已取消")
