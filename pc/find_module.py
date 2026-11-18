#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
find_module.py —— 在有多块同名 HC-04BLE 的环境里，找出「哪一块是你的」

做法：扫描所有名字里含 HC 的 BLE 设备，逐个连上去、订阅 ffe1、等 2 秒，
看这 2 秒里有没有收到以 AA 55 开头的数据。收到的那块就是你的板子。

用法：
    py find_module.py                 # 默认找名字含 "HC" 的设备
    py find_module.py --key HC-04     # 指定名字关键字
    py find_module.py --wait 3        # 每块设备观察 3 秒（默认 2 秒）

找到之后，把打印出来的 MAC 地址记下来（建议写在贴纸上贴到板子背面），
以后就用固定地址连接：

    py ble_plot.py --addr C4:25:01:17:05:AD
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

# 模块的透传特征（不同批次可能不同，这里都试一遍）
CAND_CHARS = [
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    "0000ff11-0000-1000-8000-00805f9b34fb",
    "0000ff12-0000-1000-8000-00805f9b34fb",
]


async def probe(dev, wait_s: float):
    """连上去看 wait_s 秒，返回 (字节数, 是否出现 AA55, 首包预览)"""
    total = 0
    saw_sof = False
    preview = b""
    evt = asyncio.Event()

    def on_notify(_ch, data: bytearray):
        nonlocal total, saw_sof, preview
        if not data:
            return
        total += len(data)
        if not preview:
            preview = bytes(data[:16])
        if b"\xAA\x55" in bytes(data):
            saw_sof = True
            evt.set()

    try:
        async with BleakClient(dev, timeout=10.0) as cli:
            subscribed = 0
            for uuid in CAND_CHARS:
                try:
                    await cli.start_notify(uuid, on_notify)
                    subscribed += 1
                except Exception:
                    pass
            if subscribed == 0:
                return 0, False, b"", "没有可订阅的特征"

            deadline = time.time() + wait_s
            while time.time() < deadline and not saw_sof:
                await asyncio.sleep(0.1)
            try:
                for uuid in CAND_CHARS:
                    try:
                        await cli.stop_notify(uuid)
                    except Exception:
                        pass
            except Exception:
                pass
            return total, saw_sof, preview, ""
    except Exception as e:
        return 0, False, b"", str(e)[:60]


def hexs(b: bytes) -> str:
    return " ".join("%02x" % x for x in b) if b else "(无)"


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--key", default="HC", help="设备名字关键字（默认 HC）")
    ap.add_argument("--wait", type=float, default=2.0, help="每块设备观察秒数")
    ap.add_argument("--scan", type=float, default=6.0, help="扫描秒数")
    args = ap.parse_args()

    print("[*] 扫描 %.0f 秒，找名字含 '%s' 的设备 ..." % (args.scan, args.key))
    found = await BleakScanner.discover(timeout=args.scan)

    cands = [d for d in found if args.key.lower() in (d.name or "").lower()]
    if not cands:
        print("[!] 没找到。先确认：模块上电、LED 慢闪、没被手机连走。")
        print("    附近所有设备：")
        for d in found:
            print("      %-18s %s" % (d.address, d.name or "(无名)"))
        return 1

    print("[*] 找到 %d 块候选设备，逐个试（每块 %.1f 秒）...\n" % (len(cands), args.wait))

    good = []
    for i, d in enumerate(cands, 1):
        name = d.name or "(无名)"
        print("[%d/%d] %-18s %-14s" % (i, len(cands), d.address, name), end=" ", flush=True)
        n, ok, prev, err = await probe(d, args.wait)
        if err:
            print("✗ %s" % err)
        elif ok:
            print("✔ 收到 AA 55，共 %d 字节" % n)
            good.append((d, n, prev))
        elif n:
            print("… 有 %d 字节但不是 AA 55  [%s]" % (n, hexs(prev)))
        else:
            print("… 无数据")
        await asyncio.sleep(0.5)

    print()
    if good:
        print("===== 你的模块 =====")
        for d, n, prev in good:
            print("  MAC : %s" % d.address)
            print("  名字: %s" % (d.name or "(无名)"))
            print("  首包: %s" % hexs(prev))
            print()
        print("记下 MAC，以后这样用：")
        print("  py ble_plot.py --addr %s" % good[0][0].address)
        return 0

    print("[!] 所有候选设备都没有发出 AA 55。")
    print("    可能原因：")
    print("      1) 板子没在跑（按一下黑色 RESET）")
    print("      2) 你的模块没被搜到（被手机连着 / 没上电）")
    print("      3) 固件波特率与模块不匹配（改 TELEM_BAUD）")
    return 2


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        print("\n[*] 已取消")
