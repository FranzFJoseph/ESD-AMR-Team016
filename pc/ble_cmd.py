#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Block4.5 —— 蓝牙指令终端（交互式）

用法：
    py ble_cmd.py --name HC            # 安静模式：不刷遥测，方便输入
    py ble_cmd.py --name HC --rt       # 显示每秒遥测

命令（回车即发送，自动补 \\r\\n）：
    GOTO 0 1.4   前往目标 (x, y)，单位米
    STOP         立即停
    F300   前进 300 mm        B300   后退 300 mm
    L90    左转 90 度         R90    右转 90 度
    S      立即停             st     打印最近一条遥测
    quit   退出
"""

import argparse
import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"


def ts() -> str:
    return time.strftime("%H:%M:%S")


async def find(name_sub, addr):
    if addr:
        d = await BleakScanner.find_device_by_address(addr, timeout=8.0)
        if d:
            return d
    if name_sub:
        for d in await BleakScanner.discover(timeout=6.0):
            if name_sub.lower() in (d.name or "").lower():
                return d
    return None


async def run(name_sub, addr, show_rt):
    dev = await find(name_sub, addr)
    if dev is None:
        print("[!] 找不到设备：确认模块上电、手机蓝牙已断开、名字/MAC 正确")
        return 1

    print(f"[+] 连接 {dev.address} ({dev.name}) ...")

    got = 0
    buf = bytearray()
    last_rt = ["(还没有遥测)"]
    last_nav = ["(还没有导航状态)"]

    def on_data(_c, data: bytearray):
        nonlocal got
        got += len(data)
        buf.extend(data)
        while True:
            i = -1
            for k, ch in enumerate(buf):
                if ch in (10, 13):
                    i = k
                    break
            if i < 0:
                break
            line = bytes(buf[:i]).decode("ascii", "ignore").strip()
            del buf[: i + 1]
            if not line:
                continue
            if line.startswith("RT "):
                last_rt[0] = line
                if show_rt:
                    print(f"\r[{ts()}] {line}")
            elif line.startswith("N "):
                if line != last_nav[0]:
                    print(f"\r[{ts()}] {line}")
                last_nav[0] = line
            elif line.startswith(("P ", "S ", "W ", "T ", "D ")):
                if show_rt and line.startswith(("T ", "D ")):
                    print(f"\r[{ts()}] {line}")
            else:
                print(f"\r[{ts()}] {line}")

    async with BleakClient(dev) as cli:
        print("[+] 已连接，订阅中 ...")
        await cli.start_notify(FFE1, on_data)

        print("    命令: GOTO 0 1.4 / STOP / F300 / B300 / L90 / R90 / S / st / quit")
        if not show_rt:
            print("    (遥测默认不刷屏，输入 st 看最新一条；要实时看加 --rt)")

        while True:
            try:
                line = (await asyncio.to_thread(input, "cmd> ")).strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not line:
                continue
            low = line.lower()
            if low in ("quit", "exit", "q"):
                break
            if low in ("st", "status"):
                print(f"    {last_rt[0]}")
                print(f"    {last_nav[0]}")
                continue

            try:
                data = (line + "\r\n").encode("ascii")
            except UnicodeEncodeError:
                print("[!] 指令只支持 ASCII。")
                continue
            characteristic = cli.services.get_characteristic(FFE1)
            if characteristic is None:
                print("[!] BLE 服务中未找到 FFE1。")
                break
            without_response = "write-without-response" in characteristic.properties
            for start in range(0, len(data), 20):
                await cli.write_gatt_char(characteristic, data[start:start + 20], response=not without_response)
            print(f"[>] {line}")

        try:
            await cli.stop_notify(FFE1)
        except Exception:
            pass

    print(f"[*] 断开，共收 {got} 字节")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="HC-04BLE", help="设备名包含的字符串，默认 HC-04BLE")
    ap.add_argument("--addr", default=None, help="直接指定 MAC")
    ap.add_argument("--rt", action="store_true", help="实时显示每秒遥测")
    a = ap.parse_args()

    if not a.name and not a.addr:
        ap.print_help()
        return 2

    try:
        return asyncio.run(run(a.name, a.addr, a.rt))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
