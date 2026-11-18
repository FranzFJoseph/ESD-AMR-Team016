#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口回环测试：验证 ST-Link 虚拟串口到 PA2/PA3 的通路

用法：
    1. 拔掉 D0/D1 上接模块的两根线
    2. 用一根杜邦线把 D0 和 D1 直接短接
    3. py loopback_test.py COM5

结果：
    收到 abc -> ST-Link 通路正常
    收不到   -> VCP 没接到 PA2/PA3（SB13/SB14 断开或驱动问题）
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM5"

try:
    s = serial.Serial(port, 9600, timeout=0.5)
except Exception as e:
    print(f"[X] 打不开 {port}: {e}")
    print("    确认板子 USB 插好、设备管理器里有这个口")
    sys.exit(1)

time.sleep(0.2)
s.reset_input_buffer()

print(f"[+] 已打开 {port}，发送 abc\\r\\n ...")
s.write(b"abc\r\n")
time.sleep(0.4)
data = s.read(64)
s.close()

if data:
    print(f"[OK] 收到 {len(data)} 字节: {data!r}")
    print("     -> ST-Link 通路正常，问题在蓝牙模块那一侧")
else:
    print("[X] 一个字节都没回来")
    print("     -> ST-Link 的 VCP 没有接到 PA2/PA3（检查 SB13/SB14），")
    print("        或者 D0/D1 没有真的短接上")
