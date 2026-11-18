#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
极简串口终端（用于给 HC-04 发 AT 命令）

用法:
    py serial_console.py COM5 9600
    py serial_console.py COM5 9600 --hex      # 收到的数据按十六进制显示

退出: Ctrl+C
"""
import sys
import threading

try:
    import serial
except ImportError:
    print("没装 pyserial:  py -m pip install pyserial")
    sys.exit(1)

args = [a for a in sys.argv[1:] if not a.startswith("--")]
HEX = "--hex" in sys.argv

port = args[0] if len(args) > 0 else "COM5"
baud = int(args[1]) if len(args) > 1 else 9600

try:
    ser = serial.Serial(port, baud, timeout=0.1)
except Exception as e:
    print(f"打不开 {port}: {e}")
    print("检查：设备管理器里的 COM 号对不对？串口助手是不是还占着？")
    sys.exit(1)

print(f"[+] 已打开 {port} @ {baud} 8N1")
print("    直接输入命令并回车（会自动补 \\r\\n）；Ctrl+C 退出")
print("-" * 50)


def rx_loop():
    while True:
        try:
            data = ser.read(256)
        except Exception:
            break
        if not data:
            continue
        if HEX:
            sys.stdout.write(" ".join(f"{b:02X}" for b in data) + "\n")
        else:
            sys.stdout.write(data.decode("utf-8", "replace"))
        sys.stdout.flush()


threading.Thread(target=rx_loop, daemon=True).start()

try:
    while True:
        line = input()
        ser.write((line + "\r\n").encode("ascii", "ignore"))
except (KeyboardInterrupt, EOFError):
    pass
finally:
    ser.close()
    print("\n[+] 已关闭串口")
