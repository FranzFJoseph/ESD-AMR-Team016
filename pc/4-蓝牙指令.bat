@echo off
chcp 936 >nul
title Block4.5 蓝牙指令终端
cd /d "%~dp0"
echo ============================================
echo  Block4.5 蓝牙指令终端
echo  连接后输入 GOTO 0 1.4 / STOP / F300 / B300 / L90 / R90 / S, quit 退出
echo ============================================
py ble_cmd.py --name HC-04BLE
echo.
pause