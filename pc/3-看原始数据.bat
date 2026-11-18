@echo off
chcp 65001 >nul
title Block3 看原始数据
cd /d "%~dp0"

echo ============================================
echo   看模块发来的原始字节（十六进制）
echo ============================================
echo.
echo  正常应看到  aa 55 01 / aa 55 02 / aa 55 03
echo  若是 80 78 3c 之类 -> 波特率不对
echo  若是 00 00 00 一片 -> 模块 RXD 没接到东西
echo.

"C:\Users\user\AppData\Local\Programs\Python\Python313\python.exe" ble_app.py --name HC

echo.
echo ============================================
echo   程序已退出
echo ============================================
pause
