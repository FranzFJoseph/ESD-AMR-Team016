@echo off
chcp 65001 >nul
title Block3 找我的模块
cd /d "%~dp0"

echo ============================================
echo   找出「哪一块 HC-04BLE 是你的板子」
echo ============================================
echo.
echo  原理：逐个连上去看 2 秒，谁在发 AA 55 就是你
echo  前提：板子正在跑、雷达在转、手机蓝牙已关
echo.

"C:\Users\user\AppData\Local\Programs\Python\Python313\python.exe" find_module.py

echo.
echo ============================================
echo   把上面打印的 MAC 抄下来，贴到板子背面
echo   然后改「1-启动点云.bat」里的 MAC 即可
echo ============================================
pause
