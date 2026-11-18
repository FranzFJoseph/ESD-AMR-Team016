@echo off
chcp 65001 >nul
title Block3 点云 - 实时点云与航向
cd /d "%~dp0"

echo ============================================
echo   Block3 Coding Day  -  实时点云 + 航向
echo   MAC: C4:25:11:06:00:4A
echo ============================================
echo.
echo  [提示] 板子已上电、雷达在转、手机蓝牙已关
echo         模块 LED 慢闪 = 空闲；常亮 = 被别人连着
echo.
echo  正在启动，窗口弹出后请等 2 秒看数据 ...
echo.

"C:\Users\user\AppData\Local\Programs\Python\Python313\python.exe" ble_plot.py --addr C4:25:11:06:00:4A

echo.
echo ============================================
echo   程序已退出
echo   连不上或没数据？双击「2-找我的模块.bat」
echo ============================================
pause
