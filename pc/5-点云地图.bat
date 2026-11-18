@echo off
chcp 936 >nul
title Block4.5 点云地图
cd /d "%~dp0"

echo ============================================
echo   Block4.5 点云地图
echo ============================================
echo   1. 先给板子上电、烧好带 S 行的固件
echo   2. 浏览器打开后点"连接蓝牙"，选 HC-04BLE
echo   3. 页面用完，这个黑窗口要手动关
echo ============================================
echo.

start "" /min py -m http.server 8765 --bind 127.0.0.1
timeout /t 2 >nul

set "CHROME=C:\Program Files\Google\Chrome\Application\chrome.exe"
if exist "%CHROME%" (
  start "" "%CHROME%" "http://127.0.0.1:8765/map.html"
) else (
  start "" msedge "http://127.0.0.1:8765/map.html"
)

echo 已启动。若浏览器没弹出，手动打开 http://127.0.0.1:8765/map.html
echo.
pause