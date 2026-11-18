# ESD 自主小车

基于 NUCLEO-F446RE、CMSIS-RTOS2 和 FreeRTOS 的差速小车。STM32 接收 HC-04BLE 的目标坐标，生成航点，用编码器和 IMU 估计位姿并执行闭环控制。PC 网页负责下发目标、显示点云和轨迹、导出测试数据。

当前基线来自团队 2026-11-18 的 CD4 Part 2 交接。验收前优先验证新 IMU 的识别与静止标定、带载 +Y 直行、障碍停车与制停距离，以及三条课程路线。

| 入口 | 内容 |
| --- | --- |
| [接线与构建](README_Part2.md) | Bench 和 Drive 命令、本车标定、RTOS 任务表、台架与实车验证 |
| [固件](firmware/build.ps1) | 构建入口。板级和任务在 [main.c](firmware/Core/Src/main.c)，导航在 [nav.c](firmware/Core/Src/nav.c) |
| [上位机](pc/Part2_上位机.md) | Web Bluetooth、网页操作、诊断字段和 CSV |

## 构建与验证

在 `firmware` 目录运行 `build.ps1`。脚本可使用 PATH 上的 Arm GNU、STM32CubeIDE 随附的工具链，或通过 `-ToolchainBin` 指定路径。交接镜像使用 GNU Arm GCC 12.3.1。默认构建 Bench，PWM 固定为零。Drive 使用本车标定参数和 353° 雷达车头角。复现交接中的带避障 Drive 时，传入 `-EnableAvoidance`。CubeIDE 元数据中的旧输出路径仍需核对，本项目使用脚本构建。

在 `pc` 目录运行 `node --test part2_protocol.test.js` 验证协议和导出逻辑。运行 `py -m http.server 8765 --bind 127.0.0.1` 后，用 Chrome 或 Edge 打开 [点云与导航界面](http://127.0.0.1:8765/map.html)。

## 团队协作

- `main` 是团队共享版本。每次改动从最新 `main` 开分支，提交后发 PR。固件与上位机协议同时变化时，在同一 PR 中说明两端兼容情况。
- PR 写清改动、验证环境和实际结果。联调记录附 commit SHA、构建参数、HEX SHA-256 和日志位置，便于不同电脑复现同一版。
- 构建输出、原始交接包、课程资料、本地烧录备份与原始 CSV 由项目工作区归档。准备验收时，给团队确认并实测的提交打标签。
