# Coding Day 4 Part 2 队友工程续作

本工程保留队友的 STM32CubeIDE 工程、AT8236 和编码器引脚、HC-04BLE、OLED、旋钮以及网页点云。STM32F446RE 接收一次目标坐标后，在车上生成航点，用双轮编码器和 IMU 更新位姿，以路径及轮速 PID 行驶。网页负责发送指令、接收和记录遥测。

## 使用本版的接线

电池和 USB 都断开后再移动线。表中电机驱动和编码器使用队友工程的分配。若接过早先的 `Part2_Navigation` 固件，请逐根核对。驱动板侧名称按实物丝印和接头定义核对。

| 接口 | Nucleo 引脚 |
| --- | --- |
| 左轮 PWM AIN1/AIN2 | PB6（CN10-17）、PB7（CN7-21） |
| 右轮 PWM BIN1/BIN2 | PB14（CN10-28）、PB15（CN10-26） |
| 左轮编码器 A/B | PA6（CN10-13）、PA7（CN10-15） |
| 右轮编码器 A/B | PC6（CN10-4）、PC7（CN10-19） |
| OLED、MPU SCL/SDA | PB8（CN10-3）、PB9（CN10-5） |
| C1 雷达 TX/RX | TX → PA10（CN10-33），RX ← PA9（CN10-21），460800 8N1 |
| HC-04BLE TXD/RXD | TXD → PA1（CN7-30），RXD ← PA0（CN7-28），模块侧 UART4 921600 8N1 |
| 旋钮 OT1/OT2/OT3 | PC0/PC1/PC2，OT1/OT2 调节手动直行和转向上限 |
| 电脑 USB 台架 | ST-LINK USB 虚拟串口，曾识别为 COM4，115200 8N1 |

IMU 和 OLED 由 3.3 V 供电，IMU 的 NCS 置高、AD0 接地（I²C 地址 0x68）。用独立稳压 5 V 支路供 C1，给约 800 mA 的启动电流留余量。电机电源走驱动板动力端。所有相连信号的两端共地，编码器与蓝牙信号电平需符合 Nucleo 的 3.3 V 输入/输出。 [ST Nucleo 排针表 29](https://www.st.com/resource/en/user_manual/um1724-.pdf) · [SLAMTEC C1 规格](https://www.slamtec.com/en/c1/spec)

## 编译与烧录

在工程的 `firmware` 目录执行以下命令。脚本使用 STM32CubeIDE 随附的 Arm GNU。也可导入队友的 CubeIDE 工程构建台架版。本工程的初始化维护在源文件中，`.ioc` 供对照。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\build.ps1
```

烧录 `Build/Bench/Block45_Part2.hex` 进行台架验证。该版本四路电机 PWM 保持为零。电池断开时依次检查姿态、雷达点云、队列和任务时序。USB 用法见 [上位机测试说明](./pc/Part2_上位机.md)。若已有在板程序，先备份，再通过 ST-LINK/SWD 烧录并校验。

若小车有航点和点云却不动，先看 OLED 顶行。显示 `PART 2 BENCH` 时，接上动力电源，PWM 仍保持零。旧版 Bench 会在目标下发约 45 秒后显示通用故障。更新后的 Bench 显示「台架预览」，网页同时显示固件版本、IMU 就绪、左右轮编码器计数与 PWM 指令。网页要收到新版固件的诊断帧才能显示这些字段。

正式驱动前，实测轮径、轮距和左右轮每圈的正交计数。手转车轮一圈后分别读 TIM3、TIM8 的计数变化并处理 16 位回绕，确定 `-LeftCpr` 和 `-RightCpr` 以及前转的计数符号。接上动力后先架空验证电机方向和物理断电开关，再低速试跑。电机符号默认使用队友原值，左右分别为 −1、+1，编码器分别为 +1、−1，按实物可用 `-LeftMotorSign`、`-RightMotorSign`、`-LeftEncoderSign`、`-RightEncoderSign` 修正。左转时航向应增大，反向时用 `-ImuSign -1`。

用当前 Bench 测量时，架空并隔离电机输出，为编码器保留其所需供电。在 `pc` 目录运行下方命令，按提示分别向前手转左右轮一整圈，将两个**带正负号**的结果和轮径、轮距一起记录。轮距量左右两条胎面中心线之间的距离。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\测编码器.ps1 -Port COM4
```

只复测一侧时，在命令末尾加 `-Wheel Left` 或 `-Wheel Right`。每次按回车后，等屏幕显示起始计数和手转提示再转。转完后停住，等待三个连续读数一致。预期只有被转动的那一侧出现明显计数变化。

```powershell
$wheel = [double](Read-Host '实测轮径，单位 mm')
$track = [double](Read-Host '实测轮距，单位 mm')
$leftTicks = [int](Read-Host '左轮向前手转一圈的 x4 计数，保留正负号')
$rightTicks = [int](Read-Host '右轮向前手转一圈的 x4 计数，保留正负号')
$leftCpr = [Math]::Abs($leftTicks)
$rightCpr = [Math]::Abs($rightTicks)
$leftSign = [Math]::Sign($leftTicks)
$rightSign = [Math]::Sign($rightTicks)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\build.ps1 `
  -EnableMotors -WheelMm $wheel -TrackMm $track -LeftCpr $leftCpr -RightCpr $rightCpr `
  -LeftEncoderSign $leftSign -RightEncoderSign $rightSign
```

驱动固件位于 `Build/Drive/Block45_Part2.hex`。驱动版从 HC-04BLE 接收坐标，开机静止校准 IMU。若 GOTO 等待校准、已开始后 IMU 数据超时、轮速命令超过 40 mm/s 且编码器 0.8 s 无脉冲，或一次运行超过 45 s，状态会保持等待或转为故障，电机输出回到零。STOP 使旧命令立即失效，下一个控制周期输出为零。

### LiDAR 保护停车

Drive 在车上处理 C1 的原始测距点。近障锁存和绕行需要构建时传入 `-EnableAvoidance`。全周一个 250 mm 内的有效回波，或 280 mm 内的成簇回波，会触发停车。前进和倒车时，行驶方向上还检查 400 mm 内、距雷达纵向中心线不超过 145 mm 的成簇回波。成簇回波指 25 ms 内两个相隔不超过 4° 的有效回波。原地转向使用近距全周保护。未标定车头角度时，成簇回波使用 400 mm 全周保护。

轮速目标上限为 120 mm/s。保护同时检查目标行驶方向和编码器测到的当前行驶方向，覆盖换向时的滑行阶段。输出从行驶转为停止时，AT8236 两输入同时置高制动 250 ms，随后释放。Bench 保持四路输出为零。没有完整有效扫描或上一圈有效扫描超过 300 ms 时，同样会保护停车。保护停车会终止当前指令，恢复后需要重新发车。

测距从雷达旋转中心起算。试车先断开电机动力，在车头正前方、扫描高度摆放直立白板，距离以网页实际测距为准。网页显示保护区有障碍后发送一次 GOTO，检查状态进入「近处障碍停车」。若此前已输出驱动，网页的控制诊断会短时标记主动制动。A 帧的 PWM 为有符号驱动指令，制动时为零，C 帧另报电桥制动状态。移开白板等扫描恢复，旧指令应保持停住。随后架空核对两轮输出与 STOP，再在开阔地低速测量实际制停距离。低于扫描平面的物体需要用其他手段检测。

### 航向与左右轮诊断

IMU 在启动和重新连接时明确设置并回读 100 Hz、陀螺 ±250°/s、加速度 ±2 g 及低通滤波配置。启动用 200 个连续静止样本估计偏置。停车、编码器静止、加速度与角速度条件持续满足后，保持航向并缓慢更新零偏。GOTO 使用当前绝对坐标系。

网页的 C 控制诊断显示两轮目标及实测速度、陀螺航向、融合航向、零偏和制动状态。向 +Y 的短直线应从物理起点、车头朝 +Y 静止复位开始。CSV 与地面尺量并行记录，区分控制器请求转向、轮速响应差异与坐标估计偏差。左右 CPR 仍采用手测的 1621 和 1664。

### 本车的实测参数

轮径约 68 mm，轮距约 171 mm。手转左轮一圈得到 +1621、右轮增量为 0。手转右轮一圈得到 −1664、左轮增量为 0。构建用计数的绝对值和各自的编码器方向。电机方向沿用队友设定（左 −1、右 +1），IMU 方向为 +1。车头方向的雷达原始角约为 353°（原先记成 173°，与车头相差 180°）。该值作为 `-LidarFrontDeg` 构建参数。若拆装后改变雷达方向，需重新标定。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\build.ps1 `
  -EnableMotors -EnableAvoidance -WheelMm 68 -TrackMm 171 -LeftCpr 1621 -RightCpr 1664 `
  -LeftEncoderSign 1 -RightEncoderSign -1 -LeftMotorSign -1 -RightMotorSign 1 -ImuSign 1 -LidarFrontDeg 353
```

首次测试先断开电机动力，架空小车再烧录。OLED 顶行应显示 `PART 2 DRIVE`。保持车身静止直至 IMU 就绪。通过 [上位机](./pc/Part2_上位机.md)连接 HC-04BLE，确认 STOP 可用，再接电机动力。仍架空时发短距离指令检查两轮是否都朝车身前进方向转动。核对方向并确认 STOP 后，再放车下地。

## 课件验收

| 任务 / 逻辑 | 触发 | CMSIS 优先级 / 栈 |
| --- | --- | --- |
| BLE RX | UART RX 中断收字节，行结束唤醒 | High3 / 1536 B |
| 位姿、路径跟踪、双轮速度环 | 同一 CTRL 任务按 10 ms 顺序执行，分别计时 | High2 / 3072 B |
| IMU | TIM2 每 10 ms 释放二值信号量 | High1 / 1536 B |
| 航点生成 | 从 `Q_Cmd` 取目标或停止 | AboveNormal / 2048 B |
| LiDAR | DMA HT/TC 写 `Q_Lidar` | Normal / 2048 B |
| BLE TX | 50 ms | BelowNormal1 / 3072 B |
| OLED | 50 ms，每次更新 32 列，单次持锁 | BelowNormal / 1536 B |

- `Q_Lidar` 容量 4，传半缓冲描述符。任务在 DMA 覆盖前拷贝，缓冲过期或队满会计数。雷达启动最多发两次 SCAN，UART 重接收按退避处理。
- `Q_Cmd` 容量 8，传目标和 STOP。 `Q_Path` 容量 2，每条传一份完整航点序列，最多 64 点。命令和路径拥塞时清过期项，保留当前请求。
- IMU 和 OLED 使用 `osMutexPrioInherit` 保护 I²C。锁持有到阻塞式 I²C 事务返回。 `T` 发每阶段激活次数、最近间隔和最大耗时，`D` 发丢弃和 I²C 诊断。TIM5 提供 1 MHz 计时。

车头朝 +y，车辆从物理起点 `(0,0)` 静止复位。向右是 +x，航向从 +x 逆时针量，初始 90°。航点约每 0.10 m 一个，图示左右支路经过 `(0,0.70)`。发一次指令，不追加手动转向。

| 试跑 | 单条命令 | 现场记录 |
| --- | --- | --- |
| 直行 | `GOTO 0 1.40` | 实测终点位置、朝向、偏差 |
| 左支路 | `GOTO -0.70 0.70` | 实测终点位置、朝向、偏差 |
| 第三点 | 正文 `GOTO 0.70 -0.70` 或插图 `GOTO 0.70 0.70` | 先向 TA 确认。正文坐标为直线路径，场地须有对应通道。 |

每次保存网页 CSV，再用尺量实际终点和朝向。网页根据回传位姿计算估计误差。课件的 ICP、OGM 和 A* 是可选后续内容。队友网页的点云和网格显示仍可用。

## 本次验证

- Arm GNU 成功编译 Bench 和实测参数 Drive，方向保护版已烧录并校验。原始回波重放确认 58.5° 的 347 mm 回波不会触发前进停车。模拟验证了前方障碍停车、倒车方向保护、近距转向保护、限速、PWM 归零和停车锁存。静态实板读数显示前方走廊畅通、后方保护区有近障。电机断电时发 `GOTO 0 0.4`，固件进入控制，最终报双轮编码器无反馈（故障 7）。带动力停车距离需在实车上测量。
- 实板台架日志对应前一版 **Bench**，其电机输出由固件固定为零。当前正式 Drive 的文件名是 `Block45_Part2.hex`。
- 实板经 COM4 接收四种目标（含第三点的两个版本）并回传完整航点。无效命令被忽略，STOP 回报已停止。此次周期采样为 IMU 10.000 ms、位姿 9.993 ms、跟踪/电机 10.003 ms、OLED 49.615 ms、遥测 50.000 ms。
- 在随后 16 秒内，收到 427 条完整点云传输行和 5124 个有效距离值，IMU 持续返回校准后的数据。COM4 数据反映采集稳定，雷达转速和持续旋转尚未直接测量。
- 上位机 12 项测试通过，覆盖协议、控制诊断、标定后点云方向和 CSV 导出字段对齐。2026 年 11 月 18 日的架空记录包含有效轮速反馈、到达后的主动制动及轮速归零，用户确认两轮转动且 STOP 有效。运行中障碍触发和落地直线偏移仍需分别测试。迷宫试跑需要记录三次单条蓝牙指令的完整遥测及实测终点和航向误差。

如果烧录 Drive 后网页已显示驱动状态，OLED 却停在 `RTOS BOOT`，使用本目录生成的最新版 Drive 固件。OLED 运行时写完地址后会等待 I²C 的 STOP 完成，再发送显示数据。OLED 共 8 行，每轮全屏约需 1.6 秒。修复版烧录后，ST-LINK 热连接实测写入计数从 2519 增至 2563，两次读取的失败计数均为 0。

原始台架日志和 Flash 备份随 2026-11-18 的交接包保存。
