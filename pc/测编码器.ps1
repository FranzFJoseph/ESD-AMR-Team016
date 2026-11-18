[CmdletBinding()]
param(
    [string]$Port = 'COM4',
    [ValidateSet('Both', 'Left', 'Right')][string]$Wheel = 'Both'
)

$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, 'None', 8, 'One')
$serial.ReadTimeout = 800

function Read-EncoderCounts {
    # RT arrives at 1 Hz on the original Bench firmware.
    Start-Sleep -Milliseconds 1200
    $serial.DiscardInBuffer()
    $elapsed = [Diagnostics.Stopwatch]::StartNew()
    $previous = $null
    $same = 0
    while ($elapsed.Elapsed.TotalSeconds -lt 8) {
        try {
            $line = $serial.ReadLine().Trim()
        } catch [System.TimeoutException] {
            continue
        }
        if ($line -match '^A\s+\d+\s+([01])\s+\d+\s+[01]\s+(-?\d+)\s+(-?\d+)\s+-?\d+\s+-?\d+$') {
            if ($Matches[1] -eq '1') { throw '检测到 Drive 固件。请用电机输出禁用的 Bench 测量。' }
            $sample = [pscustomobject]@{ Left = [int]$Matches[2]; Right = [int]$Matches[3]; Frame = 'A' }
        } elseif ($line -match '^RT\s+.*\beL=(-?\d+)\s+eR=(-?\d+)\b') {
            # The previous Bench emits right counts after applying ENC_R_DIR=-1.
            $sample = [pscustomobject]@{ Left = [int]$Matches[1]; Right = -[int]$Matches[2]; Frame = 'RT' }
        } else {
            continue
        }
        if ($previous -and $sample.Frame -eq 'RT' -and $previous.Frame -eq 'A') { continue }
        if ($previous -and $sample.Frame -eq $previous.Frame -and
            $sample.Left -eq $previous.Left -and $sample.Right -eq $previous.Right) {
            ++$same
        } else {
            $same = 1
        }
        $previous = $sample
        if ($same -eq 3) { return $sample }
    }
    if ($previous) {
        throw "在 $Port 收到编码器计数，但两轮静止时读数未稳定。最后一帧：左 $($previous.Left)，右 $($previous.Right)。"
    }
    throw "没有在 $Port 收到编码器计数。核对 ST-LINK COM 号并关闭其他串口工具。"
}

function Get-CountChange([int]$Before, [int]$After) {
    return (($After - $Before + 32768) -band 65535) - 32768
}

function Measure-Wheel ([string]$Name, [string]$Side, [string]$OtherSide) {
    for ($attempt = 1; $attempt -le 2; ++$attempt) {
        Read-Host "准备测$($Name)（第 $attempt 次）。两轮静止后按回车，等下一条提示" | Out-Null
        $before = Read-EncoderCounts
        Write-Host "起始计数  左 $($before.Left)  右 $($before.Right)"
        Read-Host "只把$($Name)沿小车前进方向手转一整圈，另一只轮保持不动。停稳后按回车，再保持两轮静止直到显示结果" | Out-Null
        $after = Read-EncoderCounts
        $delta = Get-CountChange $before.$Side $after.$Side
        $other = Get-CountChange $before.$OtherSide $after.$OtherSide
        Write-Host "结束计数  左 $($after.Left)  右 $($after.Right)"
        Write-Host "$($Name)计数变化 $delta，另一轮变化 $other"
        if ($delta -ne 0 -and [Math]::Abs($other) -le 2) { return $delta }
        if ($delta -eq 0 -and [Math]::Abs($other) -gt 2) {
            Write-Host '只有另一侧计数变化。核对左右 A/B 接头与软件通道是否一致。'
        } elseif ($delta -eq 0) {
            Write-Host '本侧没有计数。检查该编码器是否供电、共地，以及 A/B 两相是否接通。'
        } else {
            Write-Host '两侧都出现计数变化。确认另一轮没有动，检查两组 A/B 接线是否串接。'
        }
    }
    return $null
}

try {
    $serial.Open()
    Write-Host '先确认板上是 Bench，架空小车，电机输出与电机隔离，并保留编码器供电。'
    if ($Wheel -ne 'Right') { $left = Measure-Wheel '左轮' 'Left' 'Right' }
    if ($Wheel -ne 'Left') { $right = Measure-Wheel '右轮' 'Right' 'Left' }
    if (($Wheel -ne 'Right' -and $null -eq $left) -or ($Wheel -ne 'Left' -and $null -eq $right)) {
        Write-Host '测量未通过。请保留本次输出，先核对计数异常的一侧。'
    } elseif ($Wheel -eq 'Left') {
        "左轮测量通过：$left（每圈 x4，保留正负号）"
    } elseif ($Wheel -eq 'Right') {
        "右轮测量通过：$right（每圈 x4，保留正负号）"
    } else {
        "测量通过：左轮 $left，右轮 $right（每圈 x4，保留正负号）"
    }
} finally {
    $serial.Dispose()
}
