[CmdletBinding()]
param(
    [string] $ToolchainBin,
    [switch] $EnableMotors,
    [switch] $EnableAvoidance,
    [double] $WheelMm,
    [double] $TrackMm,
    [int] $LeftCpr,
    [int] $RightCpr,
    [ValidateSet(-1, 1)][int] $LeftEncoderSign = 1,
    [ValidateSet(-1, 1)][int] $RightEncoderSign = -1,
    [ValidateSet(-1, 1)][int] $LeftMotorSign = -1,
    [ValidateSet(-1, 1)][int] $RightMotorSign = 1,
    [ValidateSet(-1, 1)][int] $ImuSign = 1,
    [double] $LidarFrontDeg = [double]::NaN
)
$ErrorActionPreference = 'Stop'
if ($EnableMotors -and (
        [double]::IsNaN($WheelMm) -or $WheelMm -lt 10 -or $WheelMm -gt 300 -or
        [double]::IsNaN($TrackMm) -or $TrackMm -lt 50 -or $TrackMm -gt 1000 -or
        $LeftCpr -le 0 -or $RightCpr -le 0)) {
    throw 'Supply measured wheel diameter, track width (mm), and positive encoder x4 counts per revolution.'
}
if ($PSBoundParameters.ContainsKey('LidarFrontDeg') -and
    ([double]::IsNaN($LidarFrontDeg) -or $LidarFrontDeg -lt 0 -or $LidarFrontDeg -ge 360)) {
    throw 'Supply the measured raw LiDAR angle of the chassis front in [0, 360) degrees.'
}
if (-not $ToolchainBin) {
    $compiler = Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
    if ($compiler) {
        $ToolchainBin = Split-Path -Parent $compiler.Source
    } else {
        $pattern = "$env:SystemDrive/ST/STM32CubeIDE*/STM32CubeIDE/plugins/*externaltools.gnu-tools-for-stm32*/tools/bin/arm-none-eabi-gcc.exe"
        $compiler = Get-ChildItem -Path $pattern -File | Sort-Object FullName -Descending | Select-Object -First 1
        if ($compiler) { $ToolchainBin = $compiler.DirectoryName }
    }
}
if (-not $ToolchainBin) { throw 'Install Arm GNU Toolchain or specify -ToolchainBin.' }
$ToolchainBin = (Resolve-Path -LiteralPath $ToolchainBin).Path
function Invoke-Tool([string] $name, [string[]] $arguments) {
    & (Join-Path $ToolchainBin "arm-none-eabi-$name.exe") @arguments
    if ($LASTEXITCODE -ne 0) { throw "$name exited with code $LASTEXITCODE." }
}
Push-Location -LiteralPath $PSScriptRoot
try {
    $profile = if ($EnableMotors) { 'Drive' } else { 'Bench' }
    $out = "Build/$profile"
    New-Item -ItemType Directory -Path "$out/obj" -Force | Out-Null
    $rtos = 'Middlewares/Third_Party/FreeRTOS/Source'
    $hal = 'Drivers/STM32F4xx_HAL_Driver'
    $arch = @('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard')
    $flags = $arch + @('-Og','-g3','-ffunction-sections','-fdata-sections',
        '-DSTM32F446xx','-DUSE_HAL_DRIVER','-ICore/Inc',
        '-isystem',"$hal/Inc",'-isystem','Drivers/CMSIS/Include',
        '-isystem','Drivers/CMSIS/Device/ST/STM32F4xx/Include',
        '-isystem',"$rtos/include",'-isystem',"$rtos/CMSIS_RTOS_V2",
        '-isystem',"$rtos/portable/GCC/ARM_CM4F")
    if ($PSBoundParameters.ContainsKey('LidarFrontDeg')) {
        $frontQ6 = [int][Math]::Round(64 * $LidarFrontDeg) % 23040
        $flags += "-DPART2_LIDAR_FRONT_Q6=$frontQ6"
    }
    if ($EnableMotors) {
        $culture = [Globalization.CultureInfo]::InvariantCulture
        $wheelValue = $WheelMm.ToString('0.0000', $culture)
        $trackValue = $TrackMm.ToString('0.0000', $culture)
        if ($EnableAvoidance) { $flags += '-DPART2_ENABLE_AVOIDANCE=1' }
        $flags += @('-DPART2_ENABLE_MOTORS=1',
            "-DPART2_WHEEL_MM=${wheelValue}f", "-DPART2_TRACK_MM=${trackValue}f",
            "-DPART2_CPR_L=$LeftCpr", "-DPART2_CPR_R=$RightCpr",
            "-DPART2_ENCODER_L=$LeftEncoderSign", "-DPART2_ENCODER_R=$RightEncoderSign",
            "-DPART2_MOTOR_L=$LeftMotorSign", "-DPART2_MOTOR_R=$RightMotorSign",
            "-DPART2_IMU_SIGN=$ImuSign")
    }
    $sources = @(Get-ChildItem -Path 'Core/Src/*.c','Core/Startup/*.s',"$hal/Src/*.c","$rtos/*.c")
    $sources += @(Get-Item -LiteralPath "$rtos/CMSIS_RTOS_V2/cmsis_os2.c","$rtos/portable/GCC/ARM_CM4F/port.c","$rtos/portable/MemMang/heap_4.c")
    $objects = [System.Collections.Generic.List[string]]::new()
    foreach ($source in $sources) {
        $relative = (Resolve-Path -LiteralPath $source.FullName -Relative).Replace('\','/')
        $object = "$out/obj/$($source.BaseName).o"
        $cFlags = @()
        if ($source.Extension -eq '.c') { $cFlags = @('-std=gnu11','-Werror=implicit-function-declaration') }
        if ($source.Name -in 'main.c','nav.c','commands.c','imu_heading.c') { $cFlags += @('-Wall','-Wextra','-Werror') }
        Invoke-Tool 'gcc' ($flags + $cFlags + @('-c',$relative,'-o',$object))
        $objects.Add($object)
    }
    $base = "$out/Block45_Part2"
    Invoke-Tool 'gcc' ($arch + @('--specs=nano.specs','--specs=nosys.specs',
        '-TSTM32F446RETX_FLASH.ld',"-Wl,--gc-sections,-Map=$base.map",
        '-o',"$base.elf") + $objects + @('-lm'))
    Invoke-Tool 'objcopy' @('-O','ihex',"$base.elf","$base.hex")
    Invoke-Tool 'size' @("$base.elf")
}
finally { Pop-Location }
