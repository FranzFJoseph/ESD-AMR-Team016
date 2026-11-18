"use strict";

// The robot's text protocol crosses BLE notification boundaries.
const Part2Protocol = (() => {
  const timingTasks = ["imu", "lidar", "rx", "plan", "pose", "track", "motor", "oled", "tx"];
  const diagnosticFields = ["lidar_drop", "cmd_drop", "path_drop", "imu_sem_full",
                            "imu_errors", "i2c_wait_max_us", "rx_drop"];
  const driveFields = ["drive_enabled", "fault_code", "imu_ready", "left_ticks", "right_ticks",
                       "left_pwm", "right_pwm"];
  const obstacleFields = ["lidar_fresh", "lidar_blocked", "nearest_mm", "stop_mm",
                          "nearest_q6", "stop_q6", "motion", "front_q6"];
  const controlFields = ["left_ref_mm_s", "right_ref_mm_s", "left_actual_mm_s", "right_actual_mm_s",
                         "gyro_yaw10", "fused_yaw10", "bias_mdps", "brake_active"];
  const lidarFullTurnQ6 = 360 * 64;
  const lidarMotions = ["停止或方向未报", "前进", "倒车", "原地左转", "原地右转"];
  const lidarStopState = 7;
  const lidarStops = {
    8: { title: "近处障碍停车", hint: "清除车身周围的障碍物，确认路线安全后重新发送行驶指令。" },
    9: { title: "LiDAR 数据过期停车", hint: "检查雷达供电、接线和点云，数据恢复后重新发送行驶指令。" },
  };
  const integer = /^-?\d+$/;
  const decimal = /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)$/;

  function integers(tokens, expected) {
    if (tokens.length !== expected || tokens.some(t => !integer.test(t))) return null;
    const values = tokens.map(Number);
    return values.every(Number.isSafeInteger) ? values : null;
  }

  function parse(line) {
    const [kind, ...tokens] = line.trim().split(/\s+/);
    if (kind === "N") {
      const n = integers(tokens, 6);
      if (!n) return null;
      const [run, state, x, y, index, total] = n;
      if (run < 0 || state < 0 || state > lidarStopState || index < 0 || index > total || total > 255) return null;
      return { kind, run, state, x: x / 1000, y: y / 1000, index, total };
    }
    if (kind === "W") {
      const n = integers(tokens, 4);
      if (!n) return null;
      const [run, index, x, y] = n;
      if (run < 0 || index < 0 || index >= 255) return null;
      return { kind, run, index, x: x / 1000, y: y / 1000 };
    }
    if (kind === "T") {
      const [task, ...args] = tokens;
      const n = integers(args, 3);
      if (!timingTasks.includes(task) || !n || n.some(value => value < 0)) return null;
      const [count, periodUs, maxUs] = n;
      return { kind, task, count, periodUs, maxUs };
    }
    if (kind === "D") {
      const n = integers(tokens, diagnosticFields.length);
      if (!n || n.some(value => value < 0)) return null;
      return { kind, values: n };
    }
    if (kind === "A") {
      const n = integers(tokens, 8);
      if (!n) return null;
      const [run, drive, fault, imuReady, leftTicks, rightTicks, leftPwm, rightPwm] = n;
      if (run < 0 || ![0, 1].includes(drive) || fault < 0 || fault > 255 ||
          ![0, 1].includes(imuReady)) return null;
      return { kind, run, drive, fault, imuReady, leftTicks, rightTicks, leftPwm, rightPwm };
    }
    if (kind === "O") {
      const extended = tokens.length === obstacleFields.length;
      const n = integers(tokens, extended ? obstacleFields.length : 4);
      if (!n) return null;
      const [fresh, blocked, nearestMm, stopMm] = n;
      if (![0, 1].includes(fresh) || ![0, 1].includes(blocked) || nearestMm < 0 || stopMm < 0) return null;
      const result = { kind, fresh, blocked, nearestMm, stopMm };
      if (!extended) return result;
      const [nearestQ6, stopQ6, motion, frontQ6] = n.slice(4);
      if ([nearestQ6, stopQ6, frontQ6].some(angle => angle < -1 || angle >= lidarFullTurnQ6) ||
          motion < 0 || motion >= lidarMotions.length) return null;
      return { ...result, nearestQ6, stopQ6, motion, frontQ6 };
    }
    if (kind === "C") {
      const n = integers(tokens, 9);
      if (!n) return null;
      const [run, leftRef, rightRef, leftActual, rightActual, gyroYaw10, fusedYaw10, biasMdps, brake] = n;
      if (run < 0 || ![0, 1].includes(brake)) return null;
      return { kind, run, leftRef, rightRef, leftActual, rightActual, gyroYaw10, fusedYaw10, biasMdps, brake };
    }
    return null;
  }

  function relativeBearingQ6(rawQ6, frontQ6) {
    if (!(rawQ6 >= 0 && frontQ6 >= 0)) return null;
    return (rawQ6 - frontQ6 + lidarFullTurnQ6) % lidarFullTurnQ6;
  }

  function targetCommand(xText, yText) {
    const coordinates = [xText.trim(), yText.trim()];
    if (!coordinates.every(value => decimal.test(value))) throw new Error("请输入两个十进制坐标，单位为米。");
    const numbers = coordinates.map(Number);
    if (!numbers.every(value => Math.abs(value) <= 3)) throw new Error("坐标超出 ±3 m。");
    const [x, y] = numbers.map(value => Number(value.toFixed(3)));
    return `GOTO ${x} ${y}\r\n`;
  }

  function lineReceiver(receive) {
    let buffer = "";
    let overflow = false;
    return chunk => {
      for (const char of chunk) {
        if (char === "\r" || char === "\n") {
          if (buffer && !overflow) receive(buffer);
          buffer = "";
          overflow = false;
        } else if (!overflow) {
          buffer += char;
          if (buffer.length > 512) {
            buffer = "";
            overflow = true;
          }
        }
      }
    };
  }

  return { timingTasks, diagnosticFields, driveFields, obstacleFields, controlFields,
           lidarStopState, lidarStops, lidarMotions, relativeBearingQ6, parse, targetCommand, lineReceiver };
})();

if (typeof module === "object") module.exports = Part2Protocol;
