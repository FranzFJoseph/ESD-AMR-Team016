"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");
const vm = require("node:vm");
const { lineReceiver, parse, targetCommand } = require("./part2_protocol.js");

function dashboard() {
  const elements = new Map();
  const downloads = [];
  function element() {
    return {
      value: "", textContent: "", className: "", disabled: false,
      classList: { add() {}, remove() {}, toggle() {} }, addEventListener() {},
      insertRow: element, insertCell: element, click() { this.onclick?.(); },
    };
  }
  const document = {
    getElementById(id) {
      if (!elements.has(id)) elements.set(id, element());
      return elements.get(id);
    },
    createElement: element,
  };
  let now = 100;
  const sandbox = {
    document, performance: { now() { return now; } }, TextEncoder, Blob,
    URL: {
      createObjectURL(blob) { downloads.push(blob); return "blob:test-download"; },
      revokeObjectURL() {},
    },
    window: { addEventListener() {} }, navigator: {}, requestAnimationFrame() {},
    setTimeout() {}, setInterval() {}, clearInterval() {},
  };
  vm.createContext(sandbox);
  vm.runInContext(fs.readFileSync(path.join(__dirname, "part2_protocol.js"), "utf8"), sandbox);
  const html = fs.readFileSync(path.join(__dirname, "map.html"), "utf8");
  vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], sandbox);
  return {
    run: script => vm.runInContext(script, sandbox),
    text: id => document.getElementById(id).textContent,
    downloads,
    setTime: time => { now = time; },
  };
}

test("GOTO formats assessment targets and fits a single default BLE payload", () => {
  assert.equal(targetCommand("0.00", "1.40"), "GOTO 0 1.4\r\n");
  assert.equal(targetCommand("-0.70", "0.70"), "GOTO -0.7 0.7\r\n");
  assert.equal(targetCommand("0.70", "-0.70"), "GOTO 0.7 -0.7\r\n");
  assert.equal(targetCommand("-2.999", "-2.999").length, 20);
  assert.throws(() => targetCommand("3.01", "0"));
  assert.throws(() => targetCommand("0\nSTOP", "0"));
  assert.throws(() => targetCommand("Infinity", "0"));
  assert.throws(() => targetCommand("", "0"));
});

test("navigation target, waypoint and diagnostics lines retain exact signed units", () => {
  assert.deepEqual(parse("N 1 2 700 -700 19 19"), {
    kind: "N", run: 1, state: 2, x: .7, y: -.7, index: 19, total: 19,
  });
  assert.deepEqual(parse("W 1 12 -700 -1000"), { kind: "W", run: 1, index: 12, x: -.7, y: -1 });
  assert.equal(parse("N 0 5 0 0 0 0").state, 5);
  assert.equal(parse("N 1 6 0 1400 0 15").state, 6);
  assert.deepEqual(parse("T imu 502 10007 257"), { kind: "T", task: "imu", count: 502, periodUs: 10007, maxUs: 257 });
  assert.deepEqual(parse("D 1 2 3 4 5 6 7"), { kind: "D", values: [1, 2, 3, 4, 5, 6, 7] });
  for (const line of ["T typo 2 0 0", "W 1 -2 0 0", "N 1 2 0 0 4 3", "D 0 0 0 0 0 -1 0", "T imu 1.5 1 1"]) {
    assert.equal(parse(line), null);
  }
});

test("BLE serial reassembly handles partial CRLF and graceful overflow recovery", () => {
  const received = [];
  const feed = lineReceiver(line => received.push(line));
  feed("P -2");
  feed(" 3 4\r\nN 1");
  feed(" 2 700 -700 19 19\r");
  feed("\n" + "z".repeat(600) + "partial");
  feed("\r\nT pose 7 10000 84");
  feed("\n");
  assert.deepEqual(received, ["P -2 3 4", "N 1 2 700 -700 19 19", "T pose 7 10000 84"]);
});

test("drive diagnostics preserve signed ticks and PWM and validate profile flags", () => {
  assert.deepEqual(parse("A 17 1 6 1 -121 14 -432 873"), {
    kind: "A", run: 17, drive: 1, fault: 6, imuReady: 1,
    leftTicks: -121, rightTicks: 14, leftPwm: -432, rightPwm: 873,
  });
  assert.equal(parse("A 1 0 0 0 0 0 0 0").drive, 0);
  assert.equal(parse("A 1 2 0 0 0 0 0 0"), null);
  assert.equal(parse("A 1 1 0 -1 0 0 0 0"), null);
  assert.equal(parse("A 1 1 0 1 0 0 0"), null);
});

test("LiDAR stops retain a terminal navigation state and distinct causes", () => {
  const { lidarStopState, lidarStops } = require("./part2_protocol.js");
  assert.equal(parse("N 17 7 0 1400 5 14").state, lidarStopState);
  assert.equal(parse("N 17 8 0 1400 5 14"), null);
  for (const reason of [8, 9]) {
    const report = parse("A 17 1 " + reason + " 1 222 220 0 0");
    assert.equal(report.fault, reason);
    assert.match(lidarStops[report.fault].hint, /重新发送行驶指令/);
  }
  assert.match(lidarStops[8].title, /近处障碍/);
  assert.match(lidarStops[8].hint, /车身周围/);
  assert.match(lidarStops[9].title, /过期/);
});

test("obstacle telemetry distinguishes raw nearest and latched stop measurements", () => {
  assert.deepEqual(parse("O 1 0 517 247"), { kind: "O", fresh: 1, blocked: 0, nearestMm: 517, stopMm: 247 });
  assert.deepEqual(parse("O 0 0 0 0"), { kind: "O", fresh: 0, blocked: 0, nearestMm: 0, stopMm: 0 });
  assert.equal(parse("O 1 2 300 300"), null);
  assert.equal(parse("O 1 1 -300 300"), null);
  assert.equal(parse("O 1 1 300"), null);
});

test("extended obstacle telemetry keeps a zero bearing valid and aligns with the measured front", () => {
  const { lidarMotions, relativeBearingQ6 } = require("./part2_protocol.js");
  assert.deepEqual(parse("O 1 1 344 250 0 640 2 23039"), {
    kind: "O", fresh: 1, blocked: 1, nearestMm: 344, stopMm: 250,
    nearestQ6: 0, stopQ6: 640, motion: 2, frontQ6: 23039,
  });
  assert.equal(parse("O 0 0 0 0 -1 -1 0 -1").frontQ6, -1);
  assert.equal(lidarMotions[2], "倒车");
  assert.equal(relativeBearingQ6(0, 23039), 1);
  assert.equal(relativeBearingQ6(23039, 0), 23039);
  assert.equal(relativeBearingQ6(0, -1), null);
  assert.equal(relativeBearingQ6(0, undefined), null);
  assert.equal(parse("O 1 1 344 250 23040 640 2 0"), null);
  assert.equal(parse("O 1 1 344 250 0 -2 2 0"), null);
  assert.equal(parse("O 1 1 344 250 0 640 5 0"), null);
  assert.equal(parse("O 1 1 344 250 0"), null);
});

test("control telemetry keeps signed wheel speeds, headings and bias in raw units", () => {
  assert.deepEqual(parse("C 17 -120 80 -104 67 -1799 1784 -123 1"), {
    kind: "C", run: 17, leftRef: -120, rightRef: 80, leftActual: -104, rightActual: 67,
    gyroYaw10: -1799, fusedYaw10: 1784, biasMdps: -123, brake: 1,
  });
  assert.equal(parse("C 0 0 0 0 0 0 0 0 0").brake, 0);
  assert.equal(parse("C -1 0 0 0 0 0 0 0 0"), null);
  assert.equal(parse("C 1 0 0 0 0 0 0 0 2"), null);
  assert.equal(parse("C 1 0 0 0 0 0 0 0"), null);
  assert.equal(parse("C 1 0.1 0 0 0 0 0 0 0"), null);
});

test("control dashboard matches runs, marks brake output, expires data, and records CSV", () => {
  const { run, text, setTime } = dashboard();
  run("st.rec=true; feedLine('N 17 1 0 1400 1 14'); feedLine('A 17 1 0 1 50 50 0 0');");
  run("feedLine('C 16 0 0 0 0 0 0 0 1');");
  assert.match(text("dControlReadings"), /等待本次行驶/);
  assert.doesNotMatch(text("dDriveReadings"), /主动制动/);
  run("feedLine('C 17 0 0 42 -17 903 899 -123 1');");
  assert.match(text("dControlReadings"), /目标 0 \/ 0 mm\/s，实测 42 \/ -17 mm\/s/);
  assert.match(text("dControlReadings"), /90.3°，融合航向 89.9°.*-0.123°\/s.*主动制动/);
  assert.match(text("dDriveReadings"), /驱动指令 PWM L\/R \(‰\) 0 \/ 0/);
  assert.match(text("dControlReadings"), /主动制动，电桥双输入高电平/);
  const rows = run("st.recRows.map(row=>row.split(','))");
  const sample = rows.find(row => row[2] === "control" && row[6] === "17");
  assert.deepEqual(Array.from(sample.slice(-8)), ["0", "0", "42", "-17", "903", "899", "-123", "1"]);
  assert.equal(sample.length, 46);
  run("feedLine('N 18 1 700 700 0 14');");
  assert.match(text("dControlReadings"), /等待本次行驶/);
  run("feedLine('C 18 140 160 138 159 900 902 12 0');");
  assert.match(text("dControlReadings"), /目标 140 \/ 160/);
  assert.doesNotMatch(text("dDriveReadings"), /主动制动/);
  setTime(4000);
  run("updateControlStatus(); updateDriveStatus();");
  assert.match(text("dControlReadings"), /过期/);
  assert.doesNotMatch(text("dDriveReadings"), /主动制动/);
  run("resetNavigation();");
  assert.equal(text("dControlReadings"), "等待轮速与航向诊断");
});

test("world map projects native clockwise bearings through measured front and CCW heading", () => {
  const { run } = dashboard();
  run("feedLine('P 0 0 900');feedLine('O 1 0 1000 0 11072 -1 1 11072');");
  const checkPoint = (bin, x, y) => {
    const point = run("st.pts=[];st.bins.fill(0);st.bins[" + bin + "]=1000;onSweepDone();st.pts[0]");
    // The 5-degree transmitted bins use their centres, at 172.5, 262.5 and 82.5 degrees.
    assert(Math.abs(point.x - x) < 0.01, "world X for bin " + bin);
    assert(Math.abs(point.y - y) < 0.01, "world Y for bin " + bin);
  };
  checkPoint(34, 0, 1);
  checkPoint(52, 1, 0);
  checkPoint(16, -1, 0);

  run("feedLine('O 1 0 1000 0');");
  const legacy = run("st.pts=[];st.bins.fill(0);st.bins[0]=1000;onSweepDone();st.pts[0]");
  assert(Math.abs(legacy.x + Math.sin(2.5 * Math.PI / 180)) < 1e-9);
  assert(Math.abs(legacy.y - Math.cos(2.5 * Math.PI / 180)) < 1e-9);
});

test("front alignment changes clear projected points and grid while preserving navigation", () => {
  const { run } = dashboard();
  run("feedLine('N 2 1 0 1400 2 14'); feedLine('W 2 0 0 100');feedLine('P 0 200 900');");
  const seed = "st.pts=[{x:3,y:4,t:100}];st.grid.set('3,4',2);st.prevPose={x:3,y:4,th:5};";
  run(seed + "feedLine('O 1 0 1000 0 11072 -1 1 11072');");
  assert.equal(run("st.pts.length"), 0);
  assert.equal(run("st.grid.size"), 0);
  assert.equal(run("st.prevPose"), null);
  assert.equal(run("st.pose.y"), 0.2);
  assert.equal(run("st.nav.run"), 2);
  assert.equal(run("st.waypoints.size"), 1);
  assert.equal(run("st.trail.length"), 1);
  run(seed + "feedLine('O 1 0 1000 0 11072 -1 1 11072');");
  assert.equal(run("st.pts.length"), 1);
  assert.equal(run("st.grid.size"), 1);
  run("feedLine('O 1 0 1000 0 11072 -1 1 11136');");
  assert.equal(run("st.pts.length"), 0);
  assert.equal(run("st.grid.size"), 0);
  run(seed + "feedLine('O 1 0 1000 0 11072 -1 1 -1');");
  assert.equal(run("st.pts.length"), 0);
  assert.equal(run("st.grid.size"), 0);
});

test("downloaded CSV maps every diagnostic and control value under its actual header", async () => {
  const { run, downloads } = dashboard();
  run("document.getElementById('btnRec').click();");
  run("feedLine('N 17 1 0 1400 2 14');");
  run("feedLine('P 124 -211 903');");
  run("feedLine('T imu 102 10002 207');");
  run("feedLine('D 11 12 13 14 15 16 17');");
  run("feedLine('A 17 1 6 1 -123 456 -321 654');");
  run("feedLine('O 1 0 344 247 11072 11520 2 11072');");
  run("feedLine('C 17 141 162 133 157 903 899 -123 1');");
  run("recordRow('command', [...Array(CSV_TASK_OFFSET).fill(''), 'GOTO 0 1.4']);");
  run("document.getElementById('btnRec').click();");
  assert.equal(downloads.length, 1);
  const [header, ...lines] = (await downloads[0].text()).trimEnd().split("\n").map(line => line.split(","));
  assert.equal(header.length, 46);
  assert.equal(new Set(header).size, header.length);
  const rows = lines.map(values => {
    assert.equal(values.length, header.length);
    return Object.fromEntries(header.map((field, index) => [field, values[index]]));
  });
  const byKind = kind => rows.find(row => row.kind === kind);
  const selected = (kind, keys) => Object.fromEntries(keys.map(key => [key, byKind(kind)[key]]));
  assert.deepEqual(selected("timing", ["total", "task_or_command", "count", "period_us", "max_us", "lidar_drop"]),
    { total: "", task_or_command: "imu", count: "102", period_us: "10002", max_us: "207", lidar_drop: "" });
  assert.deepEqual(selected("diagnostics", ["max_us", "lidar_drop", "cmd_drop", "path_drop", "imu_sem_full",
      "imu_errors", "i2c_wait_max_us", "rx_drop", "drive_enabled"]), {
    max_us: "", lidar_drop: "11", cmd_drop: "12", path_drop: "13", imu_sem_full: "14",
    imu_errors: "15", i2c_wait_max_us: "16", rx_drop: "17", drive_enabled: "",
  });
  assert.deepEqual(selected("drive", ["run", "rx_drop", "drive_enabled", "fault_code", "imu_ready", "left_ticks",
      "right_ticks", "left_pwm", "right_pwm", "lidar_fresh"]), {
    run: "17", rx_drop: "", drive_enabled: "1", fault_code: "6", imu_ready: "1",
    left_ticks: "-123", right_ticks: "456", left_pwm: "-321", right_pwm: "654", lidar_fresh: "",
  });
  assert.deepEqual(selected("control", ["run", "front_q6", "left_ref_mm_s", "right_ref_mm_s", "left_actual_mm_s",
      "right_actual_mm_s", "gyro_yaw10", "fused_yaw10", "bias_mdps", "brake_active"]), {
    run: "17", front_q6: "", left_ref_mm_s: "141", right_ref_mm_s: "162", left_actual_mm_s: "133",
    right_actual_mm_s: "157", gyro_yaw10: "903", fused_yaw10: "899", bias_mdps: "-123", brake_active: "1",
  });
  assert.deepEqual(selected("obstacle", ["run", "right_pwm", "lidar_fresh", "lidar_blocked", "nearest_mm",
      "stop_mm", "nearest_q6", "stop_q6", "motion", "front_q6", "left_ref_mm_s"]), {
    run: "17", right_pwm: "", lidar_fresh: "1", lidar_blocked: "0", nearest_mm: "344",
    stop_mm: "247", nearest_q6: "11072", stop_q6: "11520", motion: "2", front_q6: "11072", left_ref_mm_s: "",
  });
  assert.equal(byKind("command").total, "");
  assert.equal(byKind("command").task_or_command, "GOTO 0 1.4");
  assert.equal(byKind("nav").total, "14");
  assert.equal(byKind("pose").x_m, "0.124");
});
