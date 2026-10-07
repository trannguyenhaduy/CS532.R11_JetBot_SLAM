#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
KIỂM THỬ ĐỘC LẬP TÍNH NĂNG PHANH KHẨN CẤP & CẢN ẢO (TEST_EMERGENCY_BRAKE.PY)
Dành riêng cho đồ án JetBot + Camera OAK-D S2
Dựa trên mạch Waveshare Motor HAT chuẩn của xe.

Cách chạy:
  python3 test_emergency_brake.py
Mở trình duyệt:
  http://192.168.1.13:8080 (hoặc http://localhost:8080)
══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import json
import threading
import numpy as np
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

# ─── 1. DRIVER ĐỘNG CƠ WAVESHARE MOTOR HAT CHUẨN XÁC ──────────────────────────
try:
    from Adafruit_MotorHAT import Adafruit_MotorHAT
    HAS_ADAFRUIT = True
except ImportError:
    HAS_ADAFRUIT = False

try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    try:
        from smbus import SMBus
        HAS_SMBUS = True
    except ImportError:
        HAS_SMBUS = False


class WaveshareMotorDriver:
    """Driver động cơ vi sai Waveshare chuẩn theo thanh ghi Adafruit_MotorHAT"""
    def __init__(self, addr=0x60, bus_num=1):
        self.is_connected = False
        self.driver_name = "SIMULATOR"
        self._driver = None
        self._left = None
        self._right = None
        self._pins = ((1, 0), (2, 3))

        if HAS_ADAFRUIT:
            try:
                self._hat_api = Adafruit_MotorHAT
                self._driver = Adafruit_MotorHAT(addr=addr, i2c_bus=bus_num)
                self._left = self._driver.getMotor(1)
                self._right = self._driver.getMotor(2)
                self.is_connected = True
                self.driver_name = "Adafruit_MotorHAT"
                self.stop()
                print(f"🤖 [MOTOR] Đã kết nối phần cứng Waveshare Motor HAT (0x{addr:02X})!")
            except Exception as e:
                print(f"⚠️ [MOTOR] Không khởi tạo được Adafruit_MotorHAT: {e}")

        if not self.is_connected:
            print("⚠️ [MOTOR] Đang chạy chế độ GIẢ LẬP (Bánh xe sẽ không quay thực tế)")

    def _set_one(self, motor, pins, value):
        if not self.is_connected or not self._driver: return
        value = max(-1.0, min(1.0, float(value)))
        mapped = int(255.0 * value)
        speed = min(max(abs(mapped), 0), 255)
        motor.setSpeed(speed)
        ina, inb = pins
        if speed < 15:
            motor.run(self._hat_api.RELEASE)
            self._driver._pwm.setPWM(ina, 0, 0)
            self._driver._pwm.setPWM(inb, 0, 0)
        elif mapped < 0:
            motor.run(self._hat_api.FORWARD)
            self._driver._pwm.setPWM(ina, 0, 0)
            self._driver._pwm.setPWM(inb, 0, speed * 16)
        else:
            motor.run(self._hat_api.BACKWARD)
            self._driver._pwm.setPWM(ina, 0, speed * 16)
            self._driver._pwm.setPWM(inb, 0, 0)

    def set_motors(self, left, right):
        # Bù ma sát tĩnh tối thiểu (Deadband boost nhẹ) để bánh xe lăn êm
        if abs(left) > 0.03 and abs(left) < 0.16:
            left = 0.16 if left > 0 else -0.16
        if abs(right) > 0.03 and abs(right) < 0.16:
            right = 0.16 if right > 0 else -0.16

        self._set_one(self._left, self._pins[0], left)
        self._set_one(self._right, self._pins[1], right)

    def stop(self):
        if not self.is_connected or not self._driver: return
        for motor, pins in ((self._left, self._pins[0]), (self._right, self._pins[1])):
            if motor: motor.run(self._hat_api.RELEASE)
            self._driver._pwm.setPWM(pins[0], 0, 0)
            self._driver._pwm.setPWM(pins[1], 0, 0)


# ─── 2. BIẾN TOÀN CỤC & TRẠNG THÁI HỆ THỐNG ──────────────────────────────────
robot = WaveshareMotorDriver()
running = True
last_drive_time = time.time()

current_left = 0.0
current_right = 0.0
forward_clearance_mm = 9999.0
brake_threshold_mm = 250.0     # 25cm = 250mm (Nâng lên 25cm để bù quán tính trôi khi dừng)
is_emergency_braked = False
brake_event_count = 0

latest_rgb_jpeg = None
latest_depth_jpeg = None


# ─── 3. LUỒNG CAMERA OAK-D S2 & TÍNH TOÁN KHOẢNG CÁCH THỜI GIAN THỰC ──────────
def oak_worker():
    global latest_rgb_jpeg, latest_depth_jpeg, running, forward_clearance_mm, is_emergency_braked

    try:
        import depthai as dai
    except ImportError:
        print("⚠️ [CAMERA] Không tìm thấy thư viện depthai. Chạy giả lập camera.")
        return

    print("🚀 [OAK-D] Đang khởi tạo Camera Stereo Depth OAK-D S2...")
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(480, 360)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(20)

    is_v3 = not hasattr(dai.node, 'XLinkOut')

    mono_l = pipeline.create(dai.node.MonoCamera)
    mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)

    mono_r = pipeline.create(dai.node.MonoCamera)
    mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True)  # Giảm điểm mù từ 35cm xuống 15cm

    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

    if is_v3:
        q_rgb = cam_rgb.preview.createOutputQueue(maxSize=2, blocking=False)
        q_depth = stereo.disparity.createOutputQueue(maxSize=2, blocking=False)
        q_raw_depth = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
    else:
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.disparity.link(xout_depth.input)

        xout_raw_depth = pipeline.create(dai.node.XLinkOut)
        xout_raw_depth.setStreamName("raw_depth")
        stereo.depth.link(xout_raw_depth.input)

    try:
        if is_v3:
            pipeline.start()
            print("✅ [OAK-D] Camera OAK-D S2 (DepthAI v3) đã sẵn sàng! Đang quét cự ly...")
        else:
            device = dai.Device(pipeline)
            q_rgb = device.getOutputQueue(name="rgb", maxSize=2, blocking=False)
            q_depth = device.getOutputQueue(name="depth", maxSize=2, blocking=False)
            q_raw_depth = device.getOutputQueue(name="raw_depth", maxSize=2, blocking=False)
            print("✅ [OAK-D] Camera OAK-D S2 (DepthAI v2) đã sẵn sàng! Đang quét cự ly...")

            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 70]

            while running:
                in_rgb = q_rgb.tryGet()
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                    # Thước đo cự ly trực tiếp lên hình ảnh
                    h, w = frame.shape[:2]

                    c_cm = forward_clearance_mm / 10.0
                    col = (0, 0, 255) if is_emergency_braked else ((0, 200, 255) if c_cm < 40 else (0, 255, 100))
                    stat_txt = f"BRAKE! {c_cm:.1f} cm" if is_emergency_braked else f"{c_cm:.1f} cm"
                    cv2.putText(frame, f"CLEARANCE: {stat_txt}", (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

                    ret, jpeg = cv2.imencode('.jpg', frame, encode_params)
                    if ret: latest_rgb_jpeg = jpeg.tobytes()

                in_depth = q_depth.tryGet()
                if in_depth is not None:
                    disp_norm = (in_depth.getCvFrame() * (255.0 / 95.0)).astype(np.uint8)
                    disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
                    disp_color = cv2.resize(disp_color, (480, 360))
                    ret, jpeg = cv2.imencode('.jpg', disp_color, encode_params)
                    if ret: latest_depth_jpeg = jpeg.tobytes()

                in_raw_depth = q_raw_depth.tryGet()
                if in_raw_depth is not None:
                    depth_data = in_raw_depth.getFrame()
                    dh, dw = depth_data.shape[:2]

                    # Vùng hành lang an toàn trung tâm (Bỏ qua sàn gạch h > 0.55dh và trần nhà h < 0.20dh)
                    h_start, h_end = int(dh * 0.20), int(dh * 0.55)
                    w_start, w_end = int(dw * 0.30), int(dw * 0.70)
                    roi = depth_data[h_start:h_end, w_start:w_end]

                    # Lọc các điểm đo vật lý hợp lệ từ 50mm đến 3500mm
                    valid_depths = roi[(roi >= 50) & (roi <= 3500)]
                    valid_count = len(valid_depths)

                    if valid_count >= 25:
                        forward_clearance_mm = float(np.percentile(valid_depths, 5))
                    elif forward_clearance_mm <= 550.0:
                        # Điểm mù stereo (< 18cm): Đang có vật cản áp sát mũi xe
                        forward_clearance_mm = 200.0
                    else:
                        forward_clearance_mm = 9999.0  # Đường thoáng, không có cản trước mặt

                time.sleep(0.01)
    except Exception as e:
        print(f"⚠️ [OAK-D] Lỗi kết nối camera: {e}")


# ─── 4. LUỒNG GIÁM SÁT AN TOÀN WATCHDOG (50 Hz) ───────────────────────────────
def watchdog_worker():
    global running, last_drive_time, current_left, current_right
    global forward_clearance_mm, brake_threshold_mm, is_emergency_braked, brake_event_count

    while running:
        # Kiểm tra điều kiện PHANH KHẨN CẤP khi xe đang tiến
        is_moving_forward = (current_left > 0.05 and current_right > 0.05)

        if forward_clearance_mm < brake_threshold_mm:
            is_emergency_braked = True
            if is_moving_forward:
                robot.stop()
                current_left = 0.0
                current_right = 0.0
                brake_event_count += 1
                c_cm = forward_clearance_mm / 10.0
                print(f"\n🚨 [PHANH KHẨN CẤP #{brake_event_count}] ĐÃ DỪNG XE! Vật cản cách {c_cm:.1f} cm (< {brake_threshold_mm/10.0:.0f} cm)")
        else:
            is_emergency_braked = False

        # Tự ngắt động cơ nếu mất lệnh lái quá 0.5s (Chống trôi xe)
        if time.time() - last_drive_time > 0.50:
            if current_left != 0.0 or current_right != 0.0:
                robot.stop()
                current_left = 0.0
                current_right = 0.0

        time.sleep(0.02)


# ─── 5. GIAO DIỆN WEB ĐIỀU KHIỂN & ĐO CỰ LY THỜI GIAN THỰC ────────────────────
HTML_PAGE = """<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>JetBot Emergency Brake Test</title>
<style>
  :root {
    --bg: #07090e; --card: #0d121d; --cyan: #00f0ff; --red: #ff2a6d; --green: #00ffa3; --amber: #ffb800;
  }
  * { box-sizing: border-box; margin:0; padding:0; font-family: -apple-system, sans-serif; }
  body { background: var(--bg); color: #fff; display: flex; flex-direction: column; align-items: center; min-height: 100vh; padding: 16px; }
  .header { width: 100%; max-width: 520px; display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
  h1 { font-size: 1.15rem; color: var(--cyan); letter-spacing: 1px; }
  .badge { padding: 4px 10px; border-radius: 12px; font-size: 0.78rem; font-weight: 700; border: 1px solid var(--green); color: var(--green); }
  .badge.alert { border-color: var(--red); color: var(--red); background: rgba(255,42,109,0.15); animation: pulse 0.6s infinite alternate; }
  @keyframes pulse { from { opacity: 0.7; } to { opacity: 1; box-shadow: 0 0 14px var(--red); } }

  .viewport { position: relative; width: 100%; max-width: 520px; height: 340px; background: #000; border-radius: 12px; overflow: hidden; border: 2px solid #1c2738; box-shadow: 0 8px 30px rgba(0,0,0,0.7); }
  .viewport img { width: 100%; height: 100%; object-fit: contain; }
  .hud-clearance { position: absolute; top: 12px; left: 14px; background: rgba(7,9,14,0.75); padding: 6px 12px; border-radius: 8px; border: 1px solid rgba(255,255,255,0.15); }
  .hud-val { font-size: 1.5rem; font-weight: 800; color: var(--cyan); }
  .hud-val.alert { color: var(--red); }

  .ctrl-card { width: 100%; max-width: 520px; background: var(--card); border-radius: 12px; border: 1px solid #1c2738; padding: 14px; margin-top: 14px; }
  .status-row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; font-size: 0.85rem; color: #94a3b8; }
  .status-val { font-weight: 700; color: #fff; }

  .dpad { display: grid; grid-template-columns: repeat(3, 70px); grid-gap: 8px; justify-content: center; margin: 12px 0; }
  .btn { height: 60px; background: #131d2e; border: 1px solid #1e2d44; color: var(--cyan); border-radius: 10px; font-size: 1.3rem; font-weight: bold; cursor: pointer; display: flex; align-items: center; justify-content: center; user-select: none; transition: 0.1s; }
  .btn:active, .btn.active { background: rgba(0,240,255,0.3); border-color: var(--cyan); transform: scale(0.96); }
  .btn-stop { color: var(--red); }
  .btn-stop:active, .btn-stop.active { background: rgba(255,42,109,0.3); border-color: var(--red); }

  .tip { text-align: center; font-size: 0.75rem; color: #64748b; margin-top: 8px; }
</style>
</head>
<body>

<div class="header">
  <h1>🛑 JETBOT EMERGENCY BRAKE</h1>
  <div class="badge" id="badge-status">VÙNG AN TOÀN</div>
</div>

<div class="viewport">
  <img id="cam-img" src="/snapshot" alt="Đang kết nối camera...">
  <div class="hud-clearance">
    <div style="font-size:0.7rem; color:#94a3b8;">CỰ LY VẬT CẢN TRƯỚC</div>
    <div class="hud-val" id="hud-val">-- cm</div>
  </div>
</div>

<div class="ctrl-card">
  <div class="status-row">
    <span>Ngưỡng phanh khẩn cấp:</span>
    <span class="status-val" id="thresh-val">25 cm (250 mm)</span>
  </div>
  <div class="status-row">
    <span>Số lần kích hoạt phanh:</span>
    <span class="status-val" id="brake-count">0 lần</span>
  </div>
  <div class="status-row">
    <span>Tốc độ di chuyển:</span>
    <span style="display:flex; gap:6px;">
      <button class="tab-btn" id="spd-slow" onclick="setSpeed(0.15, 0.13, 'slow')" style="padding:2px 8px; border-radius:4px; font-size:0.75rem; background:#131d2e; color:#94a3b8; border:1px solid #1e2d44; cursor:pointer;">🐢 Chậm (15%)</button>
      <button class="tab-btn active" id="spd-norm" onclick="setSpeed(0.20, 0.16, 'norm')" style="padding:2px 8px; border-radius:4px; font-size:0.75rem; background:rgba(0,240,255,0.2); color:var(--cyan); border:1px solid var(--cyan); cursor:pointer; font-weight:bold;">🐇 Chuẩn (20%)</button>
      <button class="tab-btn" id="spd-fast" onclick="setSpeed(0.25, 0.20, 'fast')" style="padding:2px 8px; border-radius:4px; font-size:0.75rem; background:#131d2e; color:#94a3b8; border:1px solid #1e2d44; cursor:pointer;">⚡ Nhanh (25%)</button>
    </span>
  </div>

  <div class="dpad">
    <div></div>
    <button class="btn" id="b-up" onmousedown="drive(speedLinear, speedLinear)" onmouseup="drive(0,0)" ontouchstart="drive(speedLinear, speedLinear)" ontouchend="drive(0,0)">▲</button>
    <div></div>
    <button class="btn" id="b-left" onmousedown="drive(-speedTurn, speedTurn)" onmouseup="drive(0,0)" ontouchstart="drive(-speedTurn, speedTurn)" ontouchend="drive(0,0)">◀</button>
    <button class="btn btn-stop" id="b-stop" onclick="drive(0,0)">■</button>
    <button class="btn" id="b-right" onmousedown="drive(speedTurn, -speedTurn)" onmouseup="drive(0,0)" ontouchstart="drive(speedTurn, -speedTurn)" ontouchend="drive(0,0)">▶</button>
    <div></div>
    <button class="btn" id="b-down" onmousedown="drive(-speedLinear, -speedLinear)" onmouseup="drive(0,0)" ontouchstart="drive(-speedLinear, -speedLinear)" ontouchend="drive(0,0)">▼</button>
    <div></div>
  </div>

  <div class="tip">💡 Dùng phím W-A-S-D hoặc các mũi tên để lái xe thẳng vào chướng ngại vật</div>
</div>

<script>
  let speedLinear = 0.20;
  let speedTurn = 0.16;

  function setSpeed(lin, turn, mode) {
    speedLinear = lin;
    speedTurn = turn;
    ['spd-slow', 'spd-norm', 'spd-fast'].forEach(id => {
      const el = document.getElementById(id);
      if (el) {
        el.style.background = '#131d2e';
        el.style.color = '#94a3b8';
        el.style.borderColor = '#1e2d44';
        el.style.fontWeight = 'normal';
      }
    });
    const cur = document.getElementById('spd-' + mode);
    if (cur) {
      cur.style.background = 'rgba(0,240,255,0.2)';
      cur.style.color = 'var(--cyan)';
      cur.style.borderColor = 'var(--cyan)';
      cur.style.fontWeight = 'bold';
    }
  }

  let driveTimer = null;
  function drive(l, r) {
    if (driveTimer) clearInterval(driveTimer);
    fetch('/drive?l=' + l + '&r=' + r).catch(()=>{});
    if (l !== 0 || r !== 0) {
      driveTimer = setInterval(() => fetch('/drive?l=' + l + '&r=' + r).catch(()=>{}), 160);
    }
  }

  // Cập nhật liên tục hình ảnh
  const camImg = document.getElementById('cam-img');
  function refreshImg() {
    const t = new Image();
    t.onload = () => { camImg.src = t.src; setTimeout(refreshImg, 50); };
    t.onerror = () => { setTimeout(refreshImg, 100); };
    t.src = '/snapshot?t=' + Date.now();
  }
  refreshImg();

  // Polling trạng thái phanh
  async function poll() {
    try {
      const res = await fetch('/api/status');
      const d = await res.json();
      const c_cm = (d.clearance_mm / 10).toFixed(1);
      const hud = document.getElementById('hud-val');
      const badge = document.getElementById('badge-status');

      hud.innerText = (d.clearance_mm > 4000) ? '> 4.0 m' : c_cm + ' cm';
      document.getElementById('brake-count').innerText = d.brake_count + ' lần';

      if (d.is_braked) {
        hud.classList.add('alert');
        badge.classList.add('alert');
        badge.innerText = '🚨 PHANH KHẨN CẤP!';
      } else if (d.clearance_mm < 450) {
        hud.classList.remove('alert');
        badge.classList.remove('alert');
        badge.innerText = '⚠️ CẢNH BÁO GIẢM TỐC';
        badge.style.borderColor = 'var(--amber)';
        badge.style.color = 'var(--amber)';
      } else {
        hud.classList.remove('alert');
        badge.classList.remove('alert');
        badge.innerText = 'VÙNG AN TOÀN';
        badge.style.borderColor = 'var(--green)';
        badge.style.color = 'var(--green)';
      }
    } catch(e) {}
    setTimeout(poll, 120);
  }
  poll();

  // Bàn phím WASD
  window.addEventListener('keydown', (e) => {
    if (e.repeat) return;
    const k = e.key.toLowerCase();
    const code = e.code;
    if (k === 'w' || code === 'KeyW' || k === 'arrowup') { drive(speedLinear, speedLinear); highlight('b-up', true); }
    else if (k === 's' || code === 'KeyS' || k === 'arrowdown') { drive(-speedLinear, -speedLinear); highlight('b-down', true); }
    else if (k === 'a' || code === 'KeyA' || k === 'arrowleft') { drive(-speedTurn, speedTurn); highlight('b-left', true); }
    else if (k === 'd' || code === 'KeyD' || k === 'arrowright') { drive(speedTurn, -speedTurn); highlight('b-right', true); }
    else if (k === ' ' || k === 'escape' || code === 'Space') { drive(0,0); highlight('b-stop', true); }
  });
  window.addEventListener('keyup', (e) => {
    const k = e.key.toLowerCase();
    const code = e.code;
    if (k === 'w' || code === 'KeyW' || k === 'arrowup') highlight('b-up', false);
    if (k === 's' || code === 'KeyS' || k === 'arrowdown') highlight('b-down', false);
    if (k === 'a' || code === 'KeyA' || k === 'arrowleft') highlight('b-left', false);
    if (k === 'd' || code === 'KeyD' || k === 'arrowright') highlight('b-right', false);
    if (k === ' ' || k === 'escape' || code === 'Space') highlight('b-stop', false);

    if (['w','s','a','d','arrowup','arrowdown','arrowleft','arrowright'].includes(k) ||
        ['KeyW','KeyS','KeyA','KeyD','Space'].includes(code)) {
      drive(0,0);
    }
  });
  function highlight(id, on) {
    const el = document.getElementById(id);
    if (el) el.classList.toggle('active', on);
  }
</script>
</body>
</html>
"""


# ─── 6. HTTP SERVER ĐIỀU KHIỂN & API ──────────────────────────────────────────
class WebHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args): pass

    def do_GET(self):
        global current_left, current_right, last_drive_time, forward_clearance_mm, brake_threshold_mm

        if self.path in ['/', '/index.html']:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))

        elif self.path.startswith('/snapshot'):
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-cache, private')
            self.end_headers()
            if latest_rgb_jpeg:
                self.wfile.write(latest_rgb_jpeg)
            else:
                blank = np.zeros((360, 480, 3), dtype=np.uint8)
                cv2.putText(blank, "OAK-D S2 INITIALIZING...", (80, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 240, 255), 2)
                _, j = cv2.imencode('.jpg', blank)
                self.wfile.write(j.tobytes())

        elif self.path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            res = {
                "clearance_mm": round(forward_clearance_mm, 1),
                "thresh_mm": brake_threshold_mm,
                "is_braked": is_emergency_braked,
                "brake_count": brake_event_count,
                "motor_connected": robot.is_connected
            }
            self.wfile.write(json.dumps(res).encode('utf-8'))

        elif self.path.startswith('/drive'):
            l, r = 0.0, 0.0
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('l='): l = float(p.split('=')[1])
                    if p.startswith('r='): r = float(p.split('=')[1])

            last_drive_time = time.time()

            # CAN THIỆP PHANH: Nếu cản nguy hiểm (< 18cm) và đang bấm TIẾN -> KHÓA LỆNH TIẾN!
            # Luôn cho phép LÙI (l < 0, r < 0) hoặc QUAY ĐẦU (l, r trái dấu) để thoát cản
            is_trying_to_move_forward = (l > 0.05 and r > 0.05)
            if is_trying_to_move_forward and forward_clearance_mm < brake_threshold_mm:
                l, r = 0.0, 0.0
                robot.stop()
                print(f"🛑 [KHÓA LỆNH TIẾN] Cản cách {forward_clearance_mm/10.0:.1f} cm (< {brake_threshold_mm/10.0:.0f} cm). Cho phép LÙI hoặc QUAY để thoát!")
            else:
                current_left = l
                current_right = r
                robot.set_motors(l, r)

            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'OK')
        else:
            self.send_response(404)
            self.end_headers()


class ThreadedServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def main():
    print("\n" + "═" * 70)
    print("🛑 KHỞI ĐỘNG CHẾ ĐỘ KIỂM THỬ PHANH KHẨN CẤP (EMERGENCY BRAKE TEST)")
    print(f"  ├─ Mạch động cơ: {robot.driver_name} (addr=0x60)")
    print(f"  ├─ Ngưỡng phanh khẩn cấp: {brake_threshold_mm/10.0:.0f} cm ({brake_threshold_mm:.0f} mm)")
    print("  └─ Web Dashboard: http://0.0.0.0:8080")
    print("═" * 70 + "\n")

    threading.Thread(target=oak_worker, daemon=True).start()
    threading.Thread(target=watchdog_worker, daemon=True).start()

    server = ThreadedServer(('0.0.0.0', 8080), WebHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Đang dừng an toàn...")
        robot.stop()
        server.shutdown()


if __name__ == '__main__':
    main()
