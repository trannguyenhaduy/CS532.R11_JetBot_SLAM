#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import json
import time
import socket
import threading
import numpy as np
import depthai as dai
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus
    except ImportError:
        SMBus = None

# Biến toàn cục
latest_rgb_jpeg = None
latest_depth_jpeg = None
running = True
last_drive_time = time.time()

forward_clearance_mm = 9999.0
current_left = 0.0
current_right = 0.0

# --- LỚP ĐIỀU KHIỂN ĐỘNG CƠ (Bypass lỗi Torch) ---
class WaveshareMotorHAT(object):
    def __init__(self):
        from Adafruit_MotorHAT import Adafruit_MotorHAT
        self._hat_api = Adafruit_MotorHAT
        self._driver = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)
        self._left = self._driver.getMotor(1)
        self._right = self._driver.getMotor(2)
        self._pins = ((1, 0), (2, 3))
        self.stop()

    def _set_one(self, motor, pins, value):
        value = max(-1.0, min(1.0, float(value)))
        mapped = int(255.0 * value)
        speed = min(max(abs(mapped), 0), 255)
        motor.setSpeed(speed)
        ina, inb = pins
        if mapped < 0:
            motor.run(self._hat_api.FORWARD)
            self._driver._pwm.setPWM(ina, 0, 0)
            self._driver._pwm.setPWM(inb, 0, speed * 16)
        else:
            motor.run(self._hat_api.BACKWARD)
            self._driver._pwm.setPWM(ina, 0, speed * 16)
            self._driver._pwm.setPWM(inb, 0, 0)

    def set_motors(self, left, right):
        self._set_one(self._left, self._pins[0], left)
        self._set_one(self._right, self._pins[1], right)

    def stop(self):
        for motor, pins in ((self._left, self._pins[0]), (self._right, self._pins[1])):
            motor.run(self._hat_api.RELEASE)
            self._driver._pwm.setPWM(pins[0], 0, 0)
            self._driver._pwm.setPWM(pins[1], 0, 0)

# Khởi tạo Robot
try:
    robot = WaveshareMotorHAT()
    print("🤖 ĐÃ KẾT NỐI MẠCH ĐỘNG CƠ TRỰC TIẾP!")
except Exception as e1:
    try:
        from jetbot import Robot
        robot = Robot()
        print("🤖 Đã kết nối thư viện JetBot mặc định!")
    except Exception as e2:
        print("Đang chạy mô phỏng. Bánh xe sẽ KHÔNG quay!")
        class DummyRobot:
            def set_motors(self, l, r): pass
            def stop(self): pass
        robot = DummyRobot()

# 1. Pipeline OAK-D S2
def create_oak_pipeline():
    p = dai.Pipeline()

    cam_rgb = p.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(480, 360)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(20)

    xout_rgb = p.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)

    mono_l = p.create(dai.node.MonoCamera)
    mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)

    mono_r = p.create(dai.node.MonoCamera)
    mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    stereo = p.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    stereo.setLeftRightCheck(True)
    # --- THÊM DÒNG NÀY ĐỂ KÉO GIẢM ĐIỂM MÙ TỪ 35CM XUỐNG 15CM ---
    stereo.setExtendedDisparity(True)

    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

    # Disparity để hiển thị màu trên web
    xout_depth = p.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.disparity.link(xout_depth.input)
    
    # Raw Depth để đo khoảng cách millimet chính xác cho tính năng Phanh
    xout_raw_depth = p.create(dai.node.XLinkOut)
    xout_raw_depth.setStreamName("raw_depth")
    stereo.depth.link(xout_raw_depth.input)

    return p

# 2. Luồng đọc dữ liệu từ OAK-D
def oak_worker():
    global latest_rgb_jpeg, latest_depth_jpeg, running, forward_clearance_mm
    print("🚀 Đang khởi động OAK-D S2...")
    pipeline = create_oak_pipeline()
    
    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue(name="rgb", maxSize=2, blocking=False)
        q_depth = device.getOutputQueue(name="depth", maxSize=2, blocking=False)
        q_raw_depth = device.getOutputQueue(name="raw_depth", maxSize=2, blocking=False)
        print("✅ OAK-D S2 đã sẵn sàng!")
        
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
        
        while running:
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                ret, jpeg = cv2.imencode('.jpg', in_rgb.getCvFrame(), encode_params)
                if ret: latest_rgb_jpeg = jpeg.tobytes()

            in_depth = q_depth.tryGet()
            if in_depth is not None:
                disp_norm = (in_depth.getCvFrame() * (255.0 / 95.0)).astype(np.uint8)
                disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
                disp_color = cv2.resize(disp_color, (480, 360))
                ret, jpeg = cv2.imencode('.jpg', disp_color, encode_params)
                if ret: latest_depth_jpeg = jpeg.tobytes()

            # --- TÍNH TOÁN KHOẢNG CÁCH VÀ LỌC ĐIỂM MÙ ---
            in_raw_depth = q_raw_depth.tryGet()
            if in_raw_depth is not None:
                depth_data = in_raw_depth.getFrame()
                h, w = depth_data.shape[:2]
                
                # Mở rộng vùng quét lên 80x120 pixel để lấy mẫu chính xác hơn
                center_roi = depth_data[h//2 - 40 : h//2 + 40, w//2 - 60 : w//2 + 60]
                
                # Chỉ lấy các điểm có giá trị từ 10cm (100mm) đến 5m
                valid_depths = center_roi[(center_roi > 100) & (center_roi < 5000)]
                
                total_pixels = center_roi.size
                valid_count = len(valid_depths)
                
                # NẾU BỊ MÙ: Số điểm ảnh hợp lệ tụt xuống dưới 20% tổng diện tích ROI
                # Nghĩa là có một vật thể cực lớn đang che khuất hoàn toàn ống kính
                if valid_count < (total_pixels * 0.2):
                    forward_clearance_mm = 0.0  # Ép về 0 để kích hoạt phanh khẩn cấp
                elif valid_count > 50:
                    # Lọc bớt nhiễu bằng cách lấy giá trị percentile thứ 5 thay vì min tuyệt đối
                    forward_clearance_mm = float(np.percentile(valid_depths, 5))
                else:
                    forward_clearance_mm = 9999.0

            time.sleep(0.01)

def watchdog_worker():
    global running, last_drive_time, current_left, current_right, forward_clearance_mm
    while running:
        # LỚP BẢO VỆ 2: Phanh khẩn cấp thời gian thực đang chạy trớn
        if (current_left > 0.05 and current_right > 0.05) and forward_clearance_mm < 180:
            robot.stop()
            current_left = 0.0
            current_right = 0.0
            print(f"🚨 TỰ ĐỘNG PHANH! Vật cản cách {forward_clearance_mm:.0f}mm.")
        
        elif time.time() - last_drive_time > 1.0:
            robot.stop()
            current_left = 0.0
            current_right = 0.0
            
        time.sleep(0.05)

def get_battery():
    if SMBus is None: return 12.0, 85.0
    try:
        bus = SMBus(1)
        r = bus.read_word_data(0x41, 2)
        bus.close()
        v = round((((r & 0xFF) << 8) | (r >> 8) >> 3) * 0.004, 2)
        if v >= 12.6: p = 100.0
        elif v <= 9.6: p = 0.0
        elif v >= 11.4: p = 20.0 + (v - 11.4) / 1.2 * 80.0
        else: p = (v - 9.6) / 1.8 * 20.0
        return v, round(p, 1)
    except:
        return 12.0, 80.0

# 3. Giao diện Web
HTML_DASHBOARD = """<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>JetBot OAK-D S2 Control</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, sans-serif; }
  body { background: #070b14; color: #f8fafc; display: flex; flex-direction: column; align-items: center; min-height: 100vh; padding: 14px; }
  header { display: flex; justify-content: space-between; align-items: center; width: 100%; max-width: 600px; margin-bottom: 10px; }
  h1 { font-size: 1.1rem; color: #38bdf8; font-weight: 700; }
  .badge { background: #1e293b; padding: 5px 12px; border-radius: 16px; font-size: 0.85rem; border: 1px solid #334155; }
  .badge span { color: #10b981; font-weight: bold; }

  .mode-selector { display: flex; gap: 8px; width: 100%; max-width: 600px; margin-bottom: 10px; }
  .mode-btn { flex: 1; padding: 10px; background: #131d31; color: #94a3b8; border: 1px solid #1e293b; border-radius: 8px; cursor: pointer; font-size: 0.85rem; font-weight: 600; text-align: center; }
  .mode-btn.active { background: #0284c7; color: #fff; border-color: #38bdf8; }

  .viewport { position: relative; width: 100%; max-width: 600px; height: 380px; background: #000; border-radius: 12px; overflow: hidden; border: 2px solid #1e293b; box-shadow: 0 10px 30px rgba(0,0,0,0.6); }
  #camView { width: 100%; height: 100%; object-fit: contain; display: block; }
  
  .hud { position: absolute; top: 10px; left: 10px; right: 10px; display: flex; justify-content: space-between; pointer-events: none; }
  .hud-tag { background: rgba(15, 23, 42, 0.8); backdrop-filter: blur(4px); padding: 4px 8px; border-radius: 6px; font-size: 0.75rem; font-family: monospace; border: 1px solid rgba(255,255,255,0.15); font-weight: bold; }
  #hudLag { color: #4ade80; }

  .controls { width: 100%; max-width: 600px; margin-top: 14px; background: #101726; border-radius: 12px; padding: 16px; border: 1px solid #1e293b; }
  .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; max-width: 240px; margin: 0 auto; }
  button.dir-btn { background: #1e293b; color: #fff; border: 1px solid #334155; padding: 14px; border-radius: 10px; font-size: 1.2rem; cursor: pointer; user-select: none; touch-action: none; }
  button.dir-btn:active { background: #0284c7; transform: scale(0.95); }
  .stop-btn { background: #991b1b !important; border-color: #ef4444 !important; }
  .slider-row { margin-top: 14px; display: flex; align-items: center; justify-content: center; gap: 12px; font-size: 0.85rem; }
  input[type=range] { width: 160px; accent-color: #38bdf8; }
</style>
</head>
<body>

<header>
  <h1>👁 JETBOT + OAK-D S2 CENTER</h1>
  <div class="badge">Pin: <span id="batVal">-- V</span></div>
</header>

<div class="mode-selector">
  <div class="mode-btn active" id="btnRgb" onclick="setMode('rgb')">📷 Camera Màu</div>
  <div class="mode-btn" id="btnDepth" onclick="setMode('depth')">🌊 Bản Đồ Chiều Sâu</div>
</div>

<div class="viewport">
  <div class="hud">
    <div class="hud-tag" id="hudFps" style="color:#fff">● LIVE: 0 FPS</div>
    <div class="hud-tag" id="hudDist" style="color:#4ade80">FRONT: -- mm</div>
    <div class="hud-tag" id="hudLag">LATENCY: -- ms</div>
  </div>
  <img id="camView" alt="OAK-D Stream">
</div>

<div class="controls">
  <div class="grid">
    <div></div>
    <button class="dir-btn" onmousedown="go('up')" onmouseup="stop()" ontouchstart="go('up')" ontouchend="stop()">▲</button>
    <div></div>
    <button class="dir-btn" onmousedown="go('left')" onmouseup="stop()" ontouchstart="go('left')" ontouchend="stop()">◀</button>
    <button class="dir-btn stop-btn" onclick="stop()">■</button>
    <button class="dir-btn" onmousedown="go('right')" onmouseup="stop()" ontouchstart="go('right')" ontouchend="stop()">▶</button>
    <div></div>
    <button class="dir-btn" onmousedown="go('down')" onmouseup="stop()" ontouchstart="go('down')" ontouchend="stop()">▼</button>
    <div></div>
  </div>

  <div class="slider-row">
    <span>Tốc độ:</span>
    <input type="range" id="spRange" min="0.1" max="0.7" step="0.05" value="0.4" onchange="speed=parseFloat(this.value);document.getElementById('spVal').innerText=Math.round(speed*100)+'%'">
    <span id="spVal">40%</span>
  </div>
</div>

<script>
let currentMode = 'rgb';
let speed = 0.4;
let driveInterval = null;
const imgElem = document.getElementById('camView');
const lagElem = document.getElementById('hudLag');
const fpsElem = document.getElementById('hudFps');

function setMode(m) {
  currentMode = m;
  document.getElementById('btnRgb').className = 'mode-btn ' + (m==='rgb'?'active':'');
  document.getElementById('btnDepth').className = 'mode-btn ' + (m==='depth'?'active':'');
}

let isFetching = false;
let frameCount = 0;
let lastFpsTime = performance.now();

function pullNextFrame() {
  if (isFetching) return;
  isFetching = true;
  const tStart = performance.now();
  
  const tempImg = new Image();
  tempImg.onload = () => {
    imgElem.src = tempImg.src;
    const rtt = Math.round(performance.now() - tStart);
    lagElem.innerText = `LATENCY: ${rtt} ms`;
    lagElem.style.color = rtt > 80 ? '#f87171' : '#4ade80';

    frameCount++;
    isFetching = false;
    requestAnimationFrame(pullNextFrame);
  };
  tempImg.onerror = () => {
    isFetching = false;
    setTimeout(pullNextFrame, 50);
  };
  tempImg.src = '/snapshot?mode=' + currentMode + '&t=' + tStart;
}

pullNextFrame();

setInterval(() => {
  const now = performance.now();
  const fps = Math.round((frameCount * 1000) / (now - lastFpsTime));
  fpsElem.innerText = `● LIVE: ${fps} FPS`;
  frameCount = 0;
  lastFpsTime = now;
}, 1000);

function send(l, r) {
  fetch('/drive', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({left: l, right: r})
  }).catch(()=>{});
}

function go(dir) {
  stop();
  let l = 0, r = 0;
  if (dir === 'up')    { l = speed; r = speed; }
  if (dir === 'down')  { l = -speed; r = -speed; }
  if (dir === 'left')  { l = -speed * 0.75; r = speed * 0.75; }
  if (dir === 'right') { l = speed * 0.75; r = -speed * 0.75; }
  send(l, r);
  driveInterval = setInterval(() => send(l, r), 200);
}

function stop() {
  if (driveInterval) clearInterval(driveInterval);
  fetch('/stop', {method: 'POST'}).catch(()=>{});
}

const keys = {'ArrowUp':'up','KeyW':'up','ArrowDown':'down','KeyS':'down','ArrowLeft':'left','KeyA':'left','ArrowRight':'right','KeyD':'right'};
let currKey = null;
window.addEventListener('keydown', (e) => {
  if (keys[e.code] && currKey !== e.code) { currKey = e.code; go(keys[e.code]); }
});
window.addEventListener('keyup', (e) => {
  if (keys[e.code] && currKey === e.code) { currKey = null; stop(); }
});

setInterval(() => {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('batVal').innerText = `${d.voltage}V (${d.percent}%)`;
    let dEle = document.getElementById('hudDist');
    if (d.clearance > 5000) { 
        dEle.innerText = "FRONT: CLEAR"; 
        dEle.style.color = '#4ade80'; 
    } else { 
        dEle.innerText = `FRONT: ${Math.round(d.clearance)} mm`; 
        dEle.style.color = d.clearance < 180 ? '#ef4444' : '#4ade80'; 
    }
  }).catch(()=>{});
}, 1000);
</script>
</body>
</html>
"""

# 4. HTTP Router
class WebHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args): pass

    def setup(self):
        super().setup()
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def do_GET(self):
        global latest_rgb_jpeg, latest_depth_jpeg, forward_clearance_mm
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_DASHBOARD.encode('utf-8'))
            
        elif self.path.startswith('/snapshot'):
            target_jpeg = latest_depth_jpeg if 'mode=depth' in self.path else latest_rgb_jpeg
            if target_jpeg is not None:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Content-Length', str(len(target_jpeg)))
                self.end_headers()
                self.wfile.write(target_jpeg)
            else:
                self.send_response(503)
                self.end_headers()

        elif self.path == '/status':
            v, p = get_battery()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            data = {'voltage': v, 'percent': p, 'clearance': forward_clearance_mm}
            self.wfile.write(json.dumps(data).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        global last_drive_time, current_left, current_right, forward_clearance_mm
        try:
            if self.path == '/drive':
                length = int(self.headers.get('Content-Length', 0))
                data = json.loads(self.rfile.read(length).decode('utf-8'))
                l_speed = float(data.get('left', 0))
                r_speed = float(data.get('right', 0))
                
                # LỚP BẢO VỆ 1: Chặn lệnh ngay lập tức từ Web UI
                if (l_speed > 0.05 and r_speed > 0.05) and forward_clearance_mm < 180:
                    robot.stop()
                    current_left = 0.0
                    current_right = 0.0
                    print(f"⚠️ CẢNH BÁO: Đã chặn lệnh tiến! Vật cản cách {forward_clearance_mm:.0f}mm.")
                else:
                    robot.set_motors(l_speed, r_speed)
                    current_left = l_speed
                    current_right = r_speed
                    
                last_drive_time = time.time()
                self.send_response(200)
                self.end_headers()
                
            elif self.path == '/stop':
                robot.stop()
                current_left = 0.0
                current_right = 0.0
                self.send_response(200)
                self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()
        except Exception as e:
            print(f"Lỗi POST: {e}")
            self.send_response(500)
            self.end_headers()

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

threading.Thread(target=oak_worker, daemon=True).start()
threading.Thread(target=watchdog_worker, daemon=True).start()

PORT = 8101
server = ThreadedHTTPServer(('0.0.0.0', PORT), WebHandler)
print(f"\n🚀 ĐÃ BẬT OAK-D S2 WEB CENTER TẠI CỔNG {PORT}!")
try:
    server.serve_forever()
except KeyboardInterrupt:
    running = False
    robot.stop()
    server.server_close()