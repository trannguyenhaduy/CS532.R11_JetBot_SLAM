#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 7: Trạm điều khiển Web Cyber Cockpit 3D Three.js (Web Server)
Cung cấp luồng video MJPEG, REST API điều khiển W-A-S-D và hiển thị mây điểm 3D.
"""

import sys
import os
import json
import time
import math
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

# Tải giao diện Three.js Cyber Cockpit 3D từ modules/templates/cockpit.html
TEMPLATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates', 'cockpit.html')

def get_cockpit_html():
    if os.path.exists(TEMPLATE_PATH):
        try:
            with open(TEMPLATE_PATH, 'r', encoding='utf-8') as f:
                return f.read()
        except Exception as e:
            print(f"⚠️ [WEB] Không thể đọc {TEMPLATE_PATH}: {e}")
    try:
        from slam_web_dashboard import HTML_PAGE as legacy_html
        return legacy_html
    except Exception:
        pass
    return """<!DOCTYPE html><html><head><title>JetBot Cockpit</title></head>
    <body style='background:#07090e;color:#00f0ff;font-family:sans-serif;text-align:center;padding:50px;'>
    <h1>🤖 JETBOT MODULAR COCKPIT</h1><p>Không tìm thấy cockpit.html</p>
    </body></html>"""

HTML_PAGE = get_cockpit_html()

class WebHandler(BaseHTTPRequestHandler):
    server_instance = None

    def setup(self):
        super().setup()
        import socket
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Khóa bộ đệm phát 64KB chống ứ đọng khung hình cũ khi WiFi bị giật
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        except Exception:
            pass

    def log_message(self, format, *args):
        pass  # Tắt log spam HTTP request

    def do_GET(self):
        inst = self.server_instance
        if self.path in ['/', '/index.html']:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            content = get_cockpit_html() or HTML_PAGE
            self.wfile.write(content.encode('utf-8'))
        elif self.path.startswith('/stream.mjpg'):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.send_header('Connection', 'close')
            self.end_headers()
            last_id = -1
            while inst and getattr(inst, 'is_running', True):
                frame_evt = getattr(inst, 'new_frame_event', None)
                if frame_evt:
                    frame_evt.wait(timeout=0.035)
                    frame_evt.clear()

                frame_id, jpeg = inst.get_latest_jpeg_with_id()
                if jpeg and (frame_id != last_id or frame_id == 0):
                    last_id = frame_id
                    try:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                         str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                        self.wfile.flush()
                    except Exception:
                        break
                elif not frame_evt:
                    time.sleep(0.008)
        elif self.path == '/api/state':
            state_data = inst.get_state_dict() if inst else {}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(state_data).encode('utf-8'))
        elif self.path.startswith('/api/drive'):
            v, w = 0.0, 0.0
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('v='):
                        try: v = float(p.split('=')[1])
                        except Exception: v = 0.0
                    if p.startswith('w='):
                        try: w = float(p.split('=')[1])
                        except Exception: w = 0.0
            if inst:
                try: inst.on_drive_command(v, w)
                except Exception as e: print(f"⚠️ [WEB DRIVE ERR]: {e}")
            self.send_response(200)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(b'OK')
        elif self.path.startswith('/api/camera_mode'):
            mode = 'ai'
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('mode='): mode = p.split('=')[1].lower()
            res_mode = mode
            if inst:
                if hasattr(inst, 'camera') and inst.camera:
                    res_mode = inst.camera.set_view_mode(mode)
                elif hasattr(inst, 'toggle_feature'):
                    inst.toggle_feature(f"cam_mode_{mode}")
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "mode": res_mode}).encode())
            return
        elif self.path.startswith('/api/map/save'):
            map_name = "room_map"
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('name='): map_name = p.split('=')[1].strip()
            ok, res = (False, "No handler")
            if inst:
                ok, res = inst.save_map(map_name)
            self.send_response(200 if ok else 500)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "result": res}).encode())
            return
        elif self.path.startswith('/api/map/reset'):
            ok = False
            if inst:
                ok = inst.reset_map()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "reset": bool(ok)}).encode())
            return
        elif self.path.startswith('/api/auto_scan'):
            active = False
            if inst and hasattr(inst, 'toggle_auto_scan'):
                active = inst.toggle_auto_scan()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "auto_scanning": bool(active)}).encode())
            return
        elif self.path.startswith('/api/toggle'):
            flag_name = None
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('flag='): flag_name = p.split('=')[1]
            if inst and flag_name:
                res = inst.toggle_feature(flag_name)
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({"flag": flag_name, "state": res}).encode())
                return
            self.send_response(400)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        inst = self.server_instance
        if self.path.startswith('/api/drive'):
            v, w = 0.0, 0.0
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('v='):
                        try: v = float(p.split('=')[1])
                        except Exception: v = 0.0
                    if p.startswith('w='):
                        try: w = float(p.split('=')[1])
                        except Exception: w = 0.0
            if inst:
                try: inst.on_drive_command(v, w)
                except Exception as e: print(f"⚠️ [WEB DRIVE ERR]: {e}")
            self.send_response(200)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(b'OK')
        elif self.path.startswith('/api/map/save'):
            map_name = "room_map"
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('name='): map_name = p.split('=')[1].strip()
            ok, res = (False, "No handler")
            if inst:
                ok, res = inst.save_map(map_name)
            self.send_response(200 if ok else 500)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "result": res}).encode())
            return
        elif self.path.startswith('/api/map/reset'):
            ok = False
            if inst:
                ok = inst.reset_map()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "reset": bool(ok)}).encode())
            return
        elif self.path.startswith('/api/auto_scan'):
            active = False
            if inst and hasattr(inst, 'toggle_auto_scan'):
                active = inst.toggle_auto_scan()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "auto_scanning": bool(active)}).encode())
            return
        elif self.path.startswith('/api/toggle'):
            # API bật/tắt tính năng động từ Web
            flag_name = None
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('flag='): flag_name = p.split('=')[1]
            if inst and flag_name:
                res = inst.toggle_feature(flag_name)
                self.send_response(200)
                self.end_headers()
                self.wfile.write(json.dumps({"flag": flag_name, "state": res}).encode())
                return
            self.send_response(400)
            self.end_headers()


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

    def server_bind(self):
        super().server_bind()
        import socket
        try:
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass


class WebCockpitServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.is_running = False
        self.server = None
        self.thread = None

        # Callbacks nối vào hệ thống
        self.state_provider_cb = None
        self.jpeg_provider_cb = None
        self.jpeg_id_provider_cb = None
        self.new_frame_event = None
        self.drive_cmd_cb = None
        self.feature_toggle_cb = None
        self.map_save_cb = None
        self.map_reset_cb = None
        self.auto_scan_cb = None

    def start(self):
        WebHandler.server_instance = self
        self.is_running = True
        self.server = ThreadedHTTPServer((self.host, self.port), WebHandler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        print(f"🌐 [WEB] JetBot 3D Cockpit đã khởi chạy tại: http://{self.host}:{self.port}")

    def stop(self):
        self.is_running = False
        if self.server:
            try: self.server.shutdown()
            except Exception: pass

    def get_latest_jpeg(self):
        if self.jpeg_provider_cb:
            return self.jpeg_provider_cb()
        return None

    def get_latest_jpeg_with_id(self):
        if self.jpeg_id_provider_cb:
            return self.jpeg_id_provider_cb()
        if self.jpeg_provider_cb:
            return 0, self.jpeg_provider_cb()
        return 0, None

    def get_state_dict(self):
        if self.state_provider_cb:
            return self.state_provider_cb()
        return {}

    def on_drive_command(self, v, w):
        if self.drive_cmd_cb:
            self.drive_cmd_cb(v, w)

    def toggle_feature(self, flag_name):
        if self.feature_toggle_cb:
            return self.feature_toggle_cb(flag_name)
        return False

    def save_map(self, name="room_map"):
        if self.map_save_cb:
            return self.map_save_cb(name)
        return False, "Chưa đăng ký hàm lưu bản đồ"

    def reset_map(self):
        if self.map_reset_cb:
            return self.map_reset_cb()
        return False

    def toggle_auto_scan(self):
        if self.auto_scan_cb:
            return self.auto_scan_cb()
        return False



if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử WebCockpitServer...")
    ws = WebCockpitServer(port=8899)
    ws.start()
    time.sleep(1.0)
    assert ws.thread.is_alive(), "Lỗi: Luồng server web không chạy!"
    ws.stop()
    print("✅ [SELF-TEST] WebCockpitServer ĐẠT CHUẨN!")
