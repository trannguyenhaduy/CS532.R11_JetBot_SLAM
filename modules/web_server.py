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
            self.send_header('Cache-Control', 'no-cache, private')
            self.end_headers()
            while inst and inst.is_running:
                jpeg = inst.get_latest_jpeg() if inst else None
                if jpeg:
                    try:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                         str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                    except Exception:
                        break
                time.sleep(0.05)
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
                    if p.startswith('v='): v = float(p.split('=')[1])
                    if p.startswith('w='): w = float(p.split('=')[1])
            if inst:
                inst.on_drive_command(v, w)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'OK')
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
                    if p.startswith('v='): v = float(p.split('=')[1])
                    if p.startswith('w='): w = float(p.split('=')[1])
            if inst:
                inst.on_drive_command(v, w)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'OK')
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
        self.drive_cmd_cb = None
        self.feature_toggle_cb = None

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


if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử WebCockpitServer...")
    ws = WebCockpitServer(port=8899)
    ws.start()
    time.sleep(1.0)
    assert ws.thread.is_alive(), "Lỗi: Luồng server web không chạy!"
    ws.stop()
    print("✅ [SELF-TEST] WebCockpitServer ĐẠT CHUẨN!")
