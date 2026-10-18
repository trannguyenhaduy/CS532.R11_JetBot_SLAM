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

    def handle_one_request(self):
        try:
            super().handle_one_request()
        except (BrokenPipeError, ConnectionResetError, IOError):
            pass

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
                    frame_evt.wait(timeout=0.015)
                    frame_evt.clear()

                frame_id, jpeg = inst.get_latest_jpeg_with_id()
                if jpeg and (frame_id != last_id or frame_id == 0):
                    last_id = frame_id
                    try:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                         str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                        self.wfile.flush()
                        if inst and hasattr(inst, 'record_stream_frame'):
                            inst.record_stream_frame(len(jpeg))
                    except Exception:
                        break
                elif not frame_evt:
                    time.sleep(0.003)
        elif self.path.startswith('/api/state'):
            client_map_ver = -1
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('map_ver='):
                        try: client_map_ver = int(p.split('=')[1])
                        except Exception: client_map_ver = -1
            state_data = inst.get_state_dict(client_map_ver=client_map_ver) if inst else {}
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps(state_data).encode('utf-8'))
            except (BrokenPipeError, ConnectionResetError, IOError):
                pass
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
        elif self.path.startswith('/api/tracker/set_mode'):
            mode = 'person'
            color = 'red'
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('mode='): mode = p.split('=')[1].lower()
                    if p.startswith('color='): color = p.split('=')[1].lower()
            res = False
            if inst and hasattr(inst, 'set_tracker_mode'):
                res = inst.set_tracker_mode(mode, color)
            elif inst and hasattr(inst, 'follower') and inst.follower:
                res = inst.follower.set_mode(mode, color)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "mode": mode, "color": color, "result": res}).encode())
            return
        elif self.path.startswith('/api/tracker/select_roi'):
            bx, by, bw, bh = 0, 0, 0, 0
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('x='): bx = int(float(p.split('=')[1]))
                    if p.startswith('y='): by = int(float(p.split('=')[1]))
                    if p.startswith('w='): bw = int(float(p.split('=')[1]))
                    if p.startswith('h='): bh = int(float(p.split('=')[1]))
            res = False
            if inst and hasattr(inst, 'init_tracker_roi'):
                res = inst.init_tracker_roi((bx, by, bw, bh))
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if res else "ERROR", "roi": [bx, by, bw, bh]}).encode())
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
        elif self.path.startswith('/api/step_move'):
            step_type = 'linear'
            direction = 'forward'
            dist_cm = 20.0
            deg = 45.0
            speed_pct = None
            val_override = None

            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('type='): step_type = p.split('=')[1].lower()
                    elif p.startswith('dist_cm='):
                        try: dist_cm = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('deg='):
                        try: deg = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('val='):
                        try: val_override = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('dir='): direction = p.split('=')[1].lower()
                    elif p.startswith('speed_pct='):
                        try: speed_pct = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('speed='):
                        try: speed_pct = float(p.split('=')[1])
                        except Exception: pass

            target_val = val_override if val_override is not None else (deg if step_type == 'turn' else dist_cm)

            ok, msg = False, "No handler"
            if inst and hasattr(inst, 'execute_step_move'):
                ok, msg = inst.execute_step_move(step_type, target_val, direction, speed_pct=speed_pct)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg}).encode())
            return
        elif self.path.startswith('/api/step_cancel'):
            ok = False
            if inst and hasattr(inst, 'cancel_step_move'):
                ok = inst.cancel_step_move()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "cancelled": ok}).encode())
            return
        elif self.path.startswith('/api/trip_reset'):
            ok = False
            if inst and hasattr(inst, 'reset_trip'):
                ok = inst.reset_trip()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "reset": bool(ok)}).encode())
            return
        elif self.path.startswith('/api/motor_trim'):
            trim_l = 1.0
            trim_r = 1.0
            should_save = False
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('left='):
                        try: trim_l = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('right='):
                        try: trim_r = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('bias='):
                        try:
                            b = float(p.split('=')[1])
                            trim_l = round(1.0 + b, 3)
                            trim_r = round(1.0 - b, 3)
                        except Exception: pass
                    elif p.startswith('save='):
                        should_save = (p.split('=')[1].lower() in ['1', 'true', 'yes'])
            ok = False
            if inst:
                if hasattr(inst, 'motor_trim_cb') and inst.motor_trim_cb:
                    ok = inst.motor_trim_cb(trim_l, trim_r)
                elif hasattr(inst, 'set_motor_trim'):
                    ok = inst.set_motor_trim(trim_l, trim_r)
                elif hasattr(inst, 'motors') and inst.motors:
                    inst.motors.set_trim(trim_l, trim_r)
                    ok = True
                if should_save:
                    if hasattr(inst, 'save_motor_trim_cb') and inst.save_motor_trim_cb:
                        inst.save_motor_trim_cb(trim_l, trim_r)
                    elif hasattr(inst, 'save_motor_trim'):
                        inst.save_motor_trim(trim_l, trim_r)
                    elif hasattr(inst, 'motors') and hasattr(inst.motors, 'save_trim_to_disk'):
                        inst.motors.save_trim_to_disk()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "left": trim_l, "right": trim_r, "updated": bool(ok), "saved": bool(should_save)}).encode())
            return
        elif self.path.startswith('/api/calibration'):
            params = {}
            save_disk = False
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if '=' in p:
                        k, v = p.split('=', 1)
                        if k == 'save':
                            save_disk = (v.lower() in ['1', 'true', 'yes'])
                        elif k in ['trim_left', 'trim_right', 'step_forward_scale', 'step_backward_scale', 'step_turn_left_scale', 'step_turn_right_scale', 'step_turn_min_pwm']:
                            try: params[k] = float(v)
                            except Exception: pass
            ok = False
            if inst and hasattr(inst, 'calibration_cb') and inst.calibration_cb:
                if params:
                    ok = inst.calibration_cb(params, save_to_disk=save_disk)
                else:
                    ok = True
            curr_calib = {}
            if inst and hasattr(inst, 'get_calibration_cb') and inst.get_calibration_cb:
                curr_calib = inst.get_calibration_cb()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "updated": bool(ok), "saved": bool(save_disk), "calibration": curr_calib}).encode())
            return
        elif self.path.startswith('/api/pattern_move'):
            p_type = 'rectangle'
            action = 'start'
            length = 80.0
            width = 40.0
            leg = 50.0
            speed_pct = 10.0
            turn_dir = 'left'
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('type='): p_type = p.split('=')[1].lower()
                    elif p.startswith('action='): action = p.split('=')[1].lower()
                    elif p.startswith('length='):
                        try: length = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('width='):
                        try: width = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('leg='):
                        try: leg = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('speed_pct='):
                        try: speed_pct = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('speed='):
                        try: speed_pct = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('turn_dir='): turn_dir = p.split('=')[1].lower()
            ok = False
            msg = ""
            if inst:
                if action in ['stop', 'cancel']:
                    if hasattr(inst, 'pattern_cancel_cb') and inst.pattern_cancel_cb:
                        ok = inst.pattern_cancel_cb()
                    elif hasattr(inst, 'cancel_pattern_patrol'):
                        ok = inst.cancel_pattern_patrol()
                    msg = "Đã dừng quỹ đạo"
                else:
                    params = {'length': length, 'width': width, 'leg': leg, 'speed_pct': speed_pct, 'turn_dir': turn_dir}
                    if hasattr(inst, 'pattern_move_cb') and inst.pattern_move_cb:
                        ok, msg = inst.pattern_move_cb(p_type, **params)
                    elif hasattr(inst, 'execute_pattern_patrol'):
                        ok, msg = inst.execute_pattern_patrol(p_type, **params)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg}).encode())
            return
        elif self.path.startswith('/api/auto_scan'):
            duration = None
            mode = None
            pwm = None
            steps = None
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('duration='):
                        try: duration = float(p.split('=')[1])
                        except Exception: duration = None
                    elif p.startswith('mode='):
                        mode = p.split('=')[1]
                    elif p.startswith('pwm='):
                        try: pwm = float(p.split('=')[1])
                        except Exception: pwm = None
                    elif p.startswith('steps='):
                        try: steps = int(p.split('=')[1])
                        except Exception: steps = None
            active = False
            if inst and hasattr(inst, 'toggle_auto_scan'):
                active = inst.toggle_auto_scan(duration=duration, mode=mode, pwm=pwm, steps=steps)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "auto_scanning": bool(active)}).encode())
            return
        elif self.path.startswith('/api/plan_path') or self.path.startswith('/api/set_goal'):
            gx, gy = None, None
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('x=') or p.startswith('gx='):
                        try: gx = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('y=') or p.startswith('gy='):
                        try: gy = float(p.split('=')[1])
                        except Exception: pass
            res = {"success": False, "message": "Thiếu tọa độ đích x, y"}
            if inst and gx is not None and gy is not None:
                if hasattr(inst, 'plan_path'):
                    res = inst.plan_path(gx, gy)
                elif hasattr(inst, 'plan_to_goal'):
                    res = inst.plan_to_goal(gx, gy)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps(res).encode())
            return
        elif self.path.startswith('/api/clear_goal'):
            ok = False
            if inst:
                if hasattr(inst, 'clear_goal'):
                    ok = inst.clear_goal()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "cleared": bool(ok)}).encode())
            return
        elif self.path.startswith('/api/start_nav'):
            res = {"success": False, "message": "Server chưa sẵn sàng"}
            if inst:
                if hasattr(inst, 'start_nav'):
                    res = inst.start_nav()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps(res).encode())
            return
        elif self.path.startswith('/api/stop_nav') or self.path.startswith('/api/cancel_nav'):
            res = {"success": False, "message": "Server chưa sẵn sàng"}
            if inst:
                if hasattr(inst, 'stop_nav'):
                    res = inst.stop_nav()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps(res).encode())
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
        elif self.path.startswith('/api/debug_pause'):
            paused = True
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('paused='):
                        paused = (p.split('=')[1].lower() in ['1', 'true', 'yes'])
            res = False
            if inst and hasattr(inst, 'debug_pause_cb') and inst.debug_pause_cb:
                res = inst.debug_pause_cb(paused)
            elif inst and hasattr(inst, 'set_debug_paused'):
                res = inst.set_debug_paused(paused)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "debug_paused": bool(res)}).encode())
            return
        elif self.path.startswith('/api/debug_step'):
            res = False
            if inst and hasattr(inst, 'debug_step_cb') and inst.debug_step_cb:
                res = inst.debug_step_cb()
            elif inst and hasattr(inst, 'request_debug_step'):
                res = inst.request_debug_step()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "step_requested": bool(res)}).encode())
            return
        elif self.path.startswith('/api/session/export'):
            session_id = None
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('session_id='):
                        session_id = p.split('=')[1].strip()

            recorder = getattr(inst, 'recorder', None)
            zip_path = None
            if recorder and hasattr(recorder, 'create_session_zip'):
                zip_path = recorder.create_session_zip(session_id=session_id)

            if zip_path and os.path.exists(zip_path):
                filename = os.path.basename(zip_path)
                file_size = os.path.getsize(zip_path)
                self.send_response(200)
                self.send_header('Content-Type', 'application/zip')
                self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
                self.send_header('Content-Length', str(file_size))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                try:
                    with open(zip_path, 'rb') as f:
                        while True:
                            chunk = f.read(65536)
                            if not chunk:
                                break
                            self.wfile.write(chunk)
                except Exception as ex:
                    print(f"⚠️ [WEB EXPORT ZIP ERR]: {ex}")
                return
            else:
                self.send_response(404)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "Chưa có session frame nào được ghi nhận hoặc không thể nén file ZIP."}).encode())
                return
        elif self.path.startswith('/api/session/stats'):
            recorder = getattr(inst, 'recorder', None)
            stats = recorder.get_session_stats() if recorder else {}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(stats).encode())
            return
        elif self.path.startswith('/api/config/full'):
            res = {}
            if inst and hasattr(inst, 'get_full_config'):
                res = inst.get_full_config()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            payload = dict(res) if isinstance(res, dict) else {}
            payload["status"] = "OK"
            payload["config"] = res
            self.wfile.write(json.dumps(payload).encode())
            return
        elif self.path.startswith('/api/config/select_profile'):
            pid = 1
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('id='):
                        try: pid = int(p.split('=')[1])
                        except Exception: pass
            ok, msg = False, "No handler"
            if inst and hasattr(inst, 'select_profile'):
                ok, msg = inst.select_profile(pid)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg, "profile_id": pid}).encode())
            return
        elif self.path.startswith('/api/config/update'):
            params = {}
            pid = None
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if '=' in p:
                        k, v = p.split('=', 1)
                        if k == 'profile_id':
                            try: pid = int(v)
                            except Exception: pass
                        else:
                            try: params[k] = float(v)
                            except Exception: pass
            ok, msg = False, "No handler"
            if inst and hasattr(inst, 'update_full_config'):
                ok, msg = inst.update_full_config(params, profile_id=pid)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg}).encode())
            return
        elif self.path.startswith('/api/collision/status'):
            res = {}
            if inst and hasattr(inst, 'get_collision_status'):
                res = inst.get_collision_status()
            elif inst and getattr(inst, 'collision_mgr', None):
                res = inst.collision_mgr.get_status()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps(res).encode())
            return
        elif self.path.startswith('/api/collision/capture'):
            lbl = 'free'
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('label='): lbl = p.split('=')[1].lower().strip()
            ok, msg, counts = False, "No handler", {}
            if inst and hasattr(inst, 'capture_collision_sample'):
                ok, msg, counts = inst.capture_collision_sample(lbl)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg, "counts": counts}).encode())
            return
        elif self.path.startswith('/api/collision/undo'):
            ok, msg, counts = False, "No handler", {}
            if inst and hasattr(inst, 'undo_collision_sample'):
                ok, msg, counts = inst.undo_collision_sample()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg, "counts": counts}).encode())
            return
        elif self.path.startswith('/api/collision/train'):
            epochs = 15
            batch = 8
            lr = 0.001
            model = 'resnet18'
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('epochs='):
                        try: epochs = int(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('batch='):
                        try: batch = int(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('lr='):
                        try: lr = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('model='):
                        model = p.split('=')[1].strip()
            ok, msg = False, "No handler"
            if inst and hasattr(inst, 'start_collision_train'):
                ok, msg = inst.start_collision_train(epochs=epochs, batch_size=batch, lr=lr, model_type=model)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg}).encode())
            return
        elif self.path.startswith('/api/collision/toggle_avoid'):
            is_avoiding = False
            if inst and hasattr(inst, 'toggle_collision_avoid'):
                is_avoiding = inst.toggle_collision_avoid()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "is_avoiding": bool(is_avoiding)}).encode())
            return
        elif self.path.startswith('/api/collision/export'):
            mgr = getattr(inst, 'collision_mgr', None)
            if mgr:
                ok, path = mgr.export_dataset_zip()
                if ok and os.path.exists(path):
                    try:
                        with open(path, 'rb') as f:
                            zip_data = f.read()
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/zip')
                        self.send_header('Content-Disposition', 'attachment; filename="dataset_collision_jetbot.zip"')
                        self.send_header('Content-Length', str(len(zip_data)))
                        self.end_headers()
                        self.wfile.write(zip_data)
                        return
                    except Exception: pass
            self.send_response(500)
            self.end_headers()
            return
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
        elif self.path.startswith('/api/step_move'):
            step_type = 'linear'
            direction = 'forward'
            dist_cm = 20.0
            deg = 45.0
            speed_pct = None
            val_override = None

            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('type='): step_type = p.split('=')[1].lower()
                    elif p.startswith('dist_cm='):
                        try: dist_cm = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('deg='):
                        try: deg = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('val='):
                        try: val_override = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('dir='): direction = p.split('=')[1].lower()
                    elif p.startswith('speed_pct='):
                        try: speed_pct = float(p.split('=')[1])
                        except Exception: pass
                    elif p.startswith('speed='):
                        try: speed_pct = float(p.split('=')[1])
                        except Exception: pass

            target_val = val_override if val_override is not None else (deg if step_type == 'turn' else dist_cm)

            ok, msg = False, "No handler"
            if inst and hasattr(inst, 'execute_step_move'):
                ok, msg = inst.execute_step_move(step_type, target_val, direction, speed_pct=speed_pct)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg}).encode())
            return
        elif self.path.startswith('/api/step_cancel'):
            ok = False
            if inst and hasattr(inst, 'cancel_step_move'):
                ok = inst.cancel_step_move()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "cancelled": ok}).encode())
            return
        elif self.path.startswith('/api/trip_reset'):
            ok = False
            if inst and hasattr(inst, 'reset_trip'):
                ok = inst.reset_trip()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "reset": bool(ok)}).encode())
            return
        elif self.path.startswith('/api/auto_scan'):
            duration = None
            mode = None
            pwm = None
            steps = None
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('duration='):
                        try: duration = float(p.split('=')[1])
                        except Exception: duration = None
                    elif p.startswith('mode='):
                        mode = p.split('=')[1]
                    elif p.startswith('pwm='):
                        try: pwm = float(p.split('=')[1])
                        except Exception: pwm = None
                    elif p.startswith('steps='):
                        try: steps = int(p.split('=')[1])
                        except Exception: steps = None
            active = False
            if inst and hasattr(inst, 'toggle_auto_scan'):
                active = inst.toggle_auto_scan(duration=duration, mode=mode, pwm=pwm, steps=steps)
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
        elif self.path.startswith('/api/config/select_profile'):
            pid = 1
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('id='):
                        try: pid = int(p.split('=')[1])
                        except Exception: pass
            ok, msg = False, "No handler"
            if inst and hasattr(inst, 'select_profile'):
                ok, msg = inst.select_profile(pid)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg, "profile_id": pid}).encode())
            return
        elif self.path.startswith('/api/config/update'):
            params = {}
            pid = None
            try:
                length = int(self.headers.get('content-length', 0))
                if length > 0:
                    body_bytes = self.rfile.read(length)
                    data = json.loads(body_bytes.decode('utf-8'))
                    if isinstance(data, dict):
                        params = data.get('params', data)
                        pid = data.get('profile_id', None)
            except Exception:
                pass
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if '=' in p:
                        k, v = p.split('=', 1)
                        if k == 'profile_id':
                            try: pid = int(v)
                            except Exception: pass
                        else:
                            try: params[k] = float(v)
                            except Exception: pass
            ok, msg = False, "No handler"
            if inst and hasattr(inst, 'update_full_config'):
                ok, msg = inst.update_full_config(params, profile_id=pid)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg}).encode())
            return
        elif self.path.startswith('/api/config/create_profile'):
            name = None
            desc = None
            params = {}
            try:
                length = int(self.headers.get('content-length', 0))
                if length > 0:
                    body_bytes = self.rfile.read(length)
                    data = json.loads(body_bytes.decode('utf-8'))
                    if isinstance(data, dict):
                        name = data.get('name')
                        desc = data.get('description')
                        params = data.get('params', {})
            except Exception:
                pass
            ok, msg, new_pid = False, "No handler", None
            if inst and hasattr(inst, 'create_profile'):
                ok, msg, new_pid = inst.create_profile(name=name, description=desc, params=params)
            cfg = inst.get_full_config() if inst and hasattr(inst, 'get_full_config') else {}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg, "profile_id": new_pid, "config": cfg}).encode())
            return
        elif self.path.startswith('/api/config/delete_profile'):
            pid = None
            try:
                length = int(self.headers.get('content-length', 0))
                if length > 0:
                    body_bytes = self.rfile.read(length)
                    data = json.loads(body_bytes.decode('utf-8'))
                    if isinstance(data, dict):
                        pid = data.get('id') or data.get('profile_id')
            except Exception:
                pass
            if pid is None and '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('id='):
                        try: pid = int(p.split('=')[1])
                        except Exception: pass
            ok, msg = False, "No handler"
            if inst and hasattr(inst, 'delete_profile'):
                ok, msg = inst.delete_profile(pid)
            cfg = inst.get_full_config() if inst and hasattr(inst, 'get_full_config') else {}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK" if ok else "ERROR", "message": msg, "config": cfg}).encode())
            return
        else:
            self.send_response(400)
            self.end_headers()


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address):
        import sys
        exc_type, exc_val, _ = sys.exc_info()
        if exc_type in (BrokenPipeError, ConnectionResetError) or isinstance(exc_val, (BrokenPipeError, ConnectionResetError)):
            # Client (trình duyệt web) vừa tắt tab hoặc refresh trang F5, socket bị ngắt tự nhiên
            return
        super().handle_error(request, client_address)

    def server_bind(self):
        import socket
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception:
            pass
        super().server_bind()
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
        self.step_move_cb = None
        self.step_cancel_cb = None
        self.trip_reset_cb = None
        self.plan_path_cb = None
        self.clear_goal_cb = None
        self.start_nav_cb = None
        self.stop_nav_cb = None
        self.motor_trim_cb = None
        self.save_motor_trim_cb = None
        self.calibration_cb = None
        self.get_calibration_cb = None
        self.get_full_config_cb = None
        self.update_full_config_cb = None
        self.select_profile_cb = None
        self.create_profile_cb = None
        self.delete_profile_cb = None
        self.pattern_move_cb = None
        self.pattern_cancel_cb = None
        self.debug_pause_cb = None
        self.debug_step_cb = None
        self.tracker_mode_cb = None
        self.tracker_roi_cb = None

        # Thống kê băng thông & FPS luồng stream video mạng (Network Stream Profiler)
        self.stream_stats = {
            "fps_sent": 0.0,
            "kbps": 0.0,
            "bytes_total": 0,
            "frames_total": 0,
            "avg_frame_kb": 0.0
        }
        self._stat_lock = threading.Lock()
        self._stat_last_time = time.time()
        self._stat_frame_counter = 0
        self._stat_byte_counter = 0

    def record_stream_frame(self, byte_len):
        now = time.time()
        with self._stat_lock:
            self._stat_frame_counter += 1
            self._stat_byte_counter += byte_len
            self.stream_stats["bytes_total"] += byte_len
            self.stream_stats["frames_total"] += 1
            dt = now - self._stat_last_time
            if dt >= 0.8:
                self.stream_stats["fps_sent"] = round(self._stat_frame_counter / dt, 1)
                self.stream_stats["kbps"] = round((self._stat_byte_counter / 1024.0) / dt, 1)
                if self._stat_frame_counter > 0:
                    self.stream_stats["avg_frame_kb"] = round((self._stat_byte_counter / 1024.0) / self._stat_frame_counter, 1)
                self._stat_frame_counter = 0
                self._stat_byte_counter = 0
                self._stat_last_time = now

    def get_stream_stats(self):
        with self._stat_lock:
            if time.time() - self._stat_last_time > 2.0 and self._stat_frame_counter == 0:
                self.stream_stats["fps_sent"] = 0.0
                self.stream_stats["kbps"] = 0.0
            return dict(self.stream_stats)

    def start(self):
        WebHandler.server_instance = self
        self.is_running = True
        try:
            self.server = ThreadedHTTPServer((self.host, self.port), WebHandler)
        except OSError as e:
            if getattr(e, 'errno', None) == 98 or 'Address already in use' in str(e):
                print(f"\n❌ [WEB ERROR] Cổng {self.port} đang bị một tiến trình Python khác (hoặc main.py cũ chưa thoát hẳn) chiếm dụng!")
                print(f"👉 Hãy chạy lệnh sau trên Terminal của Jetson để giải phóng cổng 8080 ngay lập tức:")
                print(f"   fuser -k {self.port}/tcp   (hoặc pkill -9 -f main.py)\n")
            raise e
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

    def get_state_dict(self, client_map_ver=-1):
        if self.state_provider_cb:
            try:
                return self.state_provider_cb(client_map_ver=client_map_ver)
            except TypeError:
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

    def toggle_auto_scan(self, duration=None, mode=None, pwm=None, steps=None):
        if self.auto_scan_cb:
            try:
                return self.auto_scan_cb(duration=duration, mode=mode, pwm=pwm, steps=steps)
            except TypeError:
                try:
                    return self.auto_scan_cb(duration=duration, mode=mode, pwm=pwm)
                except TypeError:
                    try:
                        return self.auto_scan_cb(duration=duration)
                    except TypeError:
                        return self.auto_scan_cb()
        return False

    def execute_step_move(self, step_type, val, direction, speed_pct=None):
        if self.step_move_cb:
            try:
                return self.step_move_cb(step_type, val, direction, speed_pct=speed_pct)
            except TypeError:
                return self.step_move_cb(step_type, val, direction)
        return False, "Chưa đăng ký hàm bước"

    def cancel_step_move(self):
        if self.step_cancel_cb:
            return self.step_cancel_cb()
        return False

    def reset_trip(self):
        if self.trip_reset_cb:
            return self.trip_reset_cb()
        return True

    def plan_path(self, gx, gy):
        if self.plan_path_cb:
            return self.plan_path_cb(gx, gy)
        return {"success": False, "message": "Chưa đăng ký hàm tìm đường"}

    def clear_goal(self):
        if self.clear_goal_cb:
            return self.clear_goal_cb()
        return False

    def start_nav(self):
        if self.start_nav_cb:
            return self.start_nav_cb()
        return {"success": False, "message": "Chưa đăng ký hàm tự hành (start_nav_cb)"}

    def stop_nav(self):
        if self.stop_nav_cb:
            return self.stop_nav_cb()
        return {"success": False, "message": "Chưa đăng ký hàm dừng tự hành (stop_nav_cb)"}

    def get_full_config(self):
        if self.get_full_config_cb:
            return self.get_full_config_cb()
        return {}

    def update_full_config(self, params, profile_id=None):
        if self.update_full_config_cb:
            return self.update_full_config_cb(params, profile_id=profile_id)
        return False, "Chưa đăng ký hàm cập nhật cấu hình"

    def select_profile(self, profile_id):
        if self.select_profile_cb:
            return self.select_profile_cb(profile_id)
        return False, "Chưa đăng ký hàm chọn profile cấu hình"

    def create_profile(self, name=None, description=None, params=None):
        if self.create_profile_cb:
            return self.create_profile_cb(name=name, description=description, params=params)
        return False, "Chưa đăng ký hàm tạo profile mới", None

    def delete_profile(self, profile_id):
        if self.delete_profile_cb:
            return self.delete_profile_cb(profile_id)
        return False, "Chưa đăng ký hàm xóa profile"

    def set_tracker_mode(self, mode, color='red'):
        if self.tracker_mode_cb:
            return self.tracker_mode_cb(mode, color)
        return False

    def init_tracker_roi(self, bbox):
        if self.tracker_roi_cb:
            return self.tracker_roi_cb(bbox)
        return False

    def get_collision_status(self):
        if hasattr(self, 'collision_status_cb') and self.collision_status_cb:
            return self.collision_status_cb()
        if getattr(self, 'collision_mgr', None):
            return self.collision_mgr.get_status()
        return {}

    def capture_collision_sample(self, label='free'):
        if hasattr(self, 'collision_capture_cb') and self.collision_capture_cb:
            return self.collision_capture_cb(label)
        if getattr(self, 'collision_mgr', None):
            frame = None
            if self.jpeg_provider_cb:
                # Nếu có frame
                pass
            return self.collision_mgr.save_sample(frame, label)
        return False, "Chưa đăng ký callback", {}

    def undo_collision_sample(self):
        if hasattr(self, 'collision_undo_cb') and self.collision_undo_cb:
            return self.collision_undo_cb()
        if getattr(self, 'collision_mgr', None):
            return self.collision_mgr.undo_last_sample()
        return False, "Chưa đăng ký callback", {}

    def start_collision_train(self, epochs=15, batch_size=8, lr=0.001, model_type='resnet18'):
        if hasattr(self, 'collision_train_cb') and self.collision_train_cb:
            return self.collision_train_cb(epochs=epochs, batch_size=batch_size, lr=lr, model_type=model_type)
        if getattr(self, 'collision_mgr', None):
            return self.collision_mgr.start_training(epochs=epochs, batch_size=batch_size, lr=lr, model_type=model_type)
        return False, "Chưa đăng ký callback"

    def toggle_collision_avoid(self):
        if hasattr(self, 'collision_toggle_avoid_cb') and self.collision_toggle_avoid_cb:
            return self.collision_toggle_avoid_cb()
        if getattr(self, 'collision_mgr', None):
            return self.collision_mgr.toggle_avoidance()
        return False




if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử WebCockpitServer...")
    ws = WebCockpitServer(port=8899)
    ws.start()
    time.sleep(1.0)
    assert ws.thread.is_alive(), "Lỗi: Luồng server web không chạy!"
    ws.stop()
    print("✅ [SELF-TEST] WebCockpitServer ĐẠT CHUẨN!")
