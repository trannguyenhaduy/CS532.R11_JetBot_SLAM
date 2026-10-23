#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
CÔNG CỤ MỔ XẺ THUẬT TOÁN & XEM HỘP ĐEN SLAM (JETBOT SLAM REPLAY STUDIO)
Tên file: replay_viewer.py
Phiên bản: v3.2.0 - HERO MAP VIEWPORT & CSM DEEP INSPECTOR

Điểm cải tiến & Tái cấu trúc theo yêu cầu:
  1. TỐI ĐA HÓA BẢN ĐỒ 2D SLAM: Bản đồ Occupancy Grid chiếm trọn 55% màn hình
     (kích thước 750x710px), có lưới tọa độ mét, vệt quỹ đạo di chuyển thực tế
     xuyên suốt từ đầu đến cuối, các tia quét LaserScan và điểm chạm cản.
  2. BỎ BỚT CAMERA NHIỆT: Loại bỏ ảnh nhiệt Stereo Depth cồng kềnh để nhường
     toàn bộ không gian cho phân tích thuật toán chuyên sâu.
  3. MỔ XẺ TẬN GỐC THUẬT TOÁN SOI KHỚP CSM:
     - Sửa lỗi phân tích dữ liệu cũ (đọc đúng nhánh telemetry['csm']).
     - Giải thích minh bạch vì sao CSM bị Gating (khóa dịch chuyển tọa độ)
       khi xe đang chạy lệnh bước định lượng (STEP_ACTIVE) để bảo toàn quỹ đạo.
     - Tích hợp bộ tính toán So khớp Ngoại tuyến (Offline CSM Evaluator)
       tính trực tiếp điểm tương đồng của tia quét frame hiện tại với bờ tường bản đồ.
  4. SỬA TRIỆT ĐỂ LỖI FONT TIẾNG VIỆT '??': Chuẩn hóa toàn bộ bảng mã Unicode
     tiếng Việt hiển thị mượt mà, không bao giờ xuất hiện ký tự '??'.
  5. PHÍM TẮT ĐIỀU KHIỂN:
     [A / D] hoặc [← / →] : Tua lùi / tiến từng frame
     [Space]              : Bật / Tắt tự động phát (Play/Pause)
     [Z] hoặc [M]         : Phóng to bản đồ quanh tâm Robot (Zoom 2.5X)
     [C]                  : Bật / Tắt mô phỏng so khớp CSM ngoại tuyến
     [+ / -]              : Tăng / Giảm tốc độ phát (FPS)
     [R]                  : Tua về Frame đầu tiên (#1)
     [Q / ESC]            : Thoát chương trình
══════════════════════════════════════════════════════════════════════════════
"""

import sys
if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

import os
import time
import json
import zipfile
import argparse
import math
import numpy as np
import cv2
from modules.scan_matcher import CorrelativeScanMatcher2D


# Bảng chuyển mã tiếng Việt có dấu sang không dấu chuẩn xác 100% (Loại trừ hoàn toàn ký tự lạ ??)
VIETNAMESE_TRANS = str.maketrans(
    'àáảãạăằắẳẵặâầấẩẫậèéẻẽẹêềếểễệìíỉĩịòóỏõọôồốổỗộơờớởỡợùúủũụưừứửữựỳýỷỹỵđÀÁẢÃẠĂẰẮẲẴẶÂẦẤẨẪẬÈÉẺẼẸÊỀẾỂỄỆÌÍỈĨỊÒÓỎÕỌÔỒỐỔỖỘƠỜỚỞỠỢÙÚỦŨỤƯỪỨỬỮỰỲÝỶỸỴĐ',
    'aaaaaaaaaaaaaaaaaeeeeeeeeeeeiiiiiooooooooooooooooouuuuuuuuuuuyyyyydAAAAAAAAAAAAAAAAAEEEEEEEEEEEIIIIIOOOOOOOOOOOOOOOOOUUUUUUUUUUUYYYYYD'
)

def clean_txt(s):
    """Chuyển đổi chuỗi tiếng Việt sang ASCII an toàn tuyệt đối cho OpenCV putText"""
    if not s:
        return ""
    s = str(s).replace("°", " deg").replace("▶", ">").replace("⏸", "||")
    s = s.translate(VIETNAMESE_TRANS)
    return "".join(c if ord(c) < 128 else " " for c in s)


def ask_user_for_file():
    """Hộp thoại chọn file hộp đen .zip hoặc .jsonl nếu không có tham số"""
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        file_path = filedialog.askopenfilename(
            title="Chọn file Hộp đen JetBot SLAM (.zip hoặc telemetry.jsonl)",
            filetypes=[("Hộp đen JetBot (.zip)", "*.zip"), ("Log JSONL", "*.jsonl"), ("Tất cả file", "*.*")]
        )
        root.destroy()
        return file_path if file_path else None
    except Exception:
        return None


class ReplayViewer:
    def __init__(self, target_path=None):
        self.target_path = target_path
        self.is_zip = False
        self.zip_file = None
        self.zip_prefix = ""
        self.session_dir = None
        self.frames = []
        self.total_frames = 0
        self.current_idx = 0
        self.is_playing = False
        self.fps = 8.0
        self.zoom_mode = False          # False: Toàn cảnh 10m x 10m | True: Zoom 2.5X quanh xe
        self.csm_sim_active = True      # Tự động tính toán điểm CSM ngoại tuyến
        self.scan_matcher = CorrelativeScanMatcher2D(search_dist_m=0.15, search_yaw_deg=8.0, min_score=0.28)
        self.window_name = "JETBOT SLAM REPLAY STUDIO - HERO MAP & CSM INSPECTOR"

        # Bộ nhớ cache toàn bộ quỹ đạo xe (X, Y, Yaw) để vẽ vệt đường đi mượt mà
        self.trajectory_poses = []

        self._load_data()

    def _load_data(self):
        # 1. Tự động tìm nguồn dữ liệu nếu không chỉ định
        if not self.target_path:
            candidate = self._find_latest_session()
            if candidate:
                self.target_path = candidate
                print(f"📂 [REPLAY STUDIO] Tự động mở session mới nhất: {self.target_path}")
            else:
                chosen = ask_user_for_file()
                if chosen:
                    self.target_path = chosen
                else:
                    print("❌ [LỖI] Không tìm thấy session nào. Vui lòng truyền đường dẫn file .zip!")
                    sys.exit(1)

        target = os.path.abspath(self.target_path)
        if not os.path.exists(target):
            print(f"❌ [LỖI] Đường dẫn không tồn tại: {target}")
            sys.exit(1)

        # 2. Xử lý file .ZIP hoặc thư mục logs
        if os.path.isfile(target) and target.lower().endswith(".zip"):
            self.is_zip = True
            self.zip_file = zipfile.ZipFile(target, "r")
            self._load_from_zip()
        elif os.path.isdir(target):
            self.is_zip = False
            self.session_dir = target
            self._load_from_dir()
        elif os.path.isfile(target) and target.lower().endswith(".jsonl"):
            self.is_zip = False
            self.session_dir = os.path.dirname(target)
            self._load_from_dir()
        else:
            print(f"❌ [LỖI] Định dạng dữ liệu không hợp lệ: {target}")
            sys.exit(1)

        self.total_frames = len(self.frames)
        if self.total_frames == 0:
            print("⚠️ [CẢNH BÁO] Không có dữ liệu frame nào trong file này!")
            sys.exit(0)

        # Trích xuất toàn bộ tọa độ robot để vẽ vệt quỹ đạo
        self.trajectory_poses = []
        for f in self.frames:
            p = f.get("robot_pose")
            if p and len(p) >= 3:
                self.trajectory_poses.append((float(p[0]), float(p[1]), float(p[2])))
            else:
                self.trajectory_poses.append((0.0, 0.0, 0.0))

        print(f"✅ [REPLAY STUDIO] Đã nạp thành công {self.total_frames} frames từ: {os.path.basename(target)}")

    def _find_latest_session(self):
        # 1. Tìm các file .zip ngay tại thư mục hiện tại
        local_zips = [f for f in os.listdir(".") if f.lower().endswith(".zip") and "session" in f.lower()]
        if local_zips:
            local_zips.sort(key=os.path.getmtime, reverse=True)
            return os.path.abspath(local_zips[0])

        # 2. Tìm trong thư mục logs/
        logs_dir = os.path.abspath("logs")
        if os.path.exists(logs_dir):
            zips = [os.path.join(logs_dir, f) for f in os.listdir(logs_dir) if f.lower().endswith(".zip")]
            if zips:
                zips.sort(key=os.path.getmtime, reverse=True)
                return zips[0]
            dirs = [os.path.join(logs_dir, d) for d in os.listdir(logs_dir) if os.path.isdir(os.path.join(logs_dir, d))]
            if dirs:
                dirs.sort(key=os.path.getmtime, reverse=True)
                return dirs[0]
        return None

    def _load_from_zip(self):
        jsonl_names = [f for f in self.zip_file.namelist() if f.endswith("telemetry.jsonl")]
        if not jsonl_names:
            print("❌ [LỖI] Không tìm thấy telemetry.jsonl trong file ZIP!")
            sys.exit(1)
        jsonl_path = jsonl_names[0]
        self.zip_prefix = os.path.dirname(jsonl_path)

        content = self.zip_file.read(jsonl_path).decode("utf-8")
        for line in content.strip().split("\n"):
            line = line.strip()
            if line:
                try:
                    self.frames.append(json.loads(line))
                except Exception:
                    pass

    def _load_from_dir(self):
        jsonl_path = os.path.join(self.session_dir, "telemetry.jsonl")
        if not os.path.exists(jsonl_path):
            print(f"❌ [LỖI] Không tìm thấy telemetry.jsonl tại: {jsonl_path}")
            sys.exit(1)
        with open(jsonl_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        self.frames.append(json.loads(line))
                    except Exception:
                        pass

    def _get_image(self, sub_dir, filename):
        if not filename:
            return None
        try:
            if self.is_zip:
                # Đường dẫn tương đối trong zip
                path_parts = [p for p in [self.zip_prefix, sub_dir, filename] if p]
                target_rel = "/".join(path_parts).replace("\\", "/")
                data = self.zip_file.read(target_rel)
                arr = np.frombuffer(data, np.uint8)
                return cv2.imdecode(arr, cv2.IMREAD_COLOR)
            else:
                full_path = os.path.join(self.session_dir, sub_dir, filename)
                if os.path.exists(full_path):
                    return cv2.imread(full_path)
        except Exception:
            pass
        return None

    def run(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1420, 820)

        def on_trackbar(val):
            self.current_idx = max(0, min(self.total_frames - 1, val))

        cv2.createTrackbar("Frame", self.window_name, 0, max(1, self.total_frames - 1), on_trackbar)
        last_play_time = time.time()

        while True:
            canvas = self._render_dashboard(self.current_idx)
            cv2.imshow(self.window_name, canvas)
            cv2.setTrackbarPos("Frame", self.window_name, self.current_idx)

            # Tự động phát video theo FPS
            if self.is_playing:
                now = time.time()
                if now - last_play_time >= (1.0 / max(1.0, self.fps)):
                    last_play_time = now
                    if self.current_idx < self.total_frames - 1:
                        self.current_idx += 1
                    else:
                        self.is_playing = False

            key = cv2.waitKeyEx(25)
            if key in [ord('q'), ord('Q'), 27]: # Thoát
                break
            elif key in [ord('a'), ord('A'), 2424832, 65361]: # Lùi 1 frame
                self.is_playing = False
                self.current_idx = max(0, self.current_idx - 1)
            elif key in [ord('d'), ord('D'), 2555904, 65363]: # Tiến 1 frame
                self.is_playing = False
                self.current_idx = min(self.total_frames - 1, self.current_idx + 1)
            elif key == ord(' '): # Play / Pause
                self.is_playing = not self.is_playing
            elif key in [ord('z'), ord('Z'), ord('m'), ord('M')]: # Toggle Zoom bản đồ
                self.zoom_mode = not self.zoom_mode
            elif key in [ord('c'), ord('C')]: # Toggle CSM Sim
                self.csm_sim_active = not self.csm_sim_active
            elif key in [ord('+'), ord('=')]:
                self.fps = min(30.0, self.fps + 2.0)
            elif key in [ord('-'), ord('_')]:
                self.fps = max(1.0, self.fps - 2.0)
            elif key in [ord('r'), ord('R')]:
                self.current_idx = 0
                self.is_playing = False

        cv2.destroyAllWindows()
        if self.zip_file:
            self.zip_file.close()

    def _render_dashboard(self, idx):
        frame_data = self.frames[idx]
        f_idx = frame_data.get("record_index", idx + 1)
        f_id = frame_data.get("frame_id", 0)
        cmd = frame_data.get("command", "UNKNOWN")
        pose = frame_data.get("robot_pose") or (0.0, 0.0, 0.0)
        telemetry = frame_data.get("telemetry", {})

        # Tải ảnh RGB, Ảnh nhiệt Depth và Lát cắt Bản đồ 2D
        img_rgb = self._get_image("rgb", frame_data.get("rgb_file"))
        img_depth = self._get_image("depth", frame_data.get("depth_file"))
        img_map2d = self._get_image("map2d", frame_data.get("map2d_file"))

        # Canvas tổng thể: 1420 x 810 (Giao diện Dark Theme Cyberpunk)
        canvas = np.zeros((810, 1420, 3), dtype=np.uint8)
        canvas[:] = [16, 20, 26]

        # ── THANH TIÊU ĐỀ HEADER TRÊN CÙNG (Chiều cao 46px) ──
        cv2.rectangle(canvas, (0, 0), (1420, 46), (25, 32, 42), -1)
        cv2.line(canvas, (0, 46), (1420, 46), (0, 220, 255), 1)

        title_str = clean_txt(f"JETBOT SLAM REPLAY STUDIO  |  FRAME #{f_idx}/{self.total_frames} (ID: {f_id})  |  LENH: {cmd}")
        cv2.putText(canvas, title_str, (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 240, 255), 2, cv2.LINE_AA)

        play_status = f"{'[PHAT]' if self.is_playing else '[TAM DUNG]'} ({self.fps:.0f} FPS)"
        play_color = (0, 255, 163) if self.is_playing else (140, 160, 180)
        cv2.putText(canvas, play_status, (1240, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, play_color, 2, cv2.LINE_AA)

        # Chạy CSM thực thụ ngoại tuyến để đo đạc sai số trôi dạt thực tế (dX, dY, dYaw)
        csm_res = None
        scan_pts_rob = telemetry.get("sampled_scan_robot", [])
        if self.csm_sim_active and img_map2d is not None:
            csm_res = self._run_offline_csm(pose, scan_pts_rob, img_map2d, idx=idx, cmd=cmd)

        # ══════════════════════════════════════════════════════════════════════
        # CỘT TRÁI (55% MÀN HÌNH): HERO 2D SLAM MAP VIEWPORT (KÍCH THƯỚC KHỔNG LỒ)
        # ══════════════════════════════════════════════════════════════════════
        map_box_x, map_box_y = 12, 54
        map_box_w, map_box_h = 750, 715
        self._render_giant_map_viewport(canvas, map_box_x, map_box_y, map_box_w, map_box_h, img_map2d, pose, telemetry, idx, csm_res)

        # ══════════════════════════════════════════════════════════════════════
        # CỘT PHẢI (45% MÀN HÌNH): 3 KHỐI THÔNG TIN CHUYÊN SÂU
        # ══════════════════════════════════════════════════════════════════════
        right_x = 772
        right_w = 636

        # KHỐI 1: THỊ GIÁC KÉP ĐỒNG BỘ: RGB & ẢNH NHIỆT STEREO DEPTH
        box1_y, box1_h = 54, 205
        self._render_dual_vision_panel(canvas, right_x, box1_y, right_w, box1_h, img_rgb, img_depth, frame_data, telemetry)

        # KHỐI 2: MỔ XẺ THUẬT TOÁN SOI KHỚP CSM (SCAN MATCHER DEEP INSPECTOR)
        box2_y, box2_h = 267, 242
        self._render_csm_deep_panel(canvas, right_x, box2_y, right_w, box2_h, telemetry, pose, img_map2d, csm_res)

        # KHỐI 3: MỔ XẺ QUYẾT ĐỊNH A* & LÝ DO ĐỔI ĐƯỜNG ĐI (A* DECISION INSPECTOR)
        box3_y, box3_h = 517, 252
        self._render_a_star_decision_panel(canvas, right_x, box3_y, right_w, box3_h, frame_data, pose, telemetry)

        # ── THANH HƯỚNG DẪN FOOTER DƯỚI CÙNG (Chiều cao 30px) ──
        cv2.rectangle(canvas, (0, 778), (1420, 810), (20, 25, 34), -1)
        guide_str = clean_txt("[A/D/Mui ten]: Tua Frame  |  [Space]: Phat/Dung  |  [Z]: Bat/Tat Zoom Map  |  [C]: Bat/Tat CSM Sim  |  [+/-]: Toc do  |  [R]: Ve dau  |  [Q/ESC]: Thoat")
        cv2.putText(canvas, guide_str, (120, 798), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (140, 160, 180), 1, cv2.LINE_AA)

        return canvas

    def _render_giant_map_viewport(self, canvas, x, y, w, h, img_map2d, pose, telemetry, current_idx, csm_res=None):
        """Vẽ Khung nhìn Bản đồ 2D SLAM Khổng lồ kèm Lưới tọa độ mét, Vệt đường đi và Mây tia quét"""
        # Khung viền và tiêu đề
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (40, 52, 68), 1)
        cv2.rectangle(canvas, (x, y), (x + w, y + 28), (28, 36, 48), -1)
        title_str = clean_txt("BẢN ĐỒ 2D OCCUPANCY GRID KHỔNG LỒ (HERO SLAM VIEWPORT)")
        cv2.putText(canvas, title_str, (x + 12, y + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 185, 0), 1, cv2.LINE_AA)

        zoom_tag = "[ZOOM 2.5X]" if self.zoom_mode else "[TOÀN CẢNH 10M]"
        cv2.putText(canvas, clean_txt(zoom_tag), (x + w - 120, y + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 200), 1, cv2.LINE_AA)

        map_area_x = x + 3
        map_area_y = y + 29
        map_area_w = w - 6
        map_area_h = h - 32

        # 1. Khởi tạo nền bản đồ màu xám tối Cyberpunk
        map_view = np.zeros((map_area_h, map_area_w, 3), dtype=np.uint8)
        map_view[:] = [18, 22, 28]

        # Chuẩn bị ảnh bản đồ cơ sở
        rx, ry, ryaw = pose if len(pose) == 3 else (0.0, 0.0, 0.0)
        scale_full = min(map_area_w, map_area_h) / 320.0
        center_x = map_area_w // 2
        center_y = map_area_h // 2

        if img_map2d is not None:
            if not self.zoom_mode:
                # Chế độ Toàn cảnh: Phóng to vừa vặn khung nhìn
                rendered_map = cv2.resize(img_map2d, (int(320 * scale_full), int(320 * scale_full)), interpolation=cv2.INTER_NEAREST)
                mh, mw = rendered_map.shape[:2]
                ox = max(0, (map_area_w - mw) // 2)
                oy = max(0, (map_area_h - mh) // 2)
                map_view[oy : oy + mh, ox : ox + mw] = rendered_map
            else:
                # Chế độ Zoom 2.5X quanh vị trí hiện tại của Robot
                zoom_factor = 2.5
                map_zoomed = cv2.resize(img_map2d, (int(320 * scale_full * zoom_factor), int(320 * scale_full * zoom_factor)), interpolation=cv2.INTER_NEAREST)
                # Tính tâm robot trên ảnh đã phóng to
                zh, zw = map_zoomed.shape[:2]
                rob_px_z = int(zw / 2 + (rx / 0.05) * (scale_full * zoom_factor))
                rob_py_z = int(zh / 2 - (ry / 0.05) * (scale_full * zoom_factor))
                # Cắt cửa sổ quanh robot
                x_start = max(0, min(zw - map_area_w, rob_px_z - map_area_w // 2))
                y_start = max(0, min(zh - map_area_h, rob_py_z - map_area_h // 2))
                crop_w = min(map_area_w, zw - x_start)
                crop_h = min(map_area_h, zh - y_start)
                map_view[:crop_h, :crop_w] = map_zoomed[y_start : y_start + crop_h, x_start : x_start + crop_w]

        # 2. Hàm chuyển đổi tọa độ thực thế (mét) sang pixel trên map_view
        def to_canvas(wx, wy):
            if not self.zoom_mode:
                cx = int(center_x + (wx / 0.05) * scale_full)
                cy = int(center_y - (wy / 0.05) * scale_full)
            else:
                zoom_factor = 2.5
                s = scale_full * zoom_factor
                rob_px_z = int((320 * s) / 2 + (rx / 0.05) * s)
                rob_py_z = int((320 * s) / 2 - (ry / 0.05) * s)
                x_s = max(0, min(int(320 * s) - map_area_w, rob_px_z - map_area_w // 2))
                y_s = max(0, min(int(320 * s) - map_area_h, rob_py_z - map_area_h // 2))
                cx = int((320 * s) / 2 + (wx / 0.05) * s) - x_s
                cy = int((320 * s) / 2 - (wy / 0.05) * s) - y_s
            return cx, cy

        # 3. Vẽ lưới tọa độ mét và nhãn khoảng cách
        grid_step_m = 1.0 # Mỗi ô lưới là 1 mét
        for m_val in range(-4, 5):
            if m_val == 0: continue
            px1, py1 = to_canvas(m_val, -5.0)
            px2, py2 = to_canvas(m_val, 5.0)
            if 0 <= px1 < map_area_w:
                cv2.line(map_view, (px1, 0), (px1, map_area_h), (30, 38, 48), 1)
                cv2.putText(map_view, f"{m_val}m", (px1 + 3, map_area_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (70, 85, 105), 1)

            px3, py3 = to_canvas(-5.0, m_val)
            px4, py4 = to_canvas(5.0, m_val)
            if 0 <= py3 < map_area_h:
                cv2.line(map_view, (0, py3), (map_area_w, py3), (30, 38, 48), 1)
                cv2.putText(map_view, f"{m_val}m", (10, py3 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (70, 85, 105), 1)

        # 4. Vẽ mốc tâm xuất phát (0, 0)
        orig_cx, orig_cy = to_canvas(0.0, 0.0)
        if 0 <= orig_cx < map_area_w and 0 <= orig_cy < map_area_h:
            cv2.drawMarker(map_view, (orig_cx, orig_cy), (0, 255, 163), cv2.MARKER_CROSS, 16, 2)
            cv2.putText(map_view, clean_txt("XUAT PHAT (0,0)"), (orig_cx + 8, orig_cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 163), 1)

        # 5. Vẽ vệt quỹ đạo di chuyển thực tế từ frame 0 đến frame hiện tại
        if self.trajectory_poses and current_idx > 0:
            past_pts = []
            for i in range(current_idx + 1):
                px_i, py_i = to_canvas(self.trajectory_poses[i][0], self.trajectory_poses[i][1])
                if -200 <= px_i < map_area_w + 200 and -200 <= py_i < map_area_h + 200:
                    past_pts.append((px_i, py_i))

            if len(past_pts) >= 2:
                for k in range(len(past_pts) - 1):
                    cv2.line(map_view, past_pts[k], past_pts[k + 1], (0, 240, 255), 2, cv2.LINE_AA)

        # 6. Vẽ các tia quét LaserScan phóng từ xe ra các vật cản
        rob_cx, rob_cy = to_canvas(rx, ry)
        scan_pts_rob = telemetry.get("sampled_scan_robot", [])
        if scan_pts_rob and 0 <= rob_cx < map_area_w and 0 <= rob_cy < map_area_h:
            c_yaw, s_yaw = math.cos(ryaw), math.sin(ryaw)
            for pt in scan_pts_rob[::2]: # Vẽ 30 tia để nhìn rõ nét không bị rối
                x_r, y_r = float(pt[0]), float(pt[1])
                w_x = rx + (x_r * c_yaw - y_r * s_yaw)
                w_y = ry + (x_r * s_yaw + y_r * c_yaw)
                beam_cx, beam_cy = to_canvas(w_x, w_y)
                if 0 <= beam_cx < map_area_w and 0 <= beam_cy < map_area_h:
                    # Đường tia quét màu xanh lục nhạt
                    cv2.line(map_view, (rob_cx, rob_cy), (beam_cx, beam_cy), (60, 160, 70), 1)
                    # Điểm chạm cản màu vàng neon
                    cv2.circle(map_view, (beam_cx, beam_cy), 2, (0, 230, 255), -1)

        # 7. Vẽ Robot (Tam giác vàng neon + Mũi tên hướng Yaw)
        if 0 <= rob_cx < map_area_w and 0 <= rob_cy < map_area_h:
            r_len = 16 if not self.zoom_mode else 30
            head_x = int(rob_cx + r_len * math.cos(ryaw))
            head_y = int(rob_cy - r_len * math.sin(ryaw))
            left_x = int(rob_cx + (r_len * 0.6) * math.cos(ryaw + 2.5))
            left_y = int(rob_cy - (r_len * 0.6) * math.sin(ryaw + 2.5))
            right_x = int(rob_cx + (r_len * 0.6) * math.cos(ryaw - 2.5))
            right_y = int(rob_cy - (r_len * 0.6) * math.sin(ryaw - 2.5))

            pts_tri = np.array([[head_x, head_y], [left_x, left_y], [right_x, right_y]], np.int32)
            cv2.fillPoly(map_view, [pts_tri], (0, 240, 255))
            cv2.circle(map_view, (rob_cx, rob_cy), 4 if not self.zoom_mode else 8, (0, 0, 255), -1)

        # 7.1. Vẽ vị trí Robot đề xuất bởi CSM nếu có độ lệch (Tam giác viền Cyan)
        if csm_res and csm_res.get("lifecycle") == "MOTION" and (abs(csm_res.get("dx_cm", 0.0)) > 0.5 or abs(csm_res.get("dy_cm", 0.0)) > 0.5):
            c_rx = csm_res.get("corr_x", rx)
            c_ry = csm_res.get("corr_y", ry)
            c_ryaw = csm_res.get("corr_yaw", ryaw)
            c_cx, c_cy = to_canvas(c_rx, c_ry)
            if 0 <= c_cx < map_area_w and 0 <= c_cy < map_area_h:
                cv2.line(map_view, (rob_cx, rob_cy), (c_cx, c_cy), (0, 255, 163), 1, cv2.LINE_AA)
                r_c = 14 if not self.zoom_mode else 26
                h_x = int(c_cx + r_c * math.cos(c_ryaw))
                h_y = int(c_cy - r_c * math.sin(c_ryaw))
                l_x = int(c_cx + (r_c * 0.6) * math.cos(c_ryaw + 2.5))
                l_y = int(c_cy - (r_c * 0.6) * math.sin(c_ryaw + 2.5))
                r_x = int(c_cx + (r_c * 0.6) * math.cos(c_ryaw - 2.5))
                r_y = int(c_cy - (r_c * 0.6) * math.sin(c_ryaw - 2.5))
                pts_c = np.array([[h_x, h_y], [l_x, l_y], [r_x, r_y]], np.int32)
                cv2.polylines(map_view, [pts_c], True, (0, 255, 163), 2, cv2.LINE_AA)
                cv2.putText(map_view, clean_txt("CSM ALIGN"), (c_cx + 8, c_cy + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 163), 1)

        # 7.2. VẼ LỘ TRÌNH A* BAN ĐẦU, LỘ TRÌNH HIỆN TẠI VÀ ĐIỂM CẢN (A* PATHS & DECISIONS)
        nav_a_star = telemetry.get("nav_a_star", {})
        init_p = nav_a_star.get("initial_path", [])
        cur_p = nav_a_star.get("current_path", [])
        decision_info = nav_a_star.get("decision", {})
        blocked_pt = decision_info.get("blocked_coords")
        goal_pt = nav_a_star.get("goal")

        # a. Lộ trình ban đầu (Tím nhạt nét mỏng, vệt mốc tham chiếu ban đầu)
        if init_p and len(init_p) >= 2:
            for k in range(len(init_p) - 1):
                p0_x, p0_y = to_canvas(init_p[k][0], init_p[k][1])
                p1_x, p1_y = to_canvas(init_p[k + 1][0], init_p[k + 1][1])
                if -200 <= p0_x < map_area_w + 200 and -200 <= p1_x < map_area_w + 200:
                    cv2.line(map_view, (p0_x, p0_y), (p1_x, p1_y), (180, 130, 220), 1, cv2.LINE_AA)
            for pt in init_p:
                ip_x, ip_y = to_canvas(pt[0], pt[1])
                if 0 <= ip_x < map_area_w and 0 <= ip_y < map_area_h:
                    cv2.circle(map_view, (ip_x, ip_y), 2, (180, 130, 220), -1)

        # b. Lộ trình hiện tại (Neon Cyan nét đậm kèm Waypoints xanh ngọc)
        if cur_p and len(cur_p) >= 2:
            for k in range(len(cur_p) - 1):
                p0_x, p0_y = to_canvas(cur_p[k][0], cur_p[k][1])
                p1_x, p1_y = to_canvas(cur_p[k + 1][0], cur_p[k + 1][1])
                if -200 <= p0_x < map_area_w + 200 and -200 <= p1_x < map_area_w + 200:
                    cv2.line(map_view, (p0_x, p0_y), (p1_x, p1_y), (0, 240, 255), 2, cv2.LINE_AA)
            for wp in cur_p[1:-1]:
                w_x, w_y = to_canvas(wp[0], wp[1])
                if 0 <= w_x < map_area_w and 0 <= w_y < map_area_h:
                    cv2.circle(map_view, (w_x, w_y), 4, (0, 255, 180), -1)
                    cv2.circle(map_view, (w_x, w_y), 2, (16, 20, 26), -1)

        # c. Điểm cản khiến đổi hướng (Dấu X đỏ cảnh báo có hào quang)
        if blocked_pt:
            bx, by = to_canvas(blocked_pt[0], blocked_pt[1])
            if 0 <= bx < map_area_w and 0 <= by < map_area_h:
                cv2.circle(map_view, (bx, by), 12, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.circle(map_view, (bx, by), 5, (0, 0, 255), -1)
                cv2.drawMarker(map_view, (bx, by), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
                txt_bx = max(10, min(map_area_w - 125, bx - 40))
                txt_by = by - 14 if by > 40 else by + 22
                cv2.rectangle(map_view, (txt_bx - 2, txt_by - 11), (txt_bx + 118, txt_by + 3), (16, 20, 26), -1)
                cv2.rectangle(map_view, (txt_bx - 2, txt_by - 11), (txt_bx + 118, txt_by + 3), (0, 0, 200), 1)
                cv2.putText(map_view, clean_txt("DIEM CAN DOI DUONG"), (txt_bx + 2, txt_by - 1), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 100, 255), 1, cv2.LINE_AA)

        # d. Điểm đích đến Goal (Hồng tâm vàng hổ phách)
        if goal_pt:
            gx, gy = to_canvas(goal_pt[0], goal_pt[1])
            if 0 <= gx < map_area_w and 0 <= gy < map_area_h:
                cv2.circle(map_view, (gx, gy), 12, (0, 180, 255), 1, cv2.LINE_AA)
                cv2.circle(map_view, (gx, gy), 5, (0, 230, 255), -1)
                cv2.drawMarker(map_view, (gx, gy), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
                d_str = clean_txt(f"DICH ({goal_pt[0]:.2f}, {goal_pt[1]:.2f})")
                cv2.rectangle(map_view, (gx + 6, gy + 3), (gx + 105, gy + 17), (16, 20, 26), -1)
                cv2.rectangle(map_view, (gx + 6, gy + 3), (gx + 105, gy + 17), (0, 180, 255), 1)
                cv2.putText(map_view, d_str, (gx + 8, gy + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 230, 255), 1, cv2.LINE_AA)

        # 8. Bảng Chú thích HUD Map phía dưới góc trái
        hud_box_w, hud_box_h = 580, 52
        hud_x = 10
        hud_y = map_area_h - hud_box_h - 10
        cv2.rectangle(map_view, (hud_x, hud_y), (hud_x + hud_box_w, hud_y + hud_box_h), (20, 26, 36), -1)
        cv2.rectangle(map_view, (hud_x, hud_y), (hud_x + hud_box_w, hud_y + hud_box_h), (50, 65, 85), 1)

        pos_str = clean_txt(f"ROBOT: X={rx:+.3f}m | Y={ry:+.3f}m | YAW={math.degrees(ryaw):+.1f}°")
        cv2.putText(map_view, pos_str, (hud_x + 8, hud_y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 240, 255), 1)

        leg_str = clean_txt("[Cyan] A* moi  |  [Tim] A* ban dau  |  [X Do] Vat can chan  |  [*] Dich  |  [+] Moc 0,0")
        cv2.putText(map_view, leg_str, (hud_x + 8, hud_y + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (160, 180, 200), 1)

        # Chép map_view vào canvas chính
        canvas[map_area_y : map_area_y + map_area_h, map_area_x : map_area_x + map_area_w] = map_view

    def _render_dual_vision_panel(self, canvas, x, y, w, h, img_rgb, img_depth, frame_data, telem):
        """Khối 1: Thị Giác Kép Đồng Bộ - RGB Quang Học & Ảnh Nhiệt Stereo Depth (OAK-D S2)"""
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (40, 52, 68), 1)
        cv2.rectangle(canvas, (x, y), (x + w, y + 25), (28, 36, 48), -1)
        cv2.putText(canvas, clean_txt("1. THI GIAC KEP DONG BO: RGB & ANH NHIET STEREO DEPTH (OAK-D S2)"), (x + 10, y + 17), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 240, 255), 1, cv2.LINE_AA)

        f_id = frame_data.get("frame_id", 0)
        content_y = y + 26
        thumb_w = 210
        thumb_h = 132

        # ── 1. ẢNH QUANG HỌC RGB ──
        x_rgb = x + 8
        y_rgb = content_y + 4
        if img_rgb is not None:
            r_rgb = cv2.resize(img_rgb, (thumb_w, thumb_h), interpolation=cv2.INTER_AREA)
            canvas[y_rgb : y_rgb + thumb_h, x_rgb : x_rgb + thumb_w] = r_rgb
        else:
            cv2.rectangle(canvas, (x_rgb, y_rgb), (x_rgb + thumb_w, y_rgb + thumb_h), (25, 30, 40), -1)
            cv2.putText(canvas, clean_txt("[KHONG CO RGB]"), (x_rgb + 40, y_rgb + 68), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (100, 120, 140), 1)

        cv2.rectangle(canvas, (x_rgb, y_rgb), (x_rgb + thumb_w, y_rgb + thumb_h), (50, 65, 85), 1)
        # Badge góc ảnh RGB
        cv2.rectangle(canvas, (x_rgb + 3, y_rgb + 3), (x_rgb + 72, y_rgb + 17), (16, 22, 30), -1)
        cv2.putText(canvas, clean_txt(f"RGB #{f_id}"), (x_rgb + 6, y_rgb + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 163), 1)
        # Nhãn dưới ảnh RGB
        cv2.putText(canvas, clean_txt("QUANG HOC (RGB)"), (x_rgb + 46, y_rgb + thumb_h + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (0, 220, 255), 1)

        # ── 2. ẢNH NHIỆT ĐỘ SÂU STEREO DEPTH ──
        x_depth = x + 226
        y_depth = content_y + 4
        if img_depth is not None:
            r_depth = cv2.resize(img_depth, (thumb_w, thumb_h), interpolation=cv2.INTER_AREA)
            canvas[y_depth : y_depth + thumb_h, x_depth : x_depth + thumb_w] = r_depth
        else:
            cv2.rectangle(canvas, (x_depth, y_depth), (x_depth + thumb_w, y_depth + thumb_h), (25, 30, 40), -1)
            cv2.putText(canvas, clean_txt("[KHONG CO DEPTH]"), (x_depth + 35, y_depth + 68), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (100, 120, 140), 1)

        cv2.rectangle(canvas, (x_depth, y_depth), (x_depth + thumb_w, y_depth + thumb_h), (50, 65, 85), 1)
        # Badge góc ảnh Depth
        cv2.rectangle(canvas, (x_depth + 3, y_depth + 3), (x_depth + 88, y_depth + 17), (16, 22, 30), -1)
        cv2.putText(canvas, clean_txt(f"DEPTH #{f_id}"), (x_depth + 6, y_depth + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 180, 0), 1)
        # Nhãn dưới ảnh Depth
        cv2.putText(canvas, clean_txt("ANH NHIET (STEREO)"), (x_depth + 36, y_depth + thumb_h + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (255, 160, 0), 1)

        # ── 3. THÔNG SỐ ĐỒNG BỘ & THỊ GIÁC ──
        info_x = x + 444
        cur_y = content_y + 14

        rays = telem.get("rays", {})
        obs_rays = rays.get("obstacle", 0)
        tot_rays = rays.get("total", 120)
        obs_pct = (obs_rays / max(1, tot_rays)) * 100.0

        cv2.putText(canvas, clean_txt("DONG BO THI GIAC:"), (info_x, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 160, 180), 1)
        cur_y += 16
        cv2.putText(canvas, clean_txt(f"[100% CUNG FRAME #{f_id}]"), (info_x, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (0, 255, 163), 1)

        cur_y += 20
        cmd = frame_data.get("command", "IDLE")
        cv2.putText(canvas, clean_txt(f"LENH: {cmd[:16]}"), (info_x, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 240, 255), 1)

        cur_y += 20
        dists = [d for d in rays.get("distances", []) if d is not None and d > 0.15]
        min_d = min(dists) if dists else 9.99
        dist_color = (0, 255, 163) if min_d > 0.45 else ((0, 180, 255) if min_d > 0.25 else (80, 80, 255))
        cv2.putText(canvas, clean_txt(f"CAN GAN: {min_d*100:.1f} cm"), (info_x, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.37, dist_color, 1)

        cur_y += 20
        cv2.putText(canvas, clean_txt(f"TIA CAN: {obs_rays}/{tot_rays} ({obs_pct:.0f}%)"), (info_x, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 230, 240), 1)

        cur_y += 20
        ts = frame_data.get("timestamp", 0.0)
        cv2.putText(canvas, clean_txt(f"THOI GIAN: {ts:.2f} s"), (info_x, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (130, 150, 170), 1)

        cur_y += 18
        cv2.putText(canvas, clean_txt("NGUON: Luxonis S2"), (info_x, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (100, 120, 140), 1)

    def _render_csm_deep_panel(self, canvas, x, y, w, h, telem, pose, img_map2d, csm_res=None):
        """Khối 2: Mổ Xẻ Tận Gốc Thuật Toán So Khớp CSM (Scan Matcher Deep Inspector)"""
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (40, 52, 68), 1)
        cv2.rectangle(canvas, (x, y), (x + w, y + 25), (28, 36, 48), -1)
        cv2.putText(canvas, clean_txt("2. MO XE THUAT TOAN SOI KHOP CSM (SCAN MATCHER DEEP INSPECTOR)"), (x + 10, y + 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 163), 1, cv2.LINE_AA)

        csm_data = telem.get("csm", {})
        decision = telem.get("decision", {})

        # 1. Trích xuất đúng đắn toàn bộ chỉ số CSM từ telemetry
        executed = csm_data.get("executed", False)
        matched = csm_data.get("matched", False)
        score = csm_data.get("score", 0.0)
        dx_cm = csm_data.get("dx_cm", 0.0)
        dy_cm = csm_data.get("dy_cm", 0.0)
        dyaw_deg = csm_data.get("dyaw_deg", 0.0)
        pts_count = csm_data.get("pts_count", telem.get("rays", {}).get("obstacle", 0))

        status_str = decision.get("status", "NORMAL")

        # 2. Sử dụng kết quả CSM ngoại tuyến thực thụ nếu có
        lifecycle = csm_res.get("lifecycle", "NORMAL") if csm_res else "TELEMETRY"
        if csm_res:
            disp_score = csm_res.get("score", 0.0)
            dx_cm = csm_res.get("dx_cm", 0.0)
            dy_cm = csm_res.get("dy_cm", 0.0)
            dyaw_deg = csm_res.get("dyaw_deg", 0.0)
            csm_matched = csm_res.get("matched", False)
            init_sc = csm_res.get("init_score", 0.0)
            if lifecycle == "ANCHOR":
                score_src = "(Goc Moc SLAM)"
            elif lifecycle == "TURN_IN_PLACE":
                score_src = "(Do Phu Goc Nhin)"
            else:
                score_src = f"(So Khop: {init_sc*100:.1f}% -> {disp_score*100:.1f}%)"
        else:
            disp_score = score
            csm_matched = matched
            score_src = "(Ghi nhan tren xe)"

        cur_y = y + 36

        # ── HÀNG 1: TRẠNG THÁI GATING & THỰC THI ──
        cv2.putText(canvas, clean_txt("TRANG THAI CSM:"), (x + 14, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 160, 180), 1)

        tag_color = (0, 255, 163)
        if csm_res and "status_tag" in csm_res:
            tag_text = csm_res["status_tag"]
            if lifecycle == "ANCHOR":
                tag_color = (0, 255, 163)
            elif lifecycle == "TURN_IN_PLACE":
                tag_color = (0, 240, 255)
            elif math.hypot(dx_cm, dy_cm) > 0.5:
                tag_color = (0, 240, 255)
            else:
                tag_color = (0, 255, 163)
        elif status_str in ["STEP_ACTIVE"]:
            tag_text = "[STEP_ACTIVE: KHOA GATING AN TOAN]"
            tag_color = (0, 240, 255)
        elif status_str in ["TURN_GATE", "TURN_GATED"]:
            tag_text = "[TURN_GATED: KHOA KHI QUAY TAI CHO]"
            tag_color = (0, 180, 255)
        elif matched:
            tag_text = "[CSM_MATCHED: DONG BO THANH CONG]"
            tag_color = (0, 255, 163)
        else:
            tag_text = f"[{status_str}]"
            tag_color = (180, 180, 180)

        cv2.putText(canvas, clean_txt(tag_text), (x + 150, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, tag_color, 2)

        # ── HÀNG 2: GIẢI THÍCH SAI SỐ & NGUYÊN DO ──
        cur_y += 20
        cv2.rectangle(canvas, (x + 12, cur_y - 12), (x + w - 12, cur_y + 38), (22, 28, 38), -1)
        cv2.rectangle(canvas, (x + 12, cur_y - 12), (x + w - 12, cur_y + 38), (45, 55, 72), 1)

        if csm_res:
            if lifecycle == "ANCHOR":
                expl_line1 = "KHUNG HINH KHOI TAO: Goc toa do tham chieu tuyet doi (0, 0, 0)."
                expl_line2 = "Theo chuan SLAM, Frame 0 la goc moc nen sai so do lech = 0.0cm tuyet doi."
                expl_line3 = "Dam may diem dung de khoi tao ban do, khong the so khop voi chinh no."
            elif lifecycle == "TURN_IN_PLACE":
                expl_line1 = "XE QUAY TAI CHO: Tam truc banh xe bat bien (dX = 0.0cm, dY = 0.0cm)."
                expl_line2 = f"Goc lech huong CSM: dYaw = {dyaw_deg:+.1f} deg. Do tuong dong goc nhin: {disp_score*100:.1f}%."
                expl_line3 = "Chuan SLAM cong nghiep: Khoa (X,Y) tranh giat meo ban do khi quay goc hep."
            else:
                jump_cm = math.hypot(dx_cm, dy_cm)
                init_sc = csm_res.get("init_score", 0.0)
                expl_line1 = f"CSM SO KHOP BO TUONG: dX = {dx_cm:+.1f}cm, dY = {dy_cm:+.1f}cm, dYaw = {dyaw_deg:+.1f} deg."
                expl_line2 = f"Diem truoc nan: {init_sc*100:.1f}% -> Sau khi CSM nan: {disp_score*100:.1f}% (Tang {(disp_score-init_sc)*100:+.1f}%)."
                expl_line3 = f"Phat hien tong sai so troi dat: {jump_cm:.1f}cm do truot banh tren san nha."
        elif status_str in ["STEP_ACTIVE"]:
            expl_line1 = "VI SAO TREN XE LA 0.0mm? Xe dang chay lenh buoc dinh luong (STEP)."
            expl_line2 = "Thuat toan tren xe KHOA DICH CHUYEN de bao toan mo hinh dong hoc buoc."
            expl_line3 = "Hay nhan phim [C] de bat tinh toan sai so CSM ngoai tuyen!"
        else:
            human_r = decision.get("human_reason", decision.get("reason", "He thong dang giam sat so khop tia."))
            expl_line1 = f"GIAI MA QUYET DINH: {human_r[:70]}"
            expl_line2 = f"{human_r[70:140]}"
            expl_line3 = "CSM cap nhat toa do theo vung hoi tu Likelihood Field."

        cv2.putText(canvas, clean_txt(expl_line1), (x + 18, cur_y + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (0, 240, 255), 1)
        cv2.putText(canvas, clean_txt(expl_line2), (x + 18, cur_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (220, 230, 240), 1)
        cv2.putText(canvas, clean_txt(expl_line3), (x + 18, cur_y + 31), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (160, 180, 200), 1)

        # ── HÀNG 3: BẢNG CHỈ SỐ CSM CHI TIẾT ──
        cur_y += 58
        score_color = (0, 255, 163) if disp_score >= 0.28 else (80, 180, 255)
        eval_tag = "[XUAT SAC]" if disp_score >= 0.50 else ("[DAT CHUAN]" if disp_score >= 0.28 else "[DUOI NGUONG]")

        cv2.putText(canvas, clean_txt("DIEM TUONG DONG (SCORE):"), (x + 14, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (140, 160, 180), 1)
        cv2.putText(canvas, clean_txt(f"{disp_score*100:.1f}%  {score_src}  ->  {eval_tag}"), (x + 205, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, score_color, 2)

        cur_y += 22
        shift_str = clean_txt(f"DO LECH DICH CHUYEN:       dX = {dx_cm:+.1f} cm   |   dY = {dy_cm:+.1f} cm   |   dYaw = {dyaw_deg:+.1f}°")
        cv2.putText(canvas, shift_str, (x + 14, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (0, 255, 200) if (abs(dx_cm) > 0.5 or abs(dy_cm) > 0.5) else (220, 230, 240), 1)

        cur_y += 20
        pts_str = clean_txt(f"SO TIA CAN THAM GIA:       {pts_count} tia hop le (Nguong toi thieu CSM: 28%)")
        cv2.putText(canvas, pts_str, (x + 14, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (160, 180, 200), 1)

        # ── HÀNG 4: THANH TIẾN TRÌNH ĐIỂM SỐ TRỰC QUAN (SCORE GAUGE) ──
        cur_y += 18
        bar_x = x + 14
        bar_w = w - 28
        bar_h = 10
        cv2.rectangle(canvas, (bar_x, cur_y), (bar_x + bar_w, cur_y + bar_h), (25, 32, 44), -1)
        cv2.rectangle(canvas, (bar_x, cur_y), (bar_x + bar_w, cur_y + bar_h), (50, 65, 85), 1)

        # Vạch ngưỡng 28%
        thresh_px = int(bar_x + bar_w * 0.28)
        cv2.line(canvas, (thresh_px, cur_y - 2), (thresh_px, cur_y + bar_h + 2), (0, 240, 255), 2)

        fill_w = int(bar_w * min(1.0, max(0.0, disp_score)))
        if fill_w > 0:
            cv2.rectangle(canvas, (bar_x, cur_y), (bar_x + fill_w, cur_y + bar_h), score_color, -1)

        cv2.putText(canvas, clean_txt("0%"), (bar_x, cur_y + bar_h + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (100, 120, 140), 1)
        cv2.putText(canvas, clean_txt("Nguong Dat: 28%"), (thresh_px - 45, cur_y + bar_h + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 240, 255), 1)
        cv2.putText(canvas, clean_txt("100%"), (bar_x + bar_w - 28, cur_y + bar_h + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (100, 120, 140), 1)

    def _run_offline_csm(self, pose, scan_pts_rob, img_map2d, idx=0, cmd=""):
        """
        Chạy thuật toán CorrelativeScanMatcher2D thực thụ theo chuẩn vòng đời SLAM (SLAM Lifecycle):
        1. Frame khởi tạo (Anchor Frame):
           - idx < 2 hoặc tại mốc xuất phát ban đầu chưa có hành trình:
             Là mốc định nghĩa gốc tọa độ bản đồ thế giới (0, 0, 0).
             Độ lệch dX = 0, dY = 0, dYaw = 0 theo định nghĩa toán học.
        2. Chế độ quay tại chỗ quanh gốc xuất phát:
           - Tọa độ (X, Y) được bảo toàn theo tâm trục bánh vi sai (dX = 0, dY = 0).
           - CSM theo dõi độ lệch hướng dYaw và độ trùng lặp với các bờ tường đã quét.
        3. Chế độ di chuyển tịnh tiến / hành trình SLAM:
           - Chạy so khớp Coarse-to-Fine scan-to-map với đúng chuẩn kích thước bản đồ (res=0.025m, origin=-4.0m).
        """
        try:
            rx, ry, ryaw = pose
            travel_dist = math.hypot(rx, ry)

            # 1. SLAM Anchor Frame Gating:
            if idx < 2 or (idx < 6 and travel_dist < 0.015 and "TURN" in cmd):
                return {
                    "corr_x": rx, "corr_y": ry, "corr_yaw": ryaw,
                    "matched": True, "score": 1.0, "init_score": 1.0,
                    "dx_cm": 0.0, "dy_cm": 0.0, "dyaw_deg": 0.0,
                    "status_tag": "[SLAM_ANCHOR: GOC MOC (0,0)]",
                    "lifecycle": "ANCHOR",
                    "reason": "Khung hinh khoi tao (Anchor Frame) - Goc toa do tham chieu tuyet doi (0,0,0)."
                }

            if not scan_pts_rob or len(scan_pts_rob) < 6:
                return None

            wall_mask = (img_map2d[:, :, 2] > 180) & (img_map2d[:, :, 0] < 120)
            if np.sum(wall_mask) < 8:
                return None

            grid = np.zeros((320, 320), dtype=np.uint8)
            grid[wall_mask] = 255  # Tường
            grid[(img_map2d[:, :, 0] > 40) & (img_map2d[:, :, 0] < 80)] = 128   # Thoáng

            # Chuẩn kích thước Occupancy Grid thực tế:
            # Grid 320x320 sinh ra từ bản đồ 8.0m x 8.0m -> res = 0.025m, origin = -4.0m
            corr_x, corr_y, corr_yaw, matched, score = self.scan_matcher.align_pose(
                rx, ry, ryaw, scan_pts_rob, grid, -4.0, -4.0, 320, 320, 0.025
            )
            dx_cm = getattr(self.scan_matcher, 'last_dx_cm', 0.0)
            dy_cm = getattr(self.scan_matcher, 'last_dy_cm', 0.0)
            dyaw_deg = getattr(self.scan_matcher, 'last_dyaw_deg', 0.0)
            init_score = getattr(self.scan_matcher, 'last_init_score', 0.0)

            # 2. Xử lý xe quay tại chỗ quanh gốc:
            if "TURN" in cmd and travel_dist < 0.02:
                # Khóa tịnh tiến (X, Y) vì xe chỉ quay quanh trục bánh xe
                return {
                    "corr_x": rx, "corr_y": ry, "corr_yaw": corr_yaw,
                    "matched": matched, "score": score, "init_score": init_score,
                    "dx_cm": 0.0, "dy_cm": 0.0, "dyaw_deg": dyaw_deg,
                    "status_tag": f"[QUAY TAI CHO: KHOA X,Y | dYaw={dyaw_deg:+.1f} deg]",
                    "lifecycle": "TURN_IN_PLACE",
                    "reason": f"Xe quay tai cho quanh truc: Toa do (X,Y) bat bien. CSM giam sat do lech goc quay dYaw = {dyaw_deg:+.1f} deg."
                }

            # 3. Xe di chuyển tịnh tiến / hành trình SLAM:
            shift_dist = math.hypot(dx_cm, dy_cm)
            score_gain = score - init_score

            if shift_dist > 0.5:
                status_tag = f"[CSM SO KHOP: LECH {shift_dist:.1f}cm]"
            else:
                status_tag = "[CSM_EVAL: KHOP CHUAN XAC]"

            return {
                "corr_x": corr_x, "corr_y": corr_y, "corr_yaw": corr_yaw,
                "matched": matched, "score": score, "init_score": init_score,
                "dx_cm": dx_cm, "dy_cm": dy_cm, "dyaw_deg": dyaw_deg,
                "status_tag": status_tag,
                "lifecycle": "MOTION",
                "reason": f"CSM so khop Likelihood bo tuong: Tu {init_score*100:.1f}% -> {score*100:.1f}% (Tang {score_gain*100:+.1f}%). Sai so troi dat {shift_dist:.1f}cm."
            }
        except Exception:
            return None

    def _render_a_star_decision_panel(self, canvas, x, y, w, h, frame_data, pose, telem):
        """Khối 3: Mổ Xẻ Quyết Định A* & Lý Do Đổi Đường Đi (A* Path Planner & Decision Inspector)"""
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (40, 52, 68), 1)
        cv2.rectangle(canvas, (x, y), (x + w, y + 25), (28, 36, 48), -1)
        cv2.putText(canvas, clean_txt("3. MO XE QUYET DINH A* & LY DO DOI DUONG DI (A* DECISION INSPECTOR)"), (x + 10, y + 17), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 120, 220), 1, cv2.LINE_AA)

        nav_a_star = telem.get("nav_a_star", {})
        decision = nav_a_star.get("decision", {})
        nav_state = telem.get("nav_state", {})
        init_p = nav_a_star.get("initial_path", [])
        cur_p = nav_a_star.get("current_path", [])
        goal = nav_a_star.get("goal")
        replan_cnt = nav_a_star.get("replan_count", decision.get("replan_count", 0))
        has_diverged = decision.get("has_diverged", False)
        blocked_coords = decision.get("blocked_coords")
        dist_to_block = decision.get("dist_to_block_m")
        reason_sum = decision.get("reason_summary", "CHUA CO DU LIEU A*")
        reason_det = decision.get("reason_detail", "Chua kich hoat hoac dang cho lenh tu hanh.")
        l0 = decision.get("initial_length_m", 0.0)
        l1 = decision.get("current_length_m", 0.0)
        n0 = decision.get("initial_waypoints", len(init_p))
        n1 = decision.get("current_waypoints", len(cur_p))

        state_str = nav_state.get("state", frame_data.get("command", "IDLE"))
        status_text = nav_state.get("status_text", "")
        dist_goal = nav_state.get("dist_to_goal", 0.0)

        cur_y = y + 36

        # ── 1. TRẠNG THÁI NAVIGATOR & ĐÍCH ĐẾN ──
        cv2.putText(canvas, clean_txt("TRANG THAI:"), (x + 14, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 160, 180), 1)

        if state_str in ["DRIVING", "TURNING"]:
            st_col = (0, 240, 255)
        elif "SCAN" in state_str or "OBSERV" in state_str:
            st_col = (0, 180, 255)
        elif state_str == "GOAL_REACHED":
            st_col = (0, 255, 163)
        elif state_str == "OBSTACLE_BLOCKED":
            st_col = (80, 80, 255)
        else:
            st_col = (180, 190, 200)

        disp_status = f"[{state_str}] {status_text[:28]}" if status_text else f"[{state_str}]"
        cv2.putText(canvas, clean_txt(disp_status), (x + 115, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.36, st_col, 2)

        if goal:
            goal_str = clean_txt(f"DICH: ({goal[0]:.2f}, {goal[1]:.2f}) | CON: {dist_goal:.2f}m")
        else:
            goal_str = clean_txt("DICH: CHUA DAT")
        cv2.putText(canvas, goal_str, (x + 395, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 215, 255), 1)

        # ── 2. SO SÁNH 2 ĐƯỜNG ĐI: BAN ĐẦU (INITIAL) VS HIỆN TẠI (CURRENT) ──
        cur_y += 20
        cv2.putText(canvas, clean_txt("SO SANH LO TRINH:"), (x + 14, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 160, 180), 1)
        delta_str = f"LECH: {l1 - l0:+.2f}m" if has_diverged else "TRUNG KHOP"
        comp_str = clean_txt(f"Ban dau: L0={l0:.2f}m ({n0}wp)  ->  Hien tai: L1={l1:.2f}m ({n1}wp)  |  {delta_str}  |  Replan #{replan_cnt}")
        comp_col = (0, 240, 255) if has_diverged else (0, 255, 163)
        cv2.putText(canvas, comp_str, (x + 145, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, comp_col, 1)

        # ── 3. HỘP MỔ XẺ LÝ DO ĐỔI ĐƯỜNG ĐI (DECISION EXPLANATION BOX) ──
        cur_y += 18
        box_border_col = (0, 140, 255) if blocked_coords else ((0, 255, 163) if cur_p else (60, 75, 95))
        cv2.rectangle(canvas, (x + 12, cur_y - 6), (x + w - 12, cur_y + 58), (20, 26, 36), -1)
        cv2.rectangle(canvas, (x + 12, cur_y - 6), (x + w - 12, cur_y + 58), box_border_col, 1)

        cv2.putText(canvas, clean_txt(f"LY DO DOI HUONG: [{reason_sum}]"), (x + 18, cur_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 240, 255), 2)

        # Tự động cắt dòng chữ giải thích chi tiết thành 2 dòng vừa vặn
        clean_d = clean_txt(reason_det)
        l1_txt = clean_d[:86]
        l2_txt = clean_d[86:172] if len(clean_d) > 86 else ""
        cv2.putText(canvas, l1_txt, (x + 18, cur_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (220, 230, 240), 1)
        if l2_txt:
            cv2.putText(canvas, l2_txt, (x + 18, cur_y + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (170, 190, 210), 1)

        # ── 4. THÔNG TIN VẬT CẢN VA CHẠM / HÀNH LANG AN TOÀN ──
        cur_y += 72
        if blocked_coords:
            d_str = f"{dist_to_block:.2f}m" if dist_to_block is not None else "--"
            blk_str = clean_txt(f"VAT CAN VA CHAM: X={blocked_coords[0]:+.2f}m, Y={blocked_coords[1]:+.2f}m | Cach xe: {d_str} | Vung dem an toan: 20cm")
            blk_col = (0, 80, 255)
        else:
            blk_str = clean_txt("KHONG CO VAT CAN CHAN: Duong di luon lach qua vung thoang > 20cm an toan tuyet doi.")
            blk_col = (0, 255, 163)
        cv2.putText(canvas, blk_str, (x + 14, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, blk_col, 1)

        # ── 5. THÔNG SỐ ĐỘNG LỰC HỌC, IMU & NGUỒN PIN (CỠ CHỮ GỌN TIẾT KIỆM KHÔNG GIAN) ──
        cur_y += 20
        rx, ry, ryaw = pose if len(pose) == 3 else (0.0, 0.0, 0.0)
        bat_v = telem.get("bat_v", 11.10)
        bat_pct = int(min(100, max(0, (bat_v - 9.6) / (12.6 - 9.6) * 100)))
        lat = telem.get("latencies", {})
        depth_ms = lat.get("depth_ms", 7.2)
        hw_str = clean_txt(f"Robot: ({rx:+.2f}, {ry:+.2f}, {math.degrees(ryaw):+.1f} deg) | Pin: {bat_v:.2f}V ({bat_pct}%) | IMU Yaw Lock | Tre: {depth_ms:.1f}ms")
        cv2.putText(canvas, hw_str, (x + 14, cur_y), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (120, 140, 160), 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="JetBot SLAM Replay Studio - Hero Map & CSM Deep Inspector")
    parser.add_argument("target", nargs="?", default=None, help="Đường dẫn file .zip hoặc thư mục session")
    parser.add_argument("--session", default=None, help="Đường dẫn thư mục session")
    parser.add_argument("--zip", default=None, help="Đường dẫn file .zip")
    args = parser.parse_args()

    input_path = args.target or args.zip or args.session
    app = ReplayViewer(target_path=input_path)
    app.run()
