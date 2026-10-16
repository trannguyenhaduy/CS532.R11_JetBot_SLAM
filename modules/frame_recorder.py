#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
MODULE: COMMAND-TRIGGERED FRAME & ALGORITHM RECORDER (RECORDER ENGINE)
Tên file: modules/frame_recorder.py
Chức năng:
  - Chỉ ghi nhận và lưu trữ Frame + Telemetry phân tích thuật toán KHI CÓ LỆNH
    di chuyển đang thực thi (bước tiến/lùi, quay góc, quét 360, lái WASD).
  - Tự động bỏ qua các khoảng thời gian nghỉ, đứng yên, không có lệnh.
  - Lưu ảnh RGB, ảnh phổ nhiệt Depth và toàn bộ chỉ số CSM, Latency vào thư mục session.
  - Tự động đóng gói thành file ZIP để tải về máy tính phân tích ngoại tuyến từng frame.
══════════════════════════════════════════════════════════════════════════════
"""

import os
import time
import json
import zipfile
import threading
import cv2
import numpy as np


class FrameRecorder:
    def __init__(self, base_dir="logs"):
        self.base_dir = os.path.abspath(base_dir)
        os.makedirs(self.base_dir, exist_ok=True)
        self.lock = threading.Lock()

        self.is_recording = False
        self.current_session_id = None
        self.current_session_dir = None
        self.current_cmd_name = ""
        self.session_frame_count = 0
        self.session_start_time = 0.0
        self.active_telemetry = []

    def start_command(self, cmd_name="COMMAND"):
        """Kích hoạt ghi nhận frame khi bắt đầu nhận lệnh di chuyển"""
        with self.lock:
            self.current_cmd_name = str(cmd_name)
            if not self.is_recording or self.current_session_dir is None:
                ts_str = time.strftime("%Y%m%d_%H%M%S")
                self.current_session_id = f"session_{ts_str}"
                self.current_session_dir = os.path.join(self.base_dir, self.current_session_id)
                os.makedirs(os.path.join(self.current_session_dir, "rgb"), exist_ok=True)
                os.makedirs(os.path.join(self.current_session_dir, "depth"), exist_ok=True)
                self.session_frame_count = 0
                self.session_start_time = time.time()
                self.active_telemetry = []
                print(f"🎬 [FRAME RECORDER] Bắt đầu phiên ghi nhận: {self.current_session_id} | Lệnh: {self.current_cmd_name}")
            self.is_recording = True

    def stop_command(self):
        """Tạm dừng ghi nhận khi hết lệnh di chuyển hoặc bước vào khoảng thời gian nghỉ"""
        with self.lock:
            if self.is_recording:
                self.is_recording = False
                print(f"⏸️ [FRAME RECORDER] Tạm dừng ghi nhận (Nghỉ/Hoàn tất lệnh: {self.current_cmd_name}) | Đã lưu: {self.session_frame_count} frames.")
                # Lưu file jsonl cập nhật
                self._save_telemetry_index()

    def record_frame(self, frame_id, rgb_img, depth_np, telemetry):
        """Lưu trữ frame ảnh và siêu dữ liệu phân tích thuật toán khi đang trong quá trình thực thi lệnh"""
        with self.lock:
            if not self.is_recording or self.current_session_dir is None:
                return

            self.session_frame_count += 1
            idx = self.session_frame_count

            # Lưu ảnh màu RGB
            rgb_filename = f"frame_{idx:04d}_id{frame_id}.jpg"
            rgb_path = os.path.join(self.current_session_dir, "rgb", rgb_filename)
            if rgb_img is not None:
                try:
                    cv2.imwrite(rgb_path, rgb_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                except Exception:
                    rgb_filename = None

            # Tạo và lưu ảnh phổ màu Depth (Thermal Colormap)
            depth_filename = f"depth_{idx:04d}_id{frame_id}.jpg"
            depth_path = os.path.join(self.current_session_dir, "depth", depth_filename)
            if depth_np is not None:
                try:
                    if depth_np.dtype == np.uint16:
                        d_mm = depth_np.astype(np.float32)
                    else:
                        d_mm = depth_np * 1000.0
                    d_disp = np.clip(d_mm, 200.0, 3500.0)
                    disp_norm = ((3500.0 - d_disp) / 3300.0 * 255.0).astype(np.uint8)
                    thermal_bgr = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
                    cv2.imwrite(depth_path, thermal_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                except Exception:
                    depth_filename = None

            # Đóng gói bản ghi phân tích thuật toán
            entry = {
                "record_index": idx,
                "frame_id": frame_id,
                "timestamp": round(time.time(), 3),
                "command": self.current_cmd_name,
                "rgb_file": rgb_filename,
                "depth_file": depth_filename,
                "telemetry": telemetry
            }
            self.active_telemetry.append(entry)

            # Ghi nối vào file JSON Lines theo thời gian thực (chống mất dữ liệu nếu mất nguồn)
            jsonl_path = os.path.join(self.current_session_dir, "telemetry.jsonl")
            try:
                with open(jsonl_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(entry, ensure_ascii=False) + "\n")
            except Exception:
                pass

    def _save_telemetry_index(self):
        """Lưu tóm tắt toàn bộ phiên ghi vào telemetry_summary.json"""
        if not self.current_session_dir:
            return
        summary_path = os.path.join(self.current_session_dir, "session_summary.json")
        try:
            summary = {
                "session_id": self.current_session_id,
                "total_frames": self.session_frame_count,
                "start_time": self.session_start_time,
                "last_update": time.time(),
                "duration_s": round(time.time() - self.session_start_time, 2),
                "last_command": self.current_cmd_name
            }
            with open(summary_path, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def get_latest_session_dir(self):
        with self.lock:
            return self.current_session_dir

    def create_session_zip(self, session_id=None):
        """Đóng gói toàn bộ thư mục session gồm ảnh và phân tích JSON thành file ZIP để tải về"""
        with self.lock:
            target_dir = self.current_session_dir
            if session_id:
                candidate = os.path.join(self.base_dir, session_id)
                if os.path.exists(candidate):
                    target_dir = candidate

            if not target_dir or not os.path.exists(target_dir):
                # Tìm session mới nhất trong logs
                sessions = [os.path.join(self.base_dir, d) for d in os.listdir(self.base_dir) if os.path.isdir(os.path.join(self.base_dir, d))]
                if not sessions:
                    return None
                sessions.sort(key=os.path.getmtime, reverse=True)
                target_dir = sessions[0]

            self._save_telemetry_index()
            sess_name = os.path.basename(target_dir)
            zip_filename = f"{sess_name}.zip"
            zip_path = os.path.join(self.base_dir, zip_filename)

            try:
                with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                    for root, _, files in os.walk(target_dir):
                        for file in files:
                            full_path = os.path.join(root, file)
                            rel_path = os.path.relpath(full_path, target_dir)
                            zipf.write(full_path, arcname=os.path.join(sess_name, rel_path))
                print(f"📦 [FRAME RECORDER] Đã đóng gói thành công file ZIP: {zip_path}")
                return zip_path
            except Exception as e:
                print(f"❌ [FRAME RECORDER] Lỗi tạo file ZIP: {e}")
                return None

    def get_session_stats(self):
        with self.lock:
            return {
                "is_recording": self.is_recording,
                "current_session_id": self.current_session_id,
                "command": self.current_cmd_name,
                "frame_count": self.session_frame_count,
                "duration_s": round(time.time() - self.session_start_time, 1) if self.session_start_time > 0 else 0.0
            }
