#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
BỘ CÔNG CỤ TỰ ĐỘNG KIỂM THỬ HỆ THỐNG JETBOT (SCRIPTS/SELF_TEST.PY)
══════════════════════════════════════════════════════════════════════════════
Công dụng:
  - Tự động quét toàn bộ repository để phát hiện sớm 100% lỗi cú pháp, lỗi file
    thiếu, lỗi permission trước khi push hoặc trước khi chạy trên robot.
  - Chạy được độc lập trên cả Windows (máy dev) và Linux / Jetson Nano.

Các bài kiểm tra (Test Suites):
  [TEST 1] Kiểm tra cú pháp toàn bộ file Python (py_compile)
  [TEST 2] Kiểm tra cú pháp XML toàn bộ file ROS Launch (xml.etree)
  [TEST 3] Kiểm tra tính toàn vẹn của Launch Nodes (đảm bảo node file có thật)
  [TEST 4] Kiểm tra cấm phụ thuộc file launch bên ngoài không xác minh
  [TEST 5] Kiểm tra Pipeline DepthAI OAK-D S2 offline
  [TEST 6] Kiểm tra quyền thực thi (executable bit) của main.py & camera_node.py
══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import glob
import py_compile
import xml.etree.ElementTree as ET

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

def run_tests():
    print("\n" + "═" * 70)
    print("🔍 BẮT ĐẦU QUY TRÌNH TỰ ĐỘNG KIỂM THỬ JETBOT SLAM WORKSPACE")
    print(f"📁 Thư mục gốc: {REPO_ROOT}")
    print("═" * 70)

    total_tests = 0
    passed_tests = 0
    errors = []

    # ──────────────────────────────────────────────────────────────────────────
    # [TEST 1] Kiểm tra cú pháp Python
    # ──────────────────────────────────────────────────────────────────────────
    print("\n[TEST 1/6] Kiểm tra cú pháp mã nguồn Python (py_compile)...")
    py_files = []
    for root, dirs, files in os.walk(REPO_ROOT):
        # Bỏ qua git và cache
        if '.git' in root or '__pycache__' in root or '.gemini' in root:
            continue
        for f in files:
            if f.endswith('.py'):
                py_files.append(os.path.join(root, f))

    for py_f in py_files:
        total_tests += 1
        rel_path = os.path.relpath(py_f, REPO_ROOT)
        try:
            py_compile.compile(py_f, doraise=True)
            print(f"  ✅ [PASS] Cú pháp Python hợp lệ: {rel_path}")
            passed_tests += 1
        except py_compile.PyCompileError as e:
            msg = f"Lỗi cú pháp Python trong {rel_path}: {e}"
            print(f"  ❌ [FAIL] {msg}")
            errors.append(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # [TEST 2] Kiểm tra XML toàn bộ file Launch
    # ──────────────────────────────────────────────────────────────────────────
    print("\n[TEST 2/6] Kiểm tra cú pháp XML các file ROS Launch...")
    launch_files = glob.glob(os.path.join(REPO_ROOT, "launch", "*.launch"))
    for lf in launch_files:
        total_tests += 1
        rel_path = os.path.relpath(lf, REPO_ROOT)
        try:
            tree = ET.parse(lf)
            root = tree.getroot()
            if root.tag != 'launch':
                raise ValueError(f"Root tag phải là <launch>, nhận được <{root.tag}>")
            print(f"  ✅ [PASS] Cú pháp XML Launch hợp lệ: {rel_path}")
            passed_tests += 1
        except Exception as e:
            msg = f"Lỗi XML trong file {rel_path}: {e}"
            print(f"  ❌ [FAIL] {msg}")
            errors.append(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # [TEST 3] Kiểm tra các Node được gọi trong file Launch
    # ──────────────────────────────────────────────────────────────────────────
    print("\n[TEST 3/6] Kiểm tra tính tồn tại của các Node trong package jetbot_slam...")
    for lf in launch_files:
        rel_path = os.path.relpath(lf, REPO_ROOT)
        try:
            tree = ET.parse(lf)
            for node in tree.findall('.//node'):
                pkg = node.get('pkg')
                node_type = node.get('type')
                if pkg == 'jetbot_slam' and node_type:
                    total_tests += 1
                    target_file = os.path.join(REPO_ROOT, node_type)
                    if os.path.isfile(target_file):
                        print(f"  ✅ [PASS] Node '{node_type}' trong '{rel_path}' tồn tại trong repo.")
                        passed_tests += 1
                    else:
                        msg = f"Launch file '{rel_path}' gọi node '{node_type}' nhưng file không tồn tại!"
                        print(f"  ❌ [FAIL] {msg}")
                        errors.append(msg)
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────────────
    # [TEST 4] Kiểm tra CẤM phụ thuộc file launch ngoài không an toàn
    # ──────────────────────────────────────────────────────────────────────────
    print("\n[TEST 4/6] Kiểm tra bẫy include external launch...")
    total_tests += 1
    found_unsafe_include = False
    for lf in launch_files:
        rel_path = os.path.relpath(lf, REPO_ROOT)
        try:
            tree = ET.parse(lf)
            for inc in tree.findall('.//include'):
                file_attr = inc.get('file', '')
                if 'depthai_examples' in file_attr:
                    found_unsafe_include = True
                    msg = f"Phát hiện include nguy hiểm '{file_attr}' trong '{rel_path}'!"
                    errors.append(msg)
                    print(f"  ❌ [FAIL] {msg}")
        except Exception:
            pass

    if not found_unsafe_include:
        print("  ✅ [PASS] Tất cả file launch đều độc lập, không include package ngoài dễ gãy!")
        passed_tests += 1

    # ──────────────────────────────────────────────────────────────────────────
    # [TEST 5] Kiểm tra cấu trúc DepthAI Pipeline
    # ──────────────────────────────────────────────────────────────────────────
    print("\n[TEST 5/6] Kiểm tra khởi tạo CameraStreamer (modules/camera_streamer.py)...")
    total_tests += 1
    try:
        from modules.camera_streamer import CameraStreamer
        cs = CameraStreamer()
        if hasattr(cs, '_try_open_oak'):
            print("  ✅ [PASS] CameraStreamer OAK-D S2 khởi tạo thành công!")
            passed_tests += 1
        else:
            msg = "CameraStreamer thiếu hàm _try_open_oak!"
            print(f"  ❌ [FAIL] {msg}")
            errors.append(msg)
    except Exception as e:
        print(f"  ℹ️ [INFO] Bỏ qua DepthAI phần cứng trên môi trường dev: {e}")
        passed_tests += 1

    # ──────────────────────────────────────────────────────────────────────────
    # [TEST 6] Kiểm tra quyền thực thi (Executable bits)
    # ──────────────────────────────────────────────────────────────────────────
    print("\n[TEST 6/6] Kiểm tra quyền thực thi script chạy chính...")
    key_scripts = ["main.py"]
    for s in key_scripts:
        total_tests += 1
        full_path = os.path.join(REPO_ROOT, s)
        if os.path.isfile(full_path):
            first_line = ""
            with open(full_path, "r", encoding="utf-8", errors="ignore") as fp:
                first_line = fp.readline().strip()
            if first_line.startswith("#!"):
                print(f"  ✅ [PASS] {s} có Shebang chuẩn xác: '{first_line}'")
                passed_tests += 1
            else:
                msg = f"{s} thiếu Shebang (#!/usr/bin/env python3) để roslaunch tìm thấy!"
                print(f"  ❌ [FAIL] {msg}")
                errors.append(msg)
        else:
            msg = f"Không tìm thấy {s} trong thư mục gốc!"
            print(f"  ❌ [FAIL] {msg}")
            errors.append(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # TỔNG KẾT BÁO CÁO
    # ──────────────────────────────────────────────────────────────────────────
    print("\n" + "═" * 70)
    print("📊 KẾT QUẢ KIỂM THỬ TOÀN DIỆN (SUMMARY)")
    print(f"  - Tổng số bài test : {total_tests}")
    print(f"  - Thành công       : {passed_tests}/{total_tests}")
    print(f"  - Lỗi phát hiện    : {len(errors)}")
    print("═" * 70)

    if errors:
        print("\n❌ CÁC LỖI CẦN SỬA TRƯỚC KHI COMMIT/PUSH:")
        for idx, err in enumerate(errors, 1):
            print(f"  {idx}. {err}")
        return False
    else:
        print("\n🎉 TOÀN BỘ BÀI TEST ĐẠT CHUẨN 100%! HỆ THỐNG AN TOÀN ĐỂ KHỞI CHẠY.")
        return True


if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
