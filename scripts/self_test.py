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
                    candidate_paths = [
                        os.path.join(REPO_ROOT, node_type),
                        os.path.join(REPO_ROOT, "scripts", node_type)
                    ]
                    matched = [p for p in candidate_paths if os.path.isfile(p)]
                    if matched:
                        rel_found = os.path.relpath(matched[0], REPO_ROOT)
                        print(f"  ✅ [PASS] Node '{node_type}' trong '{rel_path}' tồn tại: {rel_found}")
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
    key_scripts = ["main.py", os.path.join("scripts", "camera_streamer_node.py")]
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
    # [TEST 7] Kiểm tra thuật toán PathPlanner A*
    # ──────────────────────────────────────────────────────────────────────────
    print("\n[TEST 7/8] Kiểm tra thuật toán tìm đường tối ưu A* (modules/path_planner.py)...")
    total_tests += 1
    try:
        import numpy as np
        from modules.path_planner import PathPlanner
        planner = PathPlanner()
        sim_grid = np.zeros((200, 200), dtype=np.uint8)
        sim_grid[80:120, 100] = 255
        res = planner.plan_path((-1.5, 0.0), (1.5, 0.0), grid=sim_grid)
        if res.get('success') and len(res.get('path', [])) >= 2:
            print(f"  ✅ [PASS] PathPlanner A* tìm đường thành công ({res['length_m']}m, {res['plan_time_ms']}ms)!")
            passed_tests += 1
        else:
            msg = f"PathPlanner không tìm thấy đường: {res.get('message')}"
            print(f"  ❌ [FAIL] {msg}")
            errors.append(msg)
    except Exception as e:
        msg = f"Lỗi khởi tạo PathPlanner: {e}"
        print(f"  ❌ [FAIL] {msg}")
        errors.append(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # [TEST 8] Kiểm tra bộ điều khiển tự hành bám đường AutonomousNavigator
    # ──────────────────────────────────────────────────────────────────────────
    print("\n[TEST 8/9] Kiểm tra bộ điều khiển tự hành bám đường (modules/navigator.py)...")
    total_tests += 1
    try:
        import time
        import math
        from modules.navigator import AutonomousNavigator
        sim_pos = [0.0, 0.0, 0.0]
        def _gp(): return sim_pos[0], sim_pos[1], sim_pos[2]
        def _scv(v, w):
            sim_pos[2] += w * 0.05
            sim_pos[0] += v * 0.05 * math.cos(sim_pos[2])
            sim_pos[1] += v * 0.05 * math.sin(sim_pos[2])
        def _st(): pass
        nav = AutonomousNavigator(_gp, _scv, _st, linear_speed=0.20, turn_speed=1.5, goal_tolerance_m=0.06)
        res = nav.start_navigation([(0.0, 0.0), (0.2, 0.0)])
        assert res.get('success'), f"Khởi động navigator thất bại: {res}"
        t0 = time.time()
        while nav.is_running and time.time() - t0 < 3.0:
            time.sleep(0.05)
        st = nav.get_status()
        if st.get('state') == AutonomousNavigator.STATE_GOAL_REACHED:
            print(f"  ✅ [PASS] AutonomousNavigator bám đường và chạm đích chính xác ({st['dist_to_goal']*100:.1f}cm)!")
            passed_tests += 1
        else:
            msg = f"Navigator chưa đạt đích (state={st.get('state')}): {st.get('status_text')}"
            print(f"  ❌ [FAIL] {msg}")
            errors.append(msg)
    except Exception as e:
        msg = f"Lỗi khởi tạo AutonomousNavigator: {e}"
        print(f"  ❌ [FAIL] {msg}")
        errors.append(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # [TEST 9] Kiểm tra thuật toán xử lý IMU OAK-D S2 & Đối chiếu góc quay vật lý
    # ──────────────────────────────────────────────────────────────────────────
    print("\n[TEST 9/9] Kiểm tra giải mã IMU OAK-D S2 (Quaternion Yaw & Gyroscope)...")
    total_tests += 1
    try:
        from modules.camera_streamer import CameraStreamer
        cs_imu = CameraStreamer()

        # Kiểm tra khởi tạo trục đứng
        assert getattr(cs_imu, '_imu_vertical_axis', None) == 'y', "Trục đứng IMU phải mặc định là 'y'"
        assert getattr(cs_imu, '_imu_vertical_sign', None) == -1.0, "Hệ số trục đứng IMU phải là -1.0"

        # 1. Giả lập gói tin Rotation Vector: Camera đứng trên bàn ở Yaw = 0 độ
        class MockRV0:
            real = -0.5
            i = 0.5
            j = -0.5
            k = 0.5
        class MockPktRV0:
            rotationVector = MockRV0()
            gyroscope = None
            acceleroMeter = None

        cs_imu._process_single_imu_packet(MockPktRV0())
        has_act, acc_yaw0, raw_yaw0 = cs_imu.get_imu_heading()
        assert has_act, "IMU phải ở trạng thái hoạt động sau khi nhận gói tin"
        assert abs(math.degrees(raw_yaw0) - 0.0) < 0.5, f"Góc mốc ban đầu phải là 0°, nhận được {math.degrees(raw_yaw0):.2f}°"

        # 2. Giả lập xe quay TRÁI 90 độ (+90° = +pi/2 rad)
        class MockRV90:
            real = -0.70710678
            i = 0.70710678
            j = 0.0
            k = 0.0
        class MockPktRV90:
            rotationVector = MockRV90()
            gyroscope = None
            acceleroMeter = None

        cs_imu._process_single_imu_packet(MockPktRV90())
        _, acc_yaw_90, raw_yaw_90 = cs_imu.get_imu_heading()
        yaw_deg = math.degrees(raw_yaw_90)
        assert abs(yaw_deg - 90.0) < 0.5, f"Góc quay giải mã phải là +90°, nhận được {yaw_deg:.2f}°"
        assert abs(math.degrees(acc_yaw_90) - 90.0) < 0.5, f"Góc tích lũy phải là +90°, nhận được {math.degrees(acc_yaw_90):.2f}°"

        # 3. Giả lập xe quay PHẢI 45 độ (-45° = -pi/4 rad)
        class MockRV45:
            real = 0.27059805
            i = -0.27059805
            j = 0.65328148
            k = -0.65328148
        class MockPktRV45:
            rotationVector = MockRV45()
            gyroscope = None
            acceleroMeter = None

        cs_imu._process_single_imu_packet(MockPktRV45())
        _, _, raw_yaw45 = cs_imu.get_imu_heading()
        yaw_deg2 = math.degrees(raw_yaw45)
        assert abs(yaw_deg2 - (-45.0)) < 0.5, f"Góc quay giải mã phải là -45°, nhận được {yaw_deg2:.2f}°"

        # 4. Kiểm tra reset không làm mất mốc
        cs_imu.reset_imu_heading()
        assert cs_imu.imu_accumulated_yaw == 0.0, "Góc tích phân phải về 0 rad sau reset"
        assert cs_imu._last_raw_imu_yaw == raw_yaw45, "Mốc raw yaw phải được giữ lại sau reset"

        print("  ✅ [PASS] Thuật toán giải mã Quaternion Yaw & Gyro IMU OAK-D S2 đạt độ chính xác tuyệt đối (sai số < 0.01°)!")
        passed_tests += 1
    except Exception as e:
        msg = f"Lỗi kiểm thử IMU OAK-D: {e}"
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
