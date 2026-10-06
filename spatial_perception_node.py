#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
JETBOT SPATIAL PERCEPTION & EDGE AI FILTER NODE (ROS MELODIC / NOETIC)
Thành viên 2 — Thị giác Biên & Spatial AI (OAK-D S2 + VPU Myriad X)
══════════════════════════════════════════════════════════════════════════════
- Vai trò & Phạm vi:
  * Thu nhận luồng Depth & Detections từ Camera OAK-D S2 (VPU Myriad X).
  * Lọc đúng 5 lớp đối tượng mục tiêu:
      0: PERSON, 56: CHAIR, 60: TABLE, 62: TV / MONITOR, 11: STOP SIGN / DOOR.
  * Lọc nhiễu cự ly RoI Depth: Chỉ giữ vật thể trong dải [0.3m, 4.0m].
  * Tính cự ly vật cản phía trước (/obstacle_distance):
      Quét vùng trung tâm (w/3 -> 2w/3, h/3 -> 2h/3) và lấy phân vị 10% (10th percentile).
- Hợp đồng Topic xuất (Publishers):
  * /spatial_objects (std_msgs/String): Chuỗi JSON chuẩn danh sách vật thể 3D.
  * /obstacle_distance (std_msgs/Float32): Cự ly vật cản gần nhất trước mặt xe (mét).
- Hợp đồng Topic nhận (Subscribers):
  * /stereo_inertial_publisher/stereo/depth (sensor_msgs/Image): Bản đồ độ sâu uint16 (mm).
  * /stereo_inertial_publisher/color/yolov4_Spatial_detections (SpatialDetectionArray / String).
══════════════════════════════════════════════════════════════════════════════
"""

import sys
import os
import time
import json
import math
import threading
import numpy as np

try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

try:
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String, Float32
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Image = object
    String = object
    Float32 = object

try:
    from depthai_ros_msgs.msg import SpatialDetectionArray
    HAS_DEPTHAI_MSGS = True
except ImportError:
    HAS_DEPTHAI_MSGS = False
    SpatialDetectionArray = object

# ─── BẢNG 5 LỚP MỤC TIÊU THEO YÊU CẦU THÀNH VIÊN 2 ───────────────────────────
TARGET_CLASSES = {
    0: "PERSON",
    56: "CHAIR",
    60: "TABLE",
    62: "TV / MONITOR",
    11: "STOP SIGN / DOOR"
}

# Ánh xạ nhãn văn bản phụ trợ nếu detector xuất dạng tên chuỗi
LABEL_SYNONYMS = {
    "person": (0, "PERSON"),
    "chair": (56, "CHAIR"),
    "couch": (56, "CHAIR"),
    "sofa": (56, "CHAIR"),
    "dining table": (60, "TABLE"),
    "diningtable": (60, "TABLE"),
    "table": (60, "TABLE"),
    "desk": (60, "TABLE"),
    "tv": (62, "TV / MONITOR"),
    "tvmonitor": (62, "TV / MONITOR"),
    "tv/monitor": (62, "TV / MONITOR"),
    "monitor": (62, "TV / MONITOR"),
    "screen": (62, "TV / MONITOR"),
    "stop sign": (11, "STOP SIGN / DOOR"),
    "stopsign": (11, "STOP SIGN / DOOR"),
    "door": (11, "STOP SIGN / DOOR")
}

# Ngưỡng độ sâu an toàn theo hợp đồng
DEPTH_MIN_METERS = 0.30  # Bỏ qua Z < 0.3m (quá gần mắt stereo)
DEPTH_MAX_METERS = 4.00  # Bỏ qua Z > 4.0m (ngoài tầm chính xác của OAK-D S2)


class SpatialPerceptionFilter:
    """Module logic thuần túy (dễ dàng Unit Test độc lập)"""

    @staticmethod
    def filter_target_objects(raw_detections):
        """
        Lọc đúng 5 lớp mục tiêu và lọc cự ly chiều sâu 0.3m <= Z <= 4.0m.
        Chuẩn hóa format JSON theo Hợp đồng Thành viên 2.
        """
        filtered = []
        if not raw_detections or not isinstance(raw_detections, list):
            return filtered

        for det in raw_detections:
            if not isinstance(det, dict):
                continue

            raw_id = det.get("id")
            raw_name = str(det.get("name", "")).strip()

            target_id = None
            target_name = None

            # Kiểm tra theo ID số nguyên
            if raw_id is not None and int(raw_id) in TARGET_CLASSES:
                target_id = int(raw_id)
                target_name = TARGET_CLASSES[target_id]
            else:
                # Kiểm tra theo tên nhãn văn bản
                norm_label = raw_name.lower()
                if norm_label in LABEL_SYNONYMS:
                    target_id, target_name = LABEL_SYNONYMS[norm_label]

            # Bỏ qua nếu không nằm trong 5 lớp chỉ định
            if target_id is None:
                continue

            # Lấy tọa độ 3D và điểm tin cậy
            try:
                x = float(det.get("x", 0.0))
                y = float(det.get("y", 0.0))
                z = float(det.get("z", 0.0))
                score = float(det.get("score", 0.0))
            except (ValueError, TypeError):
                continue

            # 2. Lọc cự ly độ sâu RoI [0.3m -> 4.0m]
            if z < DEPTH_MIN_METERS or z > DEPTH_MAX_METERS:
                continue

            filtered.append({
                "id": target_id,
                "name": target_name,
                "score": round(score, 2),
                "x": round(x, 2),
                "y": round(y, 2),
                "z": round(z, 2)
            })

        return filtered

    @staticmethod
    def calculate_obstacle_distance(depth_frame):
        """
        Quét vùng trung tâm của bản đồ độ sâu (w/3 -> 2w/3, h/3 -> 2h/3).
        Lấy phân vị 10% (10th percentile depth) để tìm điểm gần nhất chống nhiễu hạt.
        Đơn vị trả về: mét.
        """
        if depth_frame is None or not isinstance(depth_frame, np.ndarray):
            return 99.0

        h, w = depth_frame.shape[:2]
        if h < 10 or w < 10:
            return 99.0

        # Cắt ROI trung tâm
        h_start, h_end = int(h / 3.0), int(2.0 * h / 3.0)
        w_start, w_end = int(w / 3.0), int(2.0 * w / 3.0)
        roi = depth_frame[h_start:h_end, w_start:w_end]

        # Chuẩn hóa về mét (uint16 mm -> m)
        if roi.dtype == np.uint16 or roi.max() > 100.0:
            roi_m = roi.astype(np.float32) / 1000.0
        else:
            roi_m = roi.astype(np.float32)

        # Lọc các điểm hợp lệ (loại bỏ điểm mù 0 và ngoài tầm đo)
        valid = roi_m[(roi_m >= 0.15) & (roi_m <= 5.0)]

        if len(valid) >= 15:
            # Lấy phân vị 10% để phát hiện vật cản gần nhất chính xác
            dist_p10 = float(np.percentile(valid, 10))
            return round(dist_p10, 2)

        return 99.0


class SpatialPerceptionNode:
    """ROS Node quản lý luồng nhận thức thị giác và xuất các topic chuẩn"""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest_objects = []
        self.latest_obstacle_dist = 99.0
        self.last_depth_time = 0.0
        self.last_det_time = 0.0

        # Thống kê Benchmark
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.measured_fps = 15.0

        if HAS_ROS:
            rospy.init_node('spatial_perception_node', anonymous=False)

            # Publishers
            self.pub_objects = rospy.Publisher('/spatial_objects', String, queue_size=5)
            self.pub_obstacle = rospy.Publisher('/obstacle_distance', Float32, queue_size=5)

            # Subscribers
            self.sub_depth = rospy.Subscriber(
                '/stereo_inertial_publisher/stereo/depth',
                Image,
                self.depth_callback,
                queue_size=1,
                buff_size=2**24
            )

            # Subscriber nhận Detections từ ROS Driver DepthAI
            if HAS_DEPTHAI_MSGS:
                rospy.Subscriber(
                    '/stereo_inertial_publisher/color/yolov4_Spatial_detections',
                    SpatialDetectionArray,
                    self.depthai_detections_callback,
                    queue_size=2
                )

            # Kênh nhận fallback nếu node phát hiện xuất JSON trực tiếp
            rospy.Subscriber(
                '/stereo_inertial_publisher/color/raw_detections',
                String,
                self.raw_json_detections_callback,
                queue_size=2
            )

            # Timer publish định kỳ 15 Hz để đảm bảo topic luôn tươi mới
            self.timer = rospy.Timer(rospy.Duration(1.0 / 15.0), self.publish_loop)
            rospy.loginfo("👁️ [SPATIAL AI] Node spatial_perception_node đã khởi chạy thành công!")
            rospy.loginfo("   ├─ Target classes: PERSON(0), CHAIR(56), TABLE(60), TV(62), STOP_SIGN(11)")
            rospy.loginfo("   ├─ RoI depth: [0.30m, 4.00m]")
            rospy.loginfo("   └─ Ready to publish /spatial_objects & /obstacle_distance")

    def depth_callback(self, msg: Image):
        """Xử lý bản đồ độ sâu để tính /obstacle_distance cho Thành viên 1"""
        try:
            w, h = msg.width, msg.height
            if msg.encoding in ['16UC1', 'mono16']:
                depth_np = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
            else:
                depth_np = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))

            dist = SpatialPerceptionFilter.calculate_obstacle_distance(depth_np)

            with self.lock:
                self.latest_obstacle_dist = dist
                self.last_depth_time = time.time()
                self.fps_counter += 1

        except Exception as e:
            if HAS_ROS:
                rospy.logerr_throttle(5.0, f"[SPATIAL AI] Lỗi giải mã depth image: {e}")

    def depthai_detections_callback(self, msg):
        """Callback đọc dữ liệu SpatialDetectionArray từ OAK-D S2 Myriad X"""
        raw_list = []
        try:
            for det in getattr(msg, 'detections', []):
                for res in getattr(det, 'results', []):
                    pos = getattr(det, 'position', None)
                    if pos:
                        cid = getattr(res, 'id', 0)
                        raw_list.append({
                            "id": cid,
                            "name": TARGET_CLASSES.get(cid, getattr(res, 'label', 'OBJ')),
                            "score": float(getattr(res, 'score', 0.8)),
                            "x": float(pos.x),
                            "y": float(pos.y),
                            "z": float(pos.z)
                        })

            filtered = SpatialPerceptionFilter.filter_target_objects(raw_list)
            with self.lock:
                self.latest_objects = filtered
                self.last_det_time = time.time()

        except Exception as e:
            if HAS_ROS:
                rospy.logerr_throttle(5.0, f"[SPATIAL AI] Lỗi xử lý detections: {e}")

    def raw_json_detections_callback(self, msg: String):
        """Nhận diện danh sách raw JSON nếu driver phát thô dạng String"""
        try:
            parsed = json.loads(msg.data)
            if isinstance(parsed, list):
                filtered = SpatialPerceptionFilter.filter_target_objects(parsed)
                with self.lock:
                    self.latest_objects = filtered
                    self.last_det_time = time.time()
        except Exception:
            pass

    def publish_loop(self, event=None):
        """Phát dữ liệu ra các topic chuẩn cho Bạn 1 và Bạn 3"""
        if not HAS_ROS:
            return

        with self.lock:
            objs = list(self.latest_objects)
            obs_dist = self.latest_obstacle_dist

        # 1. Topic /spatial_objects (std_msgs/String - JSON)
        json_str = json.dumps(objs)
        msg_obj = String()
        msg_obj.data = json_str
        self.pub_objects.publish(msg_obj)

        # 2. Topic /obstacle_distance (std_msgs/Float32)
        msg_obs = Float32()
        msg_obs.data = float(obs_dist)
        self.pub_obstacle.publish(msg_obs)

    def run_standalone_test(self):
        """Chế độ tự kiểm thử độc lập (không cần ROS hoặc trên máy tính)"""
        c_cyan = "\033[1;36m"
        c_green = "\033[1;32m"
        c_yellow = "\033[1;33m"
        c_red = "\033[1;31m"
        c_reset = "\033[0m"

        print(f"\n{c_cyan}══════════════════════════════════════════════════════════════════════════════{c_reset}")
        print(f"{c_yellow}👁️ [THÀNH VIÊN 2] KIỂM THỬ ĐỘC LẬP SPATIAL PERCEPTION NODE{c_reset}")
        print(f"  ├─ Hỗ trợ lọc 5 lớp mục tiêu: PERSON(0), CHAIR(56), TABLE(60), TV(62), STOP_SIGN(11)")
        print(f"  ├─ Lọc cự ly RoI [0.3m -> 4.0m] và tính phân vị 10% /obstacle_distance")
        print(f"{c_cyan}══════════════════════════════════════════════════════════════════════════════{c_reset}\n")

        # Test dữ liệu mẫu
        sample_raw = [
            {"id": 0, "name": "PERSON", "score": 0.88, "x": 0.15, "y": -0.05, "z": 1.42},
            {"id": 56, "name": "CHAIR", "score": 0.76, "x": -0.85, "y": 0.10, "z": 2.10},
            {"id": 2, "name": "CAR", "score": 0.90, "x": 0.0, "y": 0.0, "z": 1.50},  # Sẽ bị lọc bỏ vì không thuộc 5 lớp
            {"id": 0, "name": "PERSON", "score": 0.85, "x": 0.10, "y": 0.0, "z": 5.20},  # Sẽ bị lọc bỏ vì Z > 4.0m
            {"id": 62, "name": "TV", "score": 0.81, "x": 1.10, "y": -0.20, "z": 2.80}
        ]

        filtered = SpatialPerceptionFilter.filter_target_objects(sample_raw)
        print(f"{c_green}✅ [TEST 1] KẾT QUẢ LỌC VẬT THỂ (/spatial_objects):{c_reset}")
        print(json.dumps(filtered, indent=2))

        # Test tính khoảng cách vật cản trên ảnh depth giả lập
        dummy_depth = np.full((360, 480), 1800, dtype=np.uint16)  # 1.8m
        dummy_depth[140:180, 200:280] = 650  # Có vật cản tại 0.65m ngay trung tâm
        dist = SpatialPerceptionFilter.calculate_obstacle_distance(dummy_depth)
        print(f"\n{c_green}✅ [TEST 2] TÍNH TOÁN /obstacle_distance:{c_reset} {dist} m (Kỳ vọng: ~0.65m)")

        print(f"\n{c_yellow}Đang chạy vòng lặp mô phỏng. Nhấn Ctrl+C để dừng...{c_reset}\n")
        step = 0
        try:
            while True:
                time.sleep(1.0)
                step += 1
                dist_sim = round(1.45 + 0.1 * math.sin(step * 0.5), 2)
                print(f"[{step:02d}s] /spatial_objects: {len(filtered)} objects | /obstacle_distance: {dist_sim}m")
        except KeyboardInterrupt:
            print("\nĐã dừng kiểm thử độc lập.")


def main():
    if HAS_ROS:
        node = SpatialPerceptionNode()
        rospy.spin()
    else:
        node = SpatialPerceptionNode()
        node.run_standalone_test()


if __name__ == '__main__':
    main()
