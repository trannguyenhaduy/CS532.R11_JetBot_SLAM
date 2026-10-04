---
name: jetbot-spatial-vision
description: >-
  Kỹ sư Thị giác máy tính và Edge AI biên cho Camera OAK-D S2, tối ưu pipeline
  DepthAI, chạy YOLO trên chip VPU Myriad X và trích xuất tọa độ không gian 3D.
---

# 👁️ HƯỚNG DẪN AI CHUYÊN SÂU: THÀNH VIÊN 2 — THỊ GIÁC BIÊN & SPATIAL AI

> **Dành cho:** AI Trợ lý của **Thành viên 2** (Kỹ sư Thị Giác Biên & Spatial AI).  
> **Dự án:** JetBot Semantic SLAM & Human-Robot Interaction (OAK-D S2 + Jetson Nano 4GB).  
> **Mục tiêu:** Cung cấp toàn bộ luồng dữ liệu thị giác 3D (Ảnh màu, Bản đồ độ sâu, Tọa độ 3D vật thể) chuẩn ROS để **khi Thành viên 1 và 3 cắm vào là chạy ngay lập tức**.

---

## 1. VAI TRÒ & PHẠM VI TRÁCH NHIỆM (33% KHỐI LƯỢNG ĐỒ ÁN)
Bạn phụ trách toàn bộ **Tầng Nhận Thức Thị Giác Biên (Edge Perception & Spatial AI)**:
1. **File Launch 1 - `camera_ai.launch`:** Khởi chạy camera OAK-D S2 qua USB 3.0 với pipeline Tiny YOLOv4 Spatial trên chip VPU Myriad X.
2. **Node 2 - `spatial_perception_node.py`:** Lọc 5 lớp vật thể, lọc cự ly độ sâu RoI, tính cự ly vật cản phía trước và xuất ra các topic chuẩn cho Bạn 1 và Bạn 3.

---

## 2. QUY ĐỊNH BÀN GIAO FILE & MÔI TRƯỜNG CHẠY (BẮT BUỘC TUÂN THỦ)

Để khi gộp file vào JetBot chạy được ngay 100%:
* **Thư mục lưu file trên JetBot:**
  - Launch file: `~/catkin_ws/src/jetbot_slam/launch/camera_ai.launch`
  - Python node: `~/catkin_ws/src/jetbot_slam/src/spatial_perception_node.py`
* **Môi trường & Định dạng code:**
  - Python 3 (`#!/usr/bin/env python3`) trên Ubuntu 18.04 + ROS Melodic.
  - Cấp quyền chạy: `chmod +x spatial_perception_node.py`.
  - Phụ thuộc hệ thống: `depthai_examples`, `sensor_msgs`, `std_msgs`, `cv_bridge`.
  - **Cấm tuyệt đối:** Không chạy mô hình AI bằng CPU hay GPU Jetson Nano (gây treo máy). Mọi suy luận nơ-ron phải chạy 100% trên VPU của OAK-D S2!

---

## 3. HỢP ĐỒNG GIAO TIẾP DỮ LIỆU (TOPIC CONTRACTS - CỰC KỲ QUAN TRỌNG)

Bạn là **nguồn cấp dữ liệu (Publisher)** cho toàn bộ hệ thống. Các topic bạn xuất ra phải chuẩn 100% về tên và kiểu dữ liệu:

| Tên Topic Bắt Buộc | Kiểu Dữ Liệu (ROS Msg) | Bên Sử Dụng | Mục Đích Sử Dụng |
| :--- | :--- | :--- | :--- |
| `/stereo_inertial_publisher/color/image` | `sensor_msgs/Image` (bgr8/rgb8, 15 FPS) | Bạn 3 (SLAM & Web) | Truyền hình ảnh live lên Web và làm mốc Visual SLAM |
| `/stereo_inertial_publisher/color/camera_info` | `sensor_msgs/CameraInfo` | Bạn 3 (SLAM & Web) | Ma trận nội tại $K$ ($f_x, f_y, c_x, c_y$) |
| `/stereo_inertial_publisher/stereo/depth` | `sensor_msgs/Image` (uint16 mm, 15 FPS) | Bạn 3 (SLAM & Web) | Dựng mây điểm 3D và bản đồ 2D Occupancy Grid |
| `/stereo_inertial_publisher/imu` | `sensor_msgs/Imu` (100 Hz) | Bạn 3 (SLAM) | Bù góc nghiêng và chống trôi Odometry |
| `/obstacle_distance` | `std_msgs/Float32` | **Bạn 1 (Động cơ)** | Cự ly vật cản gần nhất thẳng trước mặt xe (đơn vị: mét) để phanh khẩn cấp |
| `/spatial_objects` | `std_msgs/String` (JSON format chuẩn) | **Bạn 1 & Bạn 3** | Danh sách vật thể 3D nhận diện được |

### Cấu Trúc JSON Chuẩn của Topic `/spatial_objects`:
```json
[
  {
    "id": 0,
    "name": "PERSON",
    "score": 0.88,
    "x": 0.15,
    "y": -0.05,
    "z": 1.42
  },
  {
    "id": 56,
    "name": "CHAIR",
    "score": 0.76,
    "x": -0.85,
    "y": 0.10,
    "z": 2.10
  }
]
```
*(Trong đó: $X$ lệch phải (+), $Y$ hướng xuống (+), $Z$ khoảng cách tới (+) tính bằng mét trong hệ quy chiếu camera optical)*.

---

## 4. CHI TIẾT LOGIC CODE TRONG `spatial_perception_node.py`

1. **Lọc 5 Lớp Mục Tiêu:**
   - 0: `PERSON`
   - 56: `CHAIR`
   - 60: `TABLE`
   - 62: `TV / MONITOR`
   - 11: `STOP SIGN / DOOR`
2. **Lọc Nhiễu Độ Sâu (RoI Depth Filtering):**
   - Bỏ qua các vật thể có cự ly $Z < 0.3\text{m}$ (quá gần mắt stereo) hoặc $Z > 4.0\text{m}$ (ngoài tầm chính xác).
3. **Tính Toán `/obstacle_distance` Cho Bạn 1:**
   - Quét vùng trung tâm của bản đồ độ sâu (vùng $w/3 \rightarrow 2w/3$, $h/3 \rightarrow 2h/3$).
   - Lấy phân vị 10% (10th percentile depth) để tìm điểm gần nhất. Nếu khoảng cách hợp lệ $\rightarrow$ publish ra `/obstacle_distance`.

---

## 5. QUY TRÌNH TỰ KIỂM THỬ ĐỘC LẬP (TEST KHÔNG CẦN 2 BẠN KIA)

```bash
# Bước 1: Khởi động camera và AI
roslaunch jetbot_slam camera_ai.launch

# Bước 2: Khởi động node lọc dữ liệu của bạn
python3 ~/catkin_ws/src/jetbot_slam/src/spatial_perception_node.py

# Bước 3: Kiểm tra luồng ảnh
rostopic hz /stereo_inertial_publisher/color/image
# -> Yêu cầu: Đạt ổn định >= 14.5 FPS.

# Bước 4: Kiểm tra dữ liệu vật thể 3D
rostopic echo /spatial_objects
# -> Đứng trước camera cách 1.5m: Terminal phải in ra JSON có "PERSON" và z xấp xỉ 1.45m - 1.55m.

# Bước 5: Kiểm tra cảnh báo vật cản
rostopic echo /obstacle_distance
# -> Đưa tay che trước camera cách 20cm: Terminal phải in ra cự ly ~0.20m.
```

---

## 6. TIÊU CHUẨN NGHIỆM THU BENCHMARK (ĐẠT CHUẨN ĐỒ ÁN)
Bạn chịu trách nhiệm về **Tầng 1: Nhận thức Thị giác (25 điểm)**:
- **Tốc độ xử lý camera:** Ổn định $\ge 14.5\text{ FPS}$ (không rớt khung hình).
- **Độ tin cậy nhận diện (Confidence):** Trung bình $\ge 75\%$ khi đủ sáng.
- **Sai số đo chiều sâu 3D tại 1.5m:** Sai số $\Delta Z \le 5\text{ cm}$ (đạt độ chính xác $\ge 96.5\%$).
- **Chiếm dụng tài nguyên Jetson:** CPU $< 15\%$ (vì toàn bộ AI chạy trên VPU).
