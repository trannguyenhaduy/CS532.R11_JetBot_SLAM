---
name: jetbot-slam-semantic
description: >-
  Kỹ sư Visual SLAM (RTAB-Map), Cây biến đổi TF2, Gom cụm bản đồ ngữ nghĩa 3D
  (Euclidean Clustering) và Trạm điều khiển Web Cockpit Digital Twin.
---

# 🗺️ HƯỚNG DẪN AI CHUYÊN SÂU: THÀNH VIÊN 3 — SLAM, NGỮ NGHĨA 3D & DIGITAL TWIN

> **Dành cho:** AI Trợ lý của **Thành viên 3** (Kỹ sư Trưởng SLAM, Ngữ Nghĩa 3D & Tích Hợp Hệ Thống).  
> **Dự án:** JetBot Semantic SLAM & Human-Robot Interaction (OAK-D S2 + Jetson Nano 4GB).  
> **Mục tiêu:** Xây dựng bản đồ hình học 2D/3D (RTAB-Map), gắn nhãn ngữ nghĩa cố định, vận hành Trạm điều khiển Web Digital Twin và duy trì file chạy tổng thể `master_system.launch` cho cả nhóm.

---

## 1. VAI TRÒ & PHẠM VI TRÁCH NHIỆM (34% KHỐI LƯỢNG ĐỒ ÁN)
Bạn phụ trách toàn bộ **Tầng Lập Bản Đồ, Không Gian Ngữ Nghĩa & Trạm Điều Hành (Mapping & Digital Twin)**:
1. **File Launch 1 - `rtabmap_slam.launch`:** Cấu hình RTAB-Map Visual Odometry, lọc nhiễu RayTracing và đóng vòng lặp (Loop Closure).
2. **Node 2 - `semantic_mapping_node.py`:** Tra cứu TF2, thuật toán gom cụm không gian (Euclidean Clustering $d < 0.4\text{m}$) và cắm mốc 3D Marker bền vững.
3. **Node 3 - `slam_web_dashboard.py`:** Trạm điều khiển Web Three.js WebGL 3D, điều khiển phím WASD và Engine đo điểm Benchmark thời gian thực (100 điểm).
4. **File Launch Tổng Hợp - `master_system.launch`:** File chìa khóa gom cả 3 thành viên thành 1 nút bấm duy nhất.

---

## 2. QUY ĐỊNH BÀN GIAO FILE & MÔI TRƯỜNG CHẠY (BẮT BUỘC TUÂN THỦ)

Để khi gộp file vào JetBot chạy được ngay 100%:
* **Thư mục lưu file trên JetBot:**
  - Launch files: `~/catkin_ws/src/jetbot_slam/launch/`
    * `rtabmap_slam.launch`
    * `master_system.launch`
  - Python nodes: `~/catkin_ws/src/jetbot_slam/src/`
    * `semantic_mapping_node.py`
    * `slam_web_dashboard.py`
* **Môi trường & Định dạng code:**
  - Python 3 (`#!/usr/bin/env python3`) trên Ubuntu 18.04 + ROS Melodic.
  - Cấp quyền chạy: `chmod +x *.py`.
  - Phụ thuộc hệ thống: `rtabmap_ros`, `tf2_ros`, `visualization_msgs`, `nav_msgs`.

---

## 3. HỢP ĐỒNG GIAO TIẾP DỮ LIỆU (TOPIC CONTRACTS - CỰC KỲ QUAN TRỌNG)

### A. Dữ Liệu Bạn Nhận Vào (Inputs):
* Từ **Thành viên 2 (Camera AI):**
  - `/stereo_inertial_publisher/color/image` & `camera_info`
  - `/stereo_inertial_publisher/stereo/depth`
  - `/spatial_objects` (`std_msgs/String` dạng JSON danh sách vật thể 3D)
* Từ **Thành viên 1 (Động cơ & Pin):**
  - `/battery_telemetry` (`std_msgs/Float32MultiArray`): Điện áp và % pin hiển thị lên HUD Web.

### B. Dữ Liệu Bạn Xuất Ra (Outputs):
* `/rtabmap/odom` (`nav_msgs/Odometry`): Quỹ đạo dịch chuyển của xe trong không gian.
* `/rtabmap/grid_map` (`nav_msgs/OccupancyGrid`): Bản đồ mặt phẳng 2D sàn nhà.
* `/semantic_markers` (`visualization_msgs/MarkerArray`): Cột mốc chữ nổi 3D và khung dây 3D hiển thị trên RViz/Web.
* `/cmd_vel` (`geometry_msgs/Twist`): Lệnh vận tốc gửi xuống cho Bạn 1 khi người dùng bấm phím W, A, S, D trên giao diện Web.
* `/mode/person_following` (`std_msgs/Bool`): Bật/Tắt chế độ tự động bám người từ nút gạt trên Web.

---

## 4. CHI TIẾT LOGIC THUẬT TOÁN BẮT BUỘC TRONG `semantic_mapping_node.py`

### Bài toán: Tránh tạo 100 nhãn trùng lặp khi xe quay qua quay lại cùng 1 cái ghế.
1. **Phép Chiếu Không Gian (TF2 Lookup):**
   ```python
   # Đổi từ camera_optical_frame sang map frame
   transform = tf_buffer.lookup_transform('map', 'oak_camera_optical_frame', rospy.Time(0), rospy.Duration(0.4))
   p_map = do_transform_point(p_cam, transform)
   ```
2. **Gom Cụm Không Gian (Euclidean Clustering):**
   * Duy trì danh sách `catalog = []`.
   * Mỗi vật thể lưu: `{"name": label, "x": x, "y": y, "z": z, "seen_count": 1}`.
   * Tính khoảng cách Euclid tới các vật thể cùng loại:
     $$d = \sqrt{(x - x_i)^2 + (y - y_i)^2}$$
   * Nếu $d < 0.4\text{ m}$: Cập nhật trung bình trọng số vị trí và `seen_count += 1`.
   * Nếu $d \ge 0.4\text{ m}$: Thêm vật thể mới vào danh mục.
3. **Bộ Lọc Bền Vững (Persistence Filter):**
   * Chỉ khi `seen_count >= 5` mới chính thức sinh ra `Marker` 3D để hiển thị lên bản đồ toàn cục.

---

## 5. FILE CHẠY TỔNG THỂ CHO CẢ NHÓM: `master_system.launch`

Bạn là người viết file này để khi cả nhóm họp chỉ cần **gõ 1 lệnh duy nhất** là chạy toàn bộ xe:

```xml
<launch>
    <!-- 1. CỤM THÀNH VIÊN 1: ĐIỀU KHIỂN & BÁM NGƯỜI -->
    <node pkg="jetbot_slam" type="motor_driver_node.py" name="motor_driver" output="screen"/>
    <node pkg="jetbot_slam" type="person_follower_node.py" name="person_follower" output="screen"/>

    <!-- 2. CỤM THÀNH VIÊN 2: MẮT THẦN CAMERA & SPATIAL AI -->
    <include file="$(find jetbot_slam)/launch/camera_ai.launch"/>
    <node pkg="jetbot_slam" type="spatial_perception_node.py" name="spatial_ai" output="screen"/>

    <!-- 3. CỤM THÀNH VIÊN 3: VISUAL SLAM & BẢN ĐỒ NGỮ NGHĨA -->
    <include file="$(find jetbot_slam)/launch/rtabmap_slam.launch"/>
    <node pkg="jetbot_slam" type="semantic_mapping_node.py" name="semantic_mapping" output="screen"/>

    <!-- 4. WEB DIGITAL TWIN & REAL-TIME BENCHMARK SCORECARD -->
    <node pkg="jetbot_slam" type="slam_web_dashboard.py" name="web_cockpit" output="screen"/>
</launch>
```

---

## 6. QUY TRÌNH TỰ KIỂM THỬ ĐỘC LẬP & BENCHMARK (ĐẠT CHUẨN ĐỒ ÁN)

### Cách Test Độc Lập:
```bash
# Bước 1: Khởi động SLAM
roslaunch jetbot_slam rtabmap_slam.launch

# Bước 2: Khởi động Web Dashboard
python3 ~/catkin_ws/src/jetbot_slam/src/slam_web_dashboard.py

# Bước 3: Giả lập Thành viên 2 gửi tọa độ cái bàn
rostopic pub -1 /spatial_objects std_msgs/String 'data: "[{\"id\":60,\"name\":\"TABLE\",\"x\":0.5,\"z\":2.0}]"'
# -> Kiểm tra: Web hiển thị khung hộp 3D tại đúng vị trí bàn trong phòng.
```

### Tiêu Chuẩn Nghiệm Thu Benchmark (60/100 Điểm Hệ Thống):
- **Tần số Odometry:** $\ge 3.0\text{ Hz}$ ổn định.
- **Số điểm Feature Inliers:** Duy trì $\ge 100$ điểm so khớp giữa các frame.
- **Đóng vòng lặp (Loop Closure):** Khi đi 1 vòng phòng về vị trí cũ, mép tường khớp hoàn toàn, xuất hiện log `Loop closure detected!`.
- **Tổng điểm Benchmark trên Web Cockpit:** Đạt $\ge 82 / 100$ điểm (Xếp loại Giỏi/Xuất Sắc).
