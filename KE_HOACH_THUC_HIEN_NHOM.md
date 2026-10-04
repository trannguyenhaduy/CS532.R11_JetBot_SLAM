# 🚀 KẾ HOẠCH HÀNH ĐỘNG CHI TIẾT & BẢN GIAO KÈO TÍCH HỢP HỆ THỐNG
## DỰ ÁN: JETBOT 3D SEMANTIC SLAM + OAK-D S2 (3 THÀNH VIÊN)

> **Mô hình làm việc:** Phân tán từ xa (Mỗi thành viên phụ trách đúng file của mình, giao tiếp 100% qua chuẩn ROS Topic).  
> **Cam kết tích hợp:** Đúng tên file, đúng đường dẫn `catkin_ws`, đúng định dạng dữ liệu $\rightarrow$ Ghép vào là chạy ngay với 1 lệnh `master_system.launch`.  
> **Quy ước đánh dấu tiến độ:**  
> - `[x]` : Đã hoàn thành và nghiệm thu đạt chuẩn  
> - `[/]` : Đang thực hiện / Cần tinh chỉnh  
> - `[ ]` : Chưa thực hiện  

---

## ⚖️ PHÂN CHIA NHIỆM VỤ CÂN BẰNG (TAM MÃ: 33% - 33% - 34%)

```
                                  [MASTER_SYSTEM.LAUNCH]
                                             │
         ┌───────────────────────────────────┼───────────────────────────────────┐
         ▼                                   ▼                                   ▼
┌──────────────────┐               ┌──────────────────┐               ┌──────────────────┐
│   THÀNH VIÊN 1   │               │   THÀNH VIÊN 2   │               │   THÀNH VIÊN 3   │
│ ĐIỀU KHIỂN & HRI │               │ THỊ GIÁC & AI    │               │ SLAM & DIGITAL   │
│  (TỰ HÀNH & BÁM) │               │   (EDGE VPU)     │               │      TWIN        │
└────────┬─────────┘               └────────┬─────────┘               └────────┬─────────┘
         │                                  │                                  │
 ├─ motor_driver_node.py            ├─ camera_ai.launch                ├─ rtabmap_slam.launch
 ├─ person_follower_node.py         ├─ spatial_perception_node.py      ├─ semantic_mapping_node.py
 └─ Phanh khẩn cấp Collision        └─ Ma trận đo sâu & Conf           └─ Web Cockpit & Benchmark
```

---

## 📑 BẢN GIAO KÈO TÍCH HỢP (TOPIC INTERFACE CONTRACT)

3 bạn làm việc từ xa chỉ cần đảm bảo **đúng tên Topic và kiểu dữ liệu** sau:

| Tên Topic | Kiểu ROS Msg | Bên Gửi (Publisher) | Bên Nhận (Subscriber) | Ý Nghĩa / Mục Đích |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Bạn 3 (Web) hoặc Bạn 1 (Follower) | **Bạn 1 (Motor Driver)** | Lệnh vận tốc dài $v$ và góc $\omega$ điều khiển bánh xe |
| `/obstacle_distance` | `std_msgs/Float32` | **Bạn 2 (Camera AI)** | **Bạn 1 (Phanh an toàn)** | Cự ly vật cản phía trước mặt xe (m) để tự phanh dừng |
| `/spatial_objects` | `std_msgs/String` (JSON) | **Bạn 2 (Camera AI)** | **Bạn 1 & Bạn 3** | Tọa độ 3D $[X, Y, Z]$ của Người, Bàn, Ghế, Cửa |
| `/battery_telemetry` | `std_msgs/Float32MultiArray` | **Bạn 1 (Đo pin)** | **Bạn 3 (Web HUD)** | Điện áp $[V, \%]$ hiển thị lên màn hình trạm điều khiển |
| `/rtabmap/odom` | `nav_msgs/Odometry` | **Bạn 3 (Visual SLAM)** | Hệ thống SLAM / Web | Quỹ đạo di chuyển và vị trí của robot trong phòng |
| `/rtabmap/grid_map` | `nav_msgs/OccupancyGrid` | **Bạn 3 (Visual SLAM)** | Web Cockpit | Bản đồ mặt bằng sàn nhà 2D |
| `/semantic_markers` | `visualization_msgs/MarkerArray`| **Bạn 3 (Gom cụm)** | Web / RViz | Cột mốc chữ nổi 3D Bàn, Ghế, Người trên bản đồ |
| `/mode/person_following` | `std_msgs/Bool` | **Bạn 3 (Web nút bấm)** | **Bạn 1 (Follower)** | Bật / Tắt chế độ tự động bám theo người |

---

## 📋 CHECKLIST CHI TIẾT TỪNG THÀNH VIÊN

### 👤 THÀNH VIÊN 1: KỸ SƯ ĐIỀU KHIỂN TỰ HÀNH & HRI (Xem chi tiết: `skill_tv1.md`)
*File bàn giao:* `~/catkin_ws/src/jetbot_slam/src/motor_driver_node.py` và `person_follower_node.py`
- [x] **1.1:** Khắc phục lỗi thanh ghi I2C PCA9685/TB6612.
- [x] **1.2:** Viết cấu trúc vi phân động học bánh xe nhận `/cmd_vel`.
- [x] **1.3:** Watchdog bảo vệ an toàn 0.5s tự phanh khi rớt mạng WiFi.
- [/] **1.4 (ĐANG LÀM):** Tích hợp thuật toán phanh khẩn cấp (Virtual Bumper) khi `/obstacle_distance < 0.25m`.
- [ ] **1.5 (TIẾP THEO):** Viết node `person_follower_node.py` với bộ điều khiển PD bám theo người.
- [ ] **1.6 (TIẾP THEO):** Đọc cảm biến pin INA219 xuất topic `/battery_telemetry`.

---

### 👤 THÀNH VIÊN 2: KỸ SƯ THỊ GIÁC BIÊN & SPATIAL AI (Xem chi tiết: `skill_tv2.md`)
*File bàn giao:* `~/catkin_ws/src/jetbot_slam/launch/camera_ai.launch` và `src/spatial_perception_node.py`
- [x] **2.1:** Kết nối OAK-D S2 qua USB 3.0 tốc độ cao (5 Gbps).
- [x] **2.2:** Chạy mạng Tiny YOLOv4 Spatial trên chip VPU Myriad X qua `stereo_inertial_node.launch`.
- [/] **2.3 (ĐANG LÀM):** Viết node `spatial_perception_node.py` lọc 5 lớp (Bàn, Ghế, Người, Cửa, Biển báo).
- [ ] **2.4 (TIẾP THEO):** Lọc cự ly độ sâu $[0.3\text{m}, 4.0\text{m}]$ và xuất topic chuẩn JSON `/spatial_objects`.
- [ ] **2.5 (TIẾP THEO):** Tính toán cự ly vật cản gần nhất xuất topic `/obstacle_distance` cho Bạn 1.
- [ ] **2.6 (TIẾP THEO):** Thực nghiệm đo sai số tại 1.0m, 1.5m, 2.0m, 2.5m lập bảng số liệu cho luận văn.

---

### 👤 THÀNH VIÊN 3: KỸ SƯ SLAM, NGỮ NGHĨA 3D & DIGITAL TWIN (Xem chi tiết: `skill_tv3.md`)
*File bàn giao:* `~/catkin_ws/src/jetbot_slam/launch/rtabmap_slam.launch`, `master_system.launch` và `src/semantic_mapping_node.py`, `src/slam_web_dashboard.py`
- [x] **3.1:** Thiết lập cây biến đổi tọa độ chuẩn TF2: `base_link` $\rightarrow$ `oak-d-base-frame`.
- [x] **3.2:** Cấu hình RTAB-Map Visual Odometry đạt inliers $\approx 140$, tần số $\approx 3.5\text{ Hz}$.
- [x] **3.3:** Lọc sạch bản đồ 2D với RayTracing và NoiseFiltering.
- [x] **3.4:** Xây dựng Web Cockpit Three.js WebGL 3D và nhúng Engine Benchmark chấm điểm (Thang 100).
- [/] **3.5 (ĐANG LÀM):** Lập trình thuật toán gom cụm Euclidean Clustering ($d < 0.4\text{m}$) chống trùng nhãn 3D.
- [ ] **3.6 (TIẾP THEO):** Viết file `master_system.launch` gom cả 3 cụm của 3 bạn thành 1 lệnh khởi động duy nhất.
- [ ] **3.7 (TIẾP THEO):** Test đóng vòng lặp (Loop Closure) khi xe đi trọn vẹn 1 vòng phòng.

---

## 🏆 HỆ THỐNG ĐIỂM CHUẨN BENCHMARK THỜI GIAN THỰC (THANG 100 ĐIỂM)

| Tầng Đánh Giá | Thành Viên Phụ Trách | Điểm Tối Đa | Tiêu Chuẩn Nghiệm Thu ĐẠT |
| :--- | :---: | :---: | :--- |
| **1. Nhận Thức Thị Giác (Perception)** | **Bạn 2** | **25 pts** | Camera $\ge 14.5\text{ FPS}$, Conf $\ge 75\%$, Sai số đo 3D $< 5\%$. |
| **2. Bản Đồ SLAM Hình Học (SLAM Core)** | **Bạn 3** | **35 pts** | VO $\ge 3.0\text{ Hz}$, Inliers $> 100$, Loop Closure khớp mép tường. |
| **3. Không Gian Ngữ Nghĩa 3D (Semantic)**| **Bạn 3** | **25 pts** | Nhãn 3D không nhấp nháy, ổn định $\Delta E < 10\text{cm}$, TF trễ $< 40\text{ms}$. |
| **4. Sức Khỏe Phần Cứng & An Toàn** | **Bạn 1** | **15 pts** | Watchdog phanh $\le 0.5\text{s}$, Pin $> 10\text{V}$, Phanh dừng cách vật $\ge 20\text{cm}$. |
| **TỔNG ĐIỂM TOÀN HỆ THỐNG** | **CẢ NHÓM** | **100 pts** | **Mục tiêu bảo vệ trước Hội đồng: $\ge 85 / 100$ điểm** |

---

## 🚀 HƯỚNG DẪN GỘP CODE VÀO JETBOT TRONG 10 GIÂY

Khi 3 bạn gửi file hoàn chỉnh qua Git hoặc Zalo, chỉ cần copy thả vào thư mục tương ứng trên JetBot:
```bash
# Thả file của Bạn 1 và 2 và 3 vào đúng thư mục:
cp motor_driver_node.py person_follower_node.py spatial_perception_node.py semantic_mapping_node.py slam_web_dashboard.py ~/catkin_ws/src/jetbot_slam/src/
cp camera_ai.launch rtabmap_slam.launch master_system.launch ~/catkin_ws/src/jetbot_slam/launch/

# Cấp quyền thực thi:
chmod +x ~/catkin_ws/src/jetbot_slam/src/*.py

# CHẠY TOÀN BỘ XE CHỈ VỚI ĐÚNG 1 LỆNH DUY NHẤT:
roslaunch jetbot_slam master_system.launch
```
Mở trình duyệt vào `http://192.168.168.154:8080` để điều khiển và theo dõi chấm điểm!
