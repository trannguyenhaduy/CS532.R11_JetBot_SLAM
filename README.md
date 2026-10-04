# CS532.R11 — Autonomous 3D Semantic SLAM on JetBot with OAK-D S2

Hệ thống xe tự hành thông minh kết hợp **Visual SLAM (RTAB-Map)**, **Thị giác máy tính Edge AI (DepthAI Tiny YOLO trên chip VPU Myriad X)**, **Bản đồ ngữ nghĩa 3D (3D Semantic Clustering & Filtering)**, **Trạm điều khiển WebGL Digital Twin (Three.js)** và **Hệ thống đánh giá điểm chuẩn thời gian thực (Real-time Benchmark Scorecard)**.

---

## 👥 Phân công nhiệm vụ nhóm (33% - 33% - 34%)

| Thành viên | Trách nhiệm chính | File bàn giao |
| :--- | :--- | :--- |
| **Thành viên 1** | Điều khiển chuyển động vi sai, Phanh an toàn Virtual Bumper, Bám người PD, Đo pin INA219 | `motor_driver_node.py`, `person_follower_node.py` |
| **Thành viên 2** | Khởi động camera OAK-D S2, Pipeline Tiny YOLO VPU, Lọc tọa độ không gian 3D | `camera_ai.launch`, `spatial_perception_node.py` |
| **Thành viên 3 (Lead)** | Kiến trúc hệ thống, RTAB-Map SLAM, Gom cụm ngữ nghĩa 3D, WebGL Cockpit & Benchmark | `master_system.launch`, `semantic_mapping_node.py`, `slam_web_dashboard.py` |

Các thành viên phát triển module độc lập và tích hợp qua Hợp đồng giao tiếp ROS Topics chuẩn hóa bên dưới.

---

## 📡 Chuẩn giao tiếp dữ liệu (ROS Topic Contracts)

```text
               ┌────────────────────────┐
               │    camera_ai.launch    │
               │ (OAK-D S2 Driver & IMU)│
               └───────────┬────────────┘
                           │ RGB, Depth, IMU
                           ▼
               ┌────────────────────────┐
               │   RTAB-Map SLAM Node   │
               └───────────┬────────────┘
                           │ /rtabmap/odom, /rtabmap/grid_map
                           ▼
  /spatial_objects  ┌────────────────────────┐  /semantic_markers
───────────────────►│ semantic_mapping_node  ├───────────────────► [RViz / Web 3D]
  (VPU Detections)  └────────────────────────┘  (MarkerArray)
                           ▲
                           │ /cmd_vel (Lái xe) & Telemetry
                           ▼
                    ┌────────────────────────┐
                    │  slam_web_dashboard    │ (Port 8080: Three.js WebGL Cockpit
                    └────────────────────────┘  & Real-Time Benchmark HUD)
```

---

## 🚀 Hướng dẫn khởi chạy trên JetBot

```bash
# 1. Đi vào thư mục catkin workspace
cd ~/catkin_ws/src/jetbot_slam

# 2. Cấp quyền thực thi cho các node Python
chmod +x *.py

# 3. Khởi chạy toàn bộ hệ thống bằng 1 lệnh duy nhất
roslaunch jetbot_slam master_system.launch
```

Mở trình duyệt truy cập Web Cockpit: `http://<IP_ROBOT>:8080` (hoặc `http://localhost:8080`).

> 📖 **Xem hướng dẫn chi tiết từng bước cho nhóm:** [HUONG_DAN_CHAY_WEB_JETBOT.md](HUONG_DAN_CHAY_WEB_JETBOT.md)

