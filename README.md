# 🤖 CS532 — HỆ THỐNG XE TỰ HÀNH JETBOT TÍCH HỢP OAK-D S2, VISUAL SEMANTIC SLAM & WEB COCKPIT 3D

> **Môn học:** CS532 — Thị giác máy tính trong tương tác người–máy  
> **Nền tảng phần cứng:** NVIDIA Jetson Nano 4GB (JetPack 4.5 / Ubuntu 18.04 / ROS Melodic / Python 3)  
> **Cảm biến thị giác:** Luxonis OAK-D S2 (DepthAI v2/v3, Intel Movidius Myriad X VPU)  
> **Giao diện điều khiển:** WebGL 3D Digital Twin (Three.js, Zero-Latency Streaming @ Port 8080)

---

## 📖 1. TỔNG QUAN DỰ ÁN

Dự án phát triển hệ thống robot tự hành thông minh dựa trên nền tảng **Waveshare JetBot**, kết hợp sức mạnh tính toán biên Edge AI của camera **Luxonis OAK-D S2**. Hệ thống tích hợp toàn diện:
- **Thị giác không gian 3D (Spatial AI):** Chạy mô hình Deep Learning trực tiếp trên chip VPU Myriad X, đo đạc tọa độ thực tế $(X, Y, Z)$ tính bằng mét ở tốc độ khung hình **60 FPS** với tải CPU Jetson Nano $\le 15\%$.
- **Phanh an toàn khẩn cấp (Virtual Bumper):** Tự động phát hiện vật cản ở cự ly gần ($< 25\text{cm}$), ngắt lệnh tiến tức thời nhưng luôn cho phép lùi và quay đầu thoát hiểm, tích hợp bộ lọc chống dính phanh giả trên sàn phẳng.
- **Tương tác Người–Máy (HRI - Person Follower):** Tự hành bám theo mục tiêu người qua thuật toán điều khiển PD vi sai, có cơ chế nhường quyền lái tay ưu tiên tuyệt đối.
- **Giám sát Năng lượng thời gian thực:** Đọc trực tiếp cảm biến I2C INA219 (Điện áp V, Dòng A, Công suất W, Tỉ lệ Pin %).
- **Trạm chỉ huy Web Cockpit 3D:** Giám sát camera kép (Camera AI Detect & Camera Nhiệt độ sâu), điều khiển xe mượt mà qua phím `W-A-S-D` hoặc Joystick cảm ứng, hiển thị bản đồ Occupancy Grid 2D và đám mây điểm 3D.

---

## ⚡ 2. CÁC TÍNH NĂNG NỔI BẬT

| Tính năng | Công nghệ cốt lõi | Hiệu năng / Đặc tả |
| :--- | :--- | :--- |
| **Edge AI Vision** | DepthAI MobileNet-SSD / YOLO trên Myriad X VPU | 60 FPS, độ trễ $< 20\text{ms}$, nhận diện 20+ lớp COCO |
| **Phanh An Toàn** | Stereo Depth RoI Filter + 15th Percentile | Phanh cứng $< 25\text{cm}$, lọc sạch mặt sàn, cho phép lùi |
| **Động Cơ Vi Sai** | Cầu H TB6612 + PCA9685 I2C (`0x60`) @ 1600Hz | Giới hạn trần an toàn PWM $\le 35\%$, deadband $\le 16\%$ |
| **Tự Hành Bám Người** | PD Closed-Loop Control + Temporal Tracker | Giữ cự ly $1.0\text{m} - 1.4\text{m}$, tự dừng khi mất dấu |
| **Bản Đồ Ngữ Nghĩa** | 3D Spatial Clustering + 2D Occupancy Grid | Xuất bản đồ chiếm dụng thời gian thực, tích hợp RTAB-Map |
| **Web Cockpit 3D** | Three.js Digital Twin + Web Audio + REST/WebSocket | Port `8080`, hỗ trợ mọi trình duyệt điện thoại/laptop |
| **Đo Pin INA219** | Direct SMBus I2C (`0x41`) | Cập nhật 1 Hz, cảnh báo pin yếu tự động |

---

## 📁 3. CẤU TRÚC THƯ MỤC DỰ ÁN

```text
d:\Robot (catkin_ws/src/jetbot_slam)
├── config.py                     # Cấu hình trung tâm: Feature Flags (ENABLE_*), ngưỡng phanh, PID
├── main.py                       # Điểm khởi chạy chính: Điều phối toàn bộ các module & Web Cockpit
├── CMakeLists.txt                # Cấu hình build package ROS Melodic
├── package.xml                   # Khai báo phụ thuộc package ROS
├── README.md                     # Tài liệu hướng dẫn dự án
│
├── modules/                      # THƯ MỤC CHỨA CÁC MODULE ĐỘC LẬP (MODULAR ARCHITECTURE)
│   ├── __init__.py               # Khai báo package modules
│   ├── motor_controller.py       # Module 1: Điều khiển động cơ TB6612/PCA9685, lái vi sai WASD
│   ├── battery_monitor.py        # Module 2: Đo đạc điện áp, dòng điện, công suất pin qua INA219
│   ├── camera_streamer.py        # Module 3: Luồng Camera OAK-D S2, VPU AI, Camera nhiệt & ROS Pub
│   ├── spatial_detector.py       # Module 4: Bộ suy luận không gian 3D, HOG People, Spatial Cluster
│   ├── person_tracker.py         # Module 5: Tự hành bám người (HRI) với bộ điều khiển vi sai PD
│   ├── emergency_brake.py        # Module 6: Hệ thống phanh khẩn cấp Virtual Bumper lọc mặt sàn
│   ├── semantic_mapper.py        # Module 7: Bản đồ ngữ nghĩa 3D thời gian thực & LaserScan 2D
│   ├── web_server.py             # Module 8: Web Server HTTP/MJPEG điều khiển xe tại cổng 8080
│   └── templates/
│       └── cockpit.html          # Giao diện Web Cockpit Three.js Digital Twin tối tân
│
├── launch/                       # CÁC FILE KHỞI CHẠY ROS LAUNCH
│   ├── camera_ai.launch          # Launch driver OAK-D S2 ROS Publisher + TF base_to_camera
│   └── master.launch             # Launch toàn bộ hệ thống qua main.py + TF
│
└── scripts/                      # SCRIPT KIỂM THỬ TỰ ĐỘNG
    └── self_test.py              # Suite kiểm thử tự động 52 bài test xác thực tính toàn vẹn hệ thống
```

---

## 🚀 4. HƯỚNG DẪN KHỞI CHẠY TRÊN JETBOT

### 4.1. Chuẩn bị môi trường trên Jetson Nano
```bash
cd ~/catkin_ws/src/jetbot_slam
git fetch origin && git reset --hard origin/main
chmod +x main.py scripts/self_test.py
```

### 4.2. Khởi chạy hệ thống

#### 🟢 Cách 1: Khuyên Dùng — 1 Terminal Duy Nhất (Tối ưu 60 FPS, Không Trễ)
Chạy trực tiếp điều phối trung tâm bằng Python 3:
```bash
python3 main.py
```
* **Ưu điểm:** Camera OAK-D S2 đưa dữ liệu trực tiếp vào RAM (Zero-Copy), VPU MobileNet-SSD tự chạy trên chip Myriad X, tải CPU cực thấp ($\le 15\%$), đạt tối đa **60 FPS**.
* **Tự động kích hoạt:** Động cơ, Đo pin, Camera OAK-D, AI VPU, Phanh khẩn cấp $< 25\text{cm}$ và Web Cockpit.

#### 🔵 Cách 2: Chế độ 2 Terminal qua ROS (Khi cần kết nối RTAB-Map / RViz)
Nếu bạn cần phát Topics hình ảnh cho các node ROS C++ ngoài:
* **Terminal 1:**
  ```bash
  roslaunch jetbot_slam camera_ai.launch confidence:=0.25
  ```
* **Terminal 2:**
  ```bash
  python3 main.py
  ```
*(Hệ thống đã được tối ưu bộ đệm socket 16MB `buff_size=2**24` và bộ khử trùng lặp khung hình, duy trì 30–35 FPS ổn định).*

---

## 🎮 5. ĐIỀU KHIỂN QUA GIAO DIỆN WEB COCKPIT

Mở trình duyệt trên điện thoại hoặc máy tính (cùng mạng Wi-Fi với JetBot):
```text
http://<IP_ROBOT>:8080    (Ví dụ: http://192.168.1.13:8080)
```

* **Bàn phím điều khiển (PC / Laptop):**
  - `W` / `Mũi tên Lên`: Tiến về phía trước (Tự động khóa phanh an toàn khi có cản $\le 25\text{cm}$).
  - `S` / `Mũi tên Xuống`: Lùi xe (Luôn cho phép lùi để thoát hiểm).
  - `A` / `Mũi tên Trái`: Quay trái tại chỗ.
  - `D` / `Mũi tên Phải`: Quay phải tại chỗ.
  - `Space` / Thả phím: Dừng khẩn cấp tức thì.
* **Cảm ứng (Mobile / Tablet):** Sử dụng nút bấm D-Pad hoặc Joystick cảm ứng trực quan trên màn hình.
* **Chuyển chế độ camera:** Bấm nút chuyển đổi giữa **Normal AI Detect** và **Thermal (Camera Nhiệt độ sâu)**.

---

## 🧪 6. QUY TRÌNH KIỂM THỬ ĐỘC LẬP (SELF-TEST)

Hệ thống được thiết kế theo kiến trúc Modular, mỗi module đều có thể tự kiểm thử độc lập mà không cần bật cả xe:

```bash
# 1. Chạy bài kiểm thử toàn diện toàn bộ 52 tiêu chí hệ thống:
python3 scripts/self_test.py

# 2. Kiểm thử độc lập Hệ thống Phanh Khẩn Cấp (Emergency Brake):
python3 -m modules.emergency_brake

# 3. Kiểm thử độc lập Động cơ vi sai PCA9685/TB6612:
python3 -m modules.motor_controller

# 4. Kiểm thử độc lập Mạch đo pin INA219:
python3 -m modules.battery_monitor

# 5. Kiểm thử độc lập Camera OAK-D S2 Streamer:
python3 -m modules.camera_streamer
```

---

## 🎛️ 7. BẢNG CỜ TÍNH NĂNG TRONG `config.py`

Bạn có thể chủ động bật/tắt an toàn bất kỳ chức năng nào trong file [config.py](config.py):

```python
ENABLE_MOTORS       = True   # Bật/Tắt module động cơ vi sai PCA9685
ENABLE_BATTERY      = True   # Bật/Tắt module đọc pin INA219
ENABLE_CAMERA       = True   # Bật/Tắt camera OAK-D S2
ENABLE_YOLO         = True   # Bật/Tắt AI nhận diện người & vật thể
ENABLE_FOLLOWER     = False  # Bật/Tắt tự động bám người (Mặc định False để lái tay an toàn)
ENABLE_MAPPER       = False  # Bật/Tắt dựng bản đồ ngữ nghĩa 3D (Bật khi test SLAM)
ENABLE_WEB          = True   # Bật/Tắt Web Cockpit 3D Dashboard (Port 8080)
ENABLE_SAFETY_BRAKE = True   # Bật/Tắt hệ thống phanh an toàn tự động (< 25cm)
```

---

## 👥 THÔNG TIN TÁC GIẢ & BẢN QUYỀN
* **Đề tài:** Autonomous 3D Semantic SLAM on JetBot with OAK-D S2
* **Khoa:** Khoa Khoa học Máy tính — Trường Đại học Công nghệ Thông tin (ĐHQG-HCM)
* **Mã môn:** CS532.R11
