---
name: jetbot-motor-control
description: >-
  Kỹ sư Điều khiển Tự hành, Động lực học vi sai (PCA9685/TB6612), Phanh an toàn
  tránh va chạm (Virtual Bumper) và Bộ điều khiển bám người (Person Following).
---

# 🤖 HƯỚNG DẪN AI CHUYÊN SÂU: THÀNH VIÊN 1 — ĐIỀU KHIỂN TỰ HÀNH & TƯƠNG TÁC HRI

> **Dành cho:** AI Trợ lý của **Thành viên 1** (Kỹ sư Điều Khiển Tự Hành & HRI).  
> **Dự án:** JetBot Semantic SLAM & Human-Robot Interaction (OAK-D S2 + Jetson Nano 4GB).  
> **Mục tiêu:** Xây dựng cụm điều khiển chuyển động, phanh chủ động và bám theo người với định dạng chuẩn ROS để **khi ghép vào hệ thống chung không bị xung đột hay sửa code**.

---

## 1. VAI TRÒ & PHẠM VI TRÁCH NHIỆM (33% KHỐI LƯỢNG ĐỒ ÁN)
Bạn phụ trách toàn bộ **Tầng Chấp Hành & Tự Hành Cấp Cơ Sở (Execution & Motion Control)**:
1. **Node 1 - `motor_driver_node.py`:** Nhận lệnh vận tốc `/cmd_vel`, giải động học vi sai, ghi thanh ghi PWM I2C, đo pin INA219, và tích hợp **Phanh khẩn cấp (Virtual Bumper)**.
2. **Node 2 - `person_follower_node.py`:** Lắng nghe tọa độ người từ Thành viên 2, áp dụng **Bộ điều khiển PD** để sinh ra lệnh `/cmd_vel` tự động bám theo người.

---

## 2. QUY ĐỊNH BÀN GIAO FILE & MÔI TRƯỜNG CHẠY (BẮT BUỘC TUÂN THỦ)

Để khi gộp file vào JetBot chạy được ngay 100%:
* **Thư mục lưu code trên JetBot:**
  `~/catkin_ws/src/jetbot_slam/src/`
* **Danh sách file bàn giao (Đúng chính xác tên):**
  1. `motor_driver_node.py`
  2. `person_follower_node.py`
* **Môi trường & Định dạng code:**
  - Python 3 (`#!/usr/bin/env python3`) trên Ubuntu 18.04 + ROS Melodic.
  - Cấp quyền chạy: `chmod +x motor_driver_node.py person_follower_node.py`
  - Thư viện cho phép dùng: `rospy`, `geometry_msgs.msg`, `smbus2`, `Adafruit_MotorHAT`, `math`, `time`. Tuyệt đối **không** dùng thư viện ngoài chưa có trong ROS.

---

## 3. HỢP ĐỒNG GIAO TIẾP DỮ LIỆU (TOPIC CONTRACTS - CỰC KỲ QUAN TRỌNG)

### A. Node `motor_driver_node.py`:
* **Topic nhận (Subscribes):**
  - `/cmd_vel` (`geometry_msgs/Twist`): Nhận lệnh điều khiển từ Web (Bạn 3) hoặc từ Person Follower.
  - `/obstacle_distance` (`std_msgs/Float32`): Cự ly vật cản gần nhất phía trước (mét).
* **Topic xuất (Publishes):**
  - `/battery_telemetry` (`std_msgs/Float32MultiArray`): Mảng `[voltage, percentage, current_amps, power_watts, remaining_min]` đọc từ cảm biến INA219.
* **Xử lý phần cứng I2C (Bẫy PCA9685/TB6612 & INA219):**
  ```python
  # 1. Điều khiển động cơ: Motor 1 (Trái), Motor 2 (Phải)
  hat._pwm.setPWM(1, 0, int(speed_L * 16))
  hat._pwm.setPWM(0, 0, 0)
  hat._pwm.setPWM(2, 0, int(speed_R * 16))
  hat._pwm.setPWM(3, 0, 0)

  # 2. Đo pin thời gian thực INA219 (I2C 0x41):
  raw_bus = bus.read_i2c_block_data(0x41, 0x02, 2)
  v = (((raw_bus[0] << 8) | raw_bus[1]) >> 3) * 0.004
  raw_shunt = bus.read_i2c_block_data(0x41, 0x01, 2)
  shunt_raw = (raw_shunt[0] << 8) | raw_shunt[1]
  if shunt_raw > 32767: shunt_raw -= 65536
  curr_a = max(0.05, abs((shunt_raw * 0.00001) / 0.1))
  ```
* **Thuật toán Phanh khẩn cấp (Virtual Bumper):**
  - Nếu `obstacle_distance < 0.25` (mét) VÀ lệnh `linear.x > 0`: **Ép `linear.x = 0` ngay lập tức** (chỉ cho phép lùi hoặc xoay).
* **Watchdog Timeout:** Sau $0.5\text{ giây}$ không có `/cmd_vel` $\rightarrow$ Tự động phanh về 0.

### B. Node `person_follower_node.py`:
* **Topic nhận (Subscribes):**
  - `/spatial_objects` (nhận từ Bạn 2): Lấy tọa độ $(X, Z)$ của vật thể có `id == 0` hoặc `name == 'PERSON'`.
  - `/mode/person_following` (`std_msgs/Bool`): Bật/Tắt tính năng bám người từ Web Dashboard.
* **Topic xuất (Publishes):**
  - `/cmd_vel` (`geometry_msgs/Twist`): Gửi lệnh lái xe xuống cho `motor_driver_node.py`.
* **Thuật toán Điều khiển PD bám người:**
  - Khoảng cách mong muốn duy trì: $Z_{target} = 0.8\text{ m}$.
  - Sai số khoảng cách: $e_z = Z_{person} - Z_{target}$.
  - Sai số góc lệch tâm: $e_x = X_{person}$ (lệch trái/phải).
  - Công thức PD:
    $$v_{linear} = \text{clip}(K_{p\_v} \cdot e_z, -0.2, 0.25)\text{ m/s}$$
    $$\omega_{angular} = \text{clip}(-K_{p\_\omega} \cdot e_x, -0.8, 0.8)\text{ rad/s}$$
    *(Hệ số đề xuất: $K_{p\_v} = 0.4$, $K_{p\_\omega} = 1.2$)*.

---

## 4. QUY TRÌNH TỰ KIỂM THỬ ĐỘC LẬP (TEST KHÔNG CẦN 2 BẠN KIA)

Trước khi gửi code cho nhóm, bạn phải tự chạy test độc lập qua 2 bước:

### Bước 1: Test Driver & Phanh An Toàn:
```bash
# Terminal 1: Chạy node motor
python3 ~/catkin_ws/src/jetbot_slam/src/motor_driver_node.py

# Terminal 2: Bắn lệnh chạy tới
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
# -> Kiểm tra: Bánh quay êm, dừng sau đúng 0.5s.

# Terminal 3: Giả lập có vật cản gần 0.15m và bắn lệnh chạy tới
rostopic pub -1 /obstacle_distance std_msgs/Float32 'data: 0.15'
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
# -> Kiểm tra: Xe đứng yên không chạy (Phanh an toàn hoạt động!).
```

### Bước 2: Test Bộ Điều Khiển Bám Người (Person Follower):
```bash
# Terminal 1: Chạy node bám người
python3 ~/catkin_ws/src/jetbot_slam/src/person_follower_node.py

# Terminal 2: Lắng nghe lệnh xuất ra
rostopic echo /cmd_vel

# Terminal 3: Giả lập Thành viên 2 gửi tọa độ người đang đứng cách 1.5m, lệch phải 0.2m
rostopic pub -1 /spatial_objects std_msgs/String 'data: "[{\"id\":0,\"name\":\"PERSON\",\"x\":0.2,\"z\":1.5}]"'
# -> Kiểm tra: Terminal 2 lập tức xuất /cmd_vel có linear.x > 0 (tiến) và angular.z < 0 (quay phải để căn giữa người).
```

---

## 5. TIÊU CHUẨN NGHIỆM THU BENCHMARK (ĐẠT CHUẨN ĐỒ ÁN)
- **Độ trễ phản ứng Watchdog:** $\le 0.5\text{ giây}$.
- **Khoảng cách phanh dừng khẩn cấp:** Cách vật cản $20\text{cm} - 25\text{cm}$, không va chạm.
- **Tần số xuất `/cmd_vel` khi bám người:** Ổn định $10\text{ Hz}$, chuyển động êm không giật cục.
- **Tài nguyên chiếm dụng trên Jetson:** CPU $< 5\%$, RAM $< 50\text{MB}$.
