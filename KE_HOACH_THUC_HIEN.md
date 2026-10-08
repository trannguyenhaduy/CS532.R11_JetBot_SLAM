# 🚀 KẾ HOẠCH TỔNG THỂ & TIẾN ĐỘ THỰC HIỆN ĐỒ ÁN XE TỰ HÀNH JETBOT

> **ĐỀ TÀI:** **JetBot — Kết hợp Camera và LiDAR (Stereo Depth emulated LiDAR) để Xây dựng bản đồ, Định vị, Lập kế hoạch đường đi và Tránh vật cản**  
> **Nền tảng:** NVIDIA Jetson Nano 4GB + Luxonis OAK-D S2 (DepthAI / VPU Myriad X) + Waveshare JetBot Chassis  
> **Kiến trúc:** Modular Architecture — Mỗi tính năng 1 file độc lập trong `modules/`.  
> **Cập nhật:** 08/10/2026 (Đồng bộ theo phản hồi kiểm thử thực tế)

---

## 👥 PHÂN CÔNG & TIẾN ĐỘ TỔNG QUAN

| Nhóm Tính Năng | Trọng Trách Kỹ Thuật | Trạng Thái |
| :--- | :--- | :---: |
| **Phần 1: Phần Cứng & An Toàn** | Động cơ vi sai PCA9685, Đọc pin INA219, Phanh khẩn cấp Virtual Bumper | ✅ 100% HOÀN THÀNH |
| **Phần 2: Phương Hướng & Định Vị** | Đồng bộ góc quay Yaw ngoài đời & Map, Odometry vi sai chuẩn xác 1:1 | ✅ 100% HOÀN THÀNH |
| **Phần 3: Quét Bounding Box Vật Cản** | Bounding Box cho mọi vật cản (`UNKNOWN`/`OBSTACLE`) từ 3D Depth | ✅ 100% HOÀN THÀNH |
| **Phần 4: Xây Dựng Bản Đồ 2D SLAM** | Lưới Occupancy Grid tĩnh, Lưu/Nạp bản đồ, Quét 360° chuẩn 1 vòng | ✅ 100% HOÀN THÀNH |
| **Phần 5: Lập Kế Hoạch Đường Đi A\*** | Costmap Inflation phình to vật cản, Tìm đường A-Star, Click Goal trên Web | ⏳ ĐANG THỰC HIỆN |
| **Phần 6: Tự Hành Né Vật Cản** | Path Follower bám Waypoints, Re-planning khi gặp cản mới đột xuất | 📅 BƯỚC TIẾP THEO |

---

## 📋 DANH SÁCH CÔNG VIỆC CHI TIẾT (TO-DO ROADMAP)

### 🟢 PHẦN 1: NỀN TẢNG PHẦN CỨNG & AN TOÀN CƠ SỞ (ĐÃ XONG)
- [x] **1.1. Điều khiển động cơ vi sai (`modules/motor_controller.py`):**
  - [x] Giao tiếp I2C chip PCA9685 (`0x60`), tần số PWM 1600 Hz.
  - [x] Hỗ trợ đảo kênh Trái/Phải (`SWAP_MOTORS`) và đảo chiều tiến/lùi (`INVERT_LINEAR`).
  - [x] Watchdog Timer an toàn 0.5s tự ngắt động cơ khi mất tín hiệu.
- [x] **1.2. Giám sát pin thời gian thực (`modules/battery_monitor.py`):**
  - [x] Đọc điện áp (V), dòng điện (A), công suất (W) qua chip INA219 (`0x41`).
  - [x] Tính toán phần trăm pin 3S Li-ion và cảnh báo khi điện áp dưới 10.2V.
- [x] **1.3. Phanh khẩn cấp ảo Virtual Bumper (`modules/emergency_brake.py`):**
  - [x] Tự động khóa lệnh tiến khi cự ly cản trước mũi xe $\le 20\text{cm}$, luôn cho phép lùi/quay.
- [x] **1.4. Web Cockpit điều khiển W-A-S-D (`modules/web_server.py`):**
  - [x] Giao diện điều khiển Web cổng 8080, luồng video MJPEG độ trễ thấp.
  - [x] Tối ưu Lock giải phóng CPU, triệt tiêu độ trễ điều khiển.

---

### 🟢 PHẦN 2: ĐỒNG BỘ PHƯƠNG HƯỚNG & ĐỊNH VỊ (LOCALIZATION & YAW - ĐÃ XONG)
> *Mục tiêu:* Xe quay $360^\circ$ ngoài đời thực thì trên bản đồ 2D phải quay đúng $360^\circ$, không bị lệch góc, không xoay lố $720^\circ$.

- [x] **2.1. Tìm ra nguyên nhân lệch góc quay giữa thực tế và phần mềm:**
  - [x] Phát hiện vận tốc góc phần mềm ($28.6^\circ/\text{s}$) lệch 3.2 lần so với tốc độ quay thực tế của bánh xe ($90^\circ - 100^\circ/\text{s}$).
  - [x] Khử bỏ việc đè góc giữa luồng thị giác thụ động và lệnh lái động học.
- [x] **2.2. Hiệu chuẩn mô hình động học Odometry góc quay (Angular Odometry Calibration):**
  - [x] Hiệu chuẩn ánh xạ tuyến tính `turn_duty = (|w| / 1.58) * 0.13` trong `modules/motor_controller.py`.
  - [x] Cập nhật vận tốc góc Web `speedAngular = 1.58 rad/s` ($\approx 90.5^\circ/\text{s}$) đồng bộ 1:1 với mô-men động cơ.
  - [x] Cập nhật các mức tốc độ Teleop: Chậm ($1.20\text{ rad/s}$), Chuẩn ($1.58\text{ rad/s}$), Nhanh ($1.90\text{ rad/s}$).
  - [x] Giữ phím A/D trong 4.0 giây: Xe quay đúng 1 vòng $360^\circ$ ngoài đời và bản đồ quay đúng 1 vòng $360^\circ$.
- [x] **2.3. Hiệu chuẩn chế độ Tự Quét 360° (Auto 360° Panorama - Triệt tiêu lỗi xoay 720°):**
  - [x] Chế độ Smooth: Điều chỉnh thời gian quay chuẩn xuống `4.2s` trong `config.py`, `cockpit.html`, và `main.py` (quay đúng 1 vòng $360^\circ$, không xoay quá trớn $720^\circ$).
  - [x] Chế độ Step (8 bước x 45°): Mỗi bước xung `0.48s` @ 13% PWM ($45^\circ$), dừng tĩnh 0.7s để quét, kết thúc đúng 1 vòng $360^\circ$ và khóa chốt ảnh mốc xuất phát Loop Closure.

---

### 🟢 PHẦN 3: QUÉT BOUNDING BOX CHO MỌI VẬT CẢN (UNKNOWN OBSTACLE CLUSTERING - ĐÃ XONG)
> *Mục tiêu:* Không phụ thuộc vào việc AI có biết tên đồ vật hay không. Đặt 2 thùng carton, balo, hộp, ghế hay bất cứ vật gì trước xe thì xe đều phải quét được Bounding Box, gán nhãn `[UNKNOWN · Khoảng_cách]` và thể hiện rõ trên bản đồ 2D để né!

- [x] **3.1. Thuật toán phân cụm không gian 3D (`OccupancySLAM._cluster_obstacle_points`):**
  - [x] Trích xuất các điểm cản thực sự ($0.05\text{m} \le wz \le 0.85\text{m}$, cự ly $0.18\text{m} - 3.2\text{m}$) từ ma trận Stereo Depth OAK-D S2.
  - [x] Thuật toán Euclidean Clustering gom các điểm cản gần nhau ($\le 28\text{cm}$) thành từng khối vật thể độc lập.
- [x] **3.2. Tính toán hình học Bounding Box & Gán nhãn UNKNOWN:**
  - [x] Tính toán tâm $(X, Y, Z)$, chiều rộng (width) và chiều sâu (depth) thực tế của từng cụm vật cản.
  - [x] Nếu AI MobileNet nhận diện được tên (ví dụ `PERSON`, `CHAIR`) $\to$ cập nhật kích thước thực tế cho nhãn đó.
  - [x] Nếu AI không nhận diện được (ví dụ thùng carton, túi giấy, hộp) $\to$ tự động tạo Landmark **`UNKNOWN`** kèm Bounding Box mét thực tế.
- [x] **3.3. Hiển thị trực quan toàn diện:**
  - [x] Vẽ khung Bounding Box kỹ thuật 2D trên Bản đồ Occupancy Grid (kèm kích thước mét thực tế và nhãn `[UNKNOWN · <cự_ly>m]`).
  - [x] Chiếu cụm cản lên luồng Camera TV2: Tự động vẽ khung Tactical Neon `UNKNOWN 85% (<cự_ly>)` ngay trên hình ảnh camera.

---

### 🟢 PHẦN 4: HOÀN THIỆN BẢN ĐỒ CHIẾM DỤNG 2D (OCCUPANCY GRID SLAM - ĐÃ XONG)
> *Mục tiêu:* Bản đồ 2D thể hiện tường, sàn nhà, vật cản cố định và có tính năng Lưu/Tải bản đồ.

- [x] **4.1. Lưới Occupancy Grid 2D tích lũy vĩnh viễn (`modules/occupancy_slam.py`):**
  - [x] Ma trận $200 \times 200$ ô (kích thước $10\text{m} \times 10\text{m}$, độ phân giải $5\text{cm/ô}$).
  - [x] Bộ lọc sàn nhà wz < 6cm không đánh dấu nhầm thành tường.
  - [x] Bộ đệm Costmap Inflation $15\text{cm}$ và đóng liền mạch tường (Wall Closing).
- [x] **4.2. Tính năng Lưu & Tải bản đồ (`save_map` / `load_map`):**
  - [x] Lưu bản đồ thành file `.json` (ma trận số + danh sách Landmark) và `.png` (ảnh đồ họa kiến trúc).
  - [x] Nạp lại bản đồ phòng đã lưu khi khởi động.
- [x] **4.3. Tối ưu hóa cập nhật cụm vật cản tĩnh vào bản đồ:**
  - [x] Tích hợp danh sách cụm vật thể `UNKNOWN` vào danh mục Landmarks để ghim vị trí cố định trên map.
  - [x] Thuật toán De-duplication tinh chỉnh: Khoảng cách gom $35\text{cm}$ cho các hộp UNKNOWN giúp bảo toàn tách biệt 2 thùng cản đặt cạnh nhau mà không bị dính chùm.

---

### 🟠 PHẦN 5: LẬP KẾ HOẠCH ĐƯỜNG ĐI A* (GLOBAL PATH PLANNING)
> *Mục tiêu:* Cho phép người dùng click 1 điểm trên Web Map, thuật toán A* tính toán đường đi ngắn nhất không va vào tường.

- [ ] **5.1. Xây dựng module `modules/path_planner.py`:**
  - [ ] Nhận ma trận Occupancy Grid 2D từ `OccupancySLAM`.
  - [ ] Tạo bản đồ chi phí an toàn (Costmap): Phình to vật cản thêm bán kính $15\text{cm}$ (bán kính thân xe JetBot).
  - [ ] Cài đặt thuật toán tìm đường ngắn nhất **A\* (A-Star)** từ vị trí xe hiện tại $(X_{rob}, Y_{rob})$ tới điểm Đích $(X_{goal}, Y_{goal})$.
  - [ ] Tối ưu ma trận Heuristic đảm bảo tính toán xong trong $< 10\text{ms}$.
- [ ] **5.2. Tích hợp tương tác trên Web Cockpit:**
  - [ ] Cho phép click chuột vào bản đồ 2D để đặt cờ điểm đích (Goal Target).
  - [ ] Vẽ đường đi dự kiến A* bằng vệt sáng neon nối từ xe đến đích.

---

### 🔵 PHẦN 6: ĐIỀU HƯỚNG TỰ HÀNH & NÉ VẬT CẢN (AUTONOMOUS NAVIGATION)
> *Mục tiêu:* Xe tự lăn bánh theo đường đi A*, gặp vật cản bất ngờ thì tự dừng hoặc né.

- [ ] **6.1. Xây dựng module `modules/navigator.py`:**
  - [ ] Thuật toán bám đường Pure Pursuit / Waypoint Follower: Tính toán vận tốc tiến $v$ và góc lái $\omega$ bám sát từng điểm mốc.
  - [ ] Khi gần tới đích ($< 10\text{cm}$), xe giảm tốc từ từ và dừng lại hoàn toàn.
- [ ] **6.2. Phát hiện chướng ngại vật đột xuất & Tìm lại đường (Dynamic Re-planning):**
  - [ ] Nếu vật cản mới xuất hiện chắn ngang đường đi A*, xe tự dừng lại an toàn.
  - [ ] Kích hoạt thuật toán A* tính toán lại một đường vòng khác để né vật cản.
- [ ] **6.3. Nút bấm điều khiển trên Web:**
  - [ ] Nút `BẮT ĐẦU TỰ HÀNH (START NAV)` và `HỦY TỰ HÀNH (CANCEL)`.

---

### 🟣 PHẦN 7: KIỂM THỬ TOÀN DIỆN & BẢO VỆ ĐỒ ÁN
- [ ] **7.1. Chạy kịch bản thực tế:** Xe tự đi từ cửa phòng vào bàn học, tự né thùng carton và dừng tại đích.
- [ ] **7.2. Tự động hóa bộ kiểm thử `scripts/self_test.py` cho 100% các module.**
- [ ] **7.3. Đóng gói video demo và báo cáo kỹ thuật hoàn chỉnh.**
