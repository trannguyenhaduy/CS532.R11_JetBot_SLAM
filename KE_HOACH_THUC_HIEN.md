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

### 🟢 PHẦN 2: ĐỒNG BỘ PHƯƠNG HƯỚNG & ĐỊNH VỊ (CLOSED-LOOP VISUAL GYROSCOPE & HEADING)
> *Mục tiêu:* Xe quay $360^\circ$ ngoài đời thực thì trên bản đồ 2D phải quay đúng $360^\circ$, không bị lệch góc, không xoay lố, không ăn gian giây.

- [x] **2.1. Phân tích nguyên nhân cốt lõi khiến quay lố / lệch hướng:**
  - [x] Phát hiện phương pháp cũ dùng thời gian mò (Open-loop timing bằng giây) bị sai lệch nặng nề do trượt bánh trên sàn gạch và điện áp pin 3S giảm từ 12.6V xuống 11.1V làm thay đổi RPM động cơ.
  - [x] Khắc phục việc luồng thị giác bị vô hiệu hóa khi xe đang di chuyển (`if not is_moving`).
- [x] **2.2. Xây dựng bộ đo góc quay thị giác vòng kín (`VisualHeadingTracker` trong `modules/camera_streamer.py`):**
  - [x] Thuật toán Pyramidal Lucas-Kanade Optical Flow (Shi-Tomasi features) xử lý siêu tốc ~1.5ms trên khung hình 320x180.
  - [x] Lọc nhiễu ngoại lai: Giới hạn rung lắc dọc $|dy| < 8\text{px}$, lấy trung vị độ dời ngang $dx_{\text{median}}$ để tính $d\theta = \arctan(dx / f_x)$ với $f_x = 232.8\text{ px/rad}$ (HFOV 69° OAK-D S2).
  - [x] Cập nhật liên tục góc quay thực tế ngoài đời vào `self.robot_yaw` ở tần số 30 FPS, đồng bộ 1:1 biểu tượng xe trên bản đồ 2D trong mọi tình huống (kể cả khi lấy tay xoay xe).
- [x] **2.3. Hợp nhất thành 1 Chế Độ Tự Quét 360° Duy Nhất — Quay Chậm Đều Vòng Kín (Single Smooth 360° Mode):**
  - [x] Loại bỏ các chế độ chia bước hoặc chọn thời gian rườm rà; chỉ giữ lại 1 nút bấm duy nhất `🔄 QUÉT 360°`.
  - [x] Robot quay chậm đều êm ái ở mức 13% PWM và đo liên tục góc thực tế qua camera OAK-D (Visual Gyroscope).
  - [x] Tự động giảm tốc xung khi còn $15^\circ$ để triệt tiêu trớn quán tính, ngắt phanh tức thì khi góc đo chạm đúng $360.0^\circ$ ($2\pi\text{ rad}$).
  - [x] Chốt cứng góc quay về đúng hướng xuất phát ban đầu, đồng bộ 100% với bản đồ 2D.


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
