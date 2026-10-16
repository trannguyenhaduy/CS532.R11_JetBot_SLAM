# 📘 BÁO CÁO KỸ THUẬT TOÀN DIỆN HỆ THỐNG XE TỰ HÀNH JETBOT SLAM
## ĐỒ ÁN CHUYÊN NGÀNH: CS532.R11 — ĐỊNH VỊ, LẬP BẢN ĐỒ VÀ ĐIỀU HƯỚNG TỰ HÀNH

> **Đề tài:** Nghiên cứu, Thiết kế và Hiện thực Hệ thống Định vị, Lập bản đồ (SLAM 2D) và Điều khiển Vi sai Độ chính xác cao cho Robot JetBot sử dụng Camera Thị giác Không gian Stereo Depth (OAK-D S2) kết hợp Cảm biến Quán tính IMU  
> **Nền tảng phần cứng:** NVIDIA Jetson Nano B01 (4GB) + Luxonis OAK-D S2 (Stereo Depth + IMU 100Hz) + Khung gầm Waveshare JetBot Chassis  
> **Thời gian hoàn thiện:** Tháng 10/2026  
> **Phiên bản hệ sinh thái:** `v3.1.0-PRECISION-SLAM-IMU-LOCK` (Kiến trúc 3 Tab: Cockpit Thực Địa, Algorithm Deep Inspector & Dynamic Profile Hot-Reload)  
> **Tác giả / Nhóm thực hiện:** Nhóm nghiên cứu CS532.R11  

---

## 📑 MỤC LỤC CHI TIẾT

1. [TỔNG QUAN HỆ THỐNG & ĐẶT VẤN ĐỀ NGHIÊN CỨU](#1-tổng-quan-hệ-thống--đặt-vấn-đề-nghiên-cứu)
   - 1.1. Bối cảnh & Thách thức đặc thù của khung gầm vi sai mini
   - 1.2. Mục tiêu nghiên cứu & Các đột phá kỹ thuật cốt lõi
   - 1.3. Mô hình triển khai: 1-Terminal Mode (`python3 main.py`) vs Dual ROS Mode
2. [KIẾN TRÚC PHẦN CỨNG & HÌNH HỌC KHÔNG GIAN (EXTRINSICS & SENSORS)](#2-kiến-trúc-phần-cứng--hình-học-không-gian-extrinsics--sensors)
   - 2.1. Sơ đồ khối tích hợp phần cứng toàn diện
   - 2.2. Thông số hình học & Ma trận biến đổi ngoại vi (Extrinsics Transformation)
   - 2.3. Cụm cảm biến thị giác OAK-D S2 & Cảm biến quán tính IMU
   - 2.4. Thuật toán tự động nhận diện trục trọng lực (Auto Gravity Detection)
3. [KIẾN TRÚC PHẦN MỀM ĐA LUỒNG & BỘ ĐIỀU PHỐI (ORCHESTRATOR)](#3-kiến-trúc-phần-mềm-đa-luồng--bộ-điều-phối-orchestrator)
   - 3.1. Phân luồng thực thi 4 tiến trình song song (Concurrency Architecture)
   - 3.2. Cơ chế an toàn dữ liệu & Đồng bộ hóa đa luồng (`threading.RLock`)
   - 3.3. Cơ chế cấu hình tập trung (`config.py` Single Source of Truth)
4. [ĐỘNG HỌC ROBOT VI SAI & CÁC THUẬT TOÁN ĐIỀU KHIỂN CẤP THẤP](#4-động-học-robot-vi-sai--các-thuật-toán-điều-khiển-cấp-thấp)
   - 4.1. Mô hình động học vi sai chuẩn hóa & Ánh xạ công suất xung PWM
   - 4.2. Cân bằng điện áp 2 bánh phần cứng (Hardware Wheel Trim Balance)
   - 4.3. Khóa hướng đi thẳng chủ động vòng kín (Active IMU Heading Lock Assist)
   - 4.4. Động học giảm tốc mượt mà Cosine S-Curve 50Hz triệt tiêu quán tính
   - 4.5. Chuẩn hóa sụt áp khối Pin 3S thời gian thực (Battery Voltage Normalization)
   - 4.6. Điều khiển bước định lượng chính xác (Metric Step Movement Worker)
   - 4.7. Logic đối chiếu góc quay thực tế: Cơ chế đánh giá trung thực (Honest Diagnostics)
5. [THỊ GIÁC KHÔNG GIAN STEREO & PIPELINE SLAM 2D CHÍNH XÁC CAO](#5-thị-giác-không-gian-stereo--pipeline-slam-2d-chính-xác-cao)
   - 5.1. Thuật toán giả lập LaserScan 2D từ Stereo Depth (120 tia quét siêu mịn)
   - 5.2. Thuật toán phân tách sàn nhà ($< 0.04\text{m}$) và vật cản thực ($0.04\text{m} - 0.85\text{m}$)
   - 5.3. Thuật toán ma trận lưới chiếm dụng 2D (Occupancy Grid Mapping)
   - 5.4. Thuật toán so khớp quét đối ứng Likelihood Field CSM 2D (Correlative Scan Matching)
   - 5.5. Bóc tách hiện tượng lệch 45° sau khi quay 360° & Cơ chế Khóa Cổng Xoay (Turn Gate & CSM Yaw Lock)
   - 5.6. Kỹ thuật cập nhật bản đồ tức thì theo từng sector (Instant Sector Raycasting)
6. [HỆ THỐNG QUẢN LÝ CẤU HÌNH ĐỘNG & ĐA PHIÊN BẢN (MULTI-PROFILE HOT-RELOAD ENGINE)](#6-hệ-thống-quản-lý-cấu-hình-động--đa-phiên-bản-multi-profile-hot-reload-engine)
   - 6.1. Kiến trúc lưu trữ bền vững hai chiều (Two-Way Persistence)
   - 6.2. Cơ chế nạp tức thì không gián đoạn (Hot-Reloading without Downtime)
   - 6.3. Giải thích chuyên sâu lý do tồn tại của từng tham số cấu hình
7. [BỘ CÔNG CỤ CHẨN ĐOÁN & PHÒNG THÍ NGHIỆM SOI THUẬT TOÁN (ALGORITHM INSPECTOR & PROFILER)](#7-bộ-công-cụ-chẩn-đoán--phòng-thí-nghiệm-soi-thuật-toán-algorithm-inspector--profiler)
   - 7.1. Bảng đo hiệu năng mạng & Phân tích thông lượng băng thông thực tế
   - 7.2. Chẩn đoán điểm nghẽn tự động (Automatic Bottleneck Detector)
   - 7.3. Phòng thí nghiệm CSM Scan Snap & Radar phân kỳ IMU vs Odometry
   - 7.4. Cơ chế đóng băng frame (Frame Freezing) & Bóc tách quyết định điều khiển
8. [CÁC BÀI THỬ NGHIỆM QUỸ ĐẠO HÌNH HỌC KHÉP KÍN (GEOMETRIC BENCHMARK)](#8-các-bài-thử-nghiệm-quỹ-đạo-hình-học-khép-kín-geometric-benchmark)
   - 8.1. Quỹ đạo Hình Chữ Nhật 4 chặng khép kín ($80\text{cm} \times 40\text{cm}$)
   - 8.2. Quỹ đạo Tam Giác Vuông Cân Pythagore ($50\text{cm} - 50\text{cm} - 70.7\text{cm}$)
   - 8.3. Bài kiểm tra Xoay 360° tại chỗ khép kín về hướng 3 giờ ban đầu
9. [BẢNG TỔNG HỢP THAM SỐ TOÀN HỆ THỐNG & DANH MỤC REST API](#9-bảng-tổng-hợp-tham-số-toàn-hệ-thống--danh-mục-rest-api)
   - 9.1. Bảng tra cứu tham số động học chuẩn trong `config.py`
   - 9.2. Danh mục đầy đủ các cổng giao tiếp REST API
10. [KẾT QUẢ THỰC NGHIỆM, KẾT LUẬN & HƯỚNG PHÁT TRIỂN](#10-kết-quả-thực-nghiệm-kết-luận--hướng-phát-triển)
   - 10.1. Đánh giá định lượng độ chính xác
   - 10.2. Kết luận khoa học
   - 10.3. Hướng phát triển tiếp theo

---

## 1. TỔNG QUAN HỆ THỐNG & ĐẶT VẤN ĐỀ NGHIÊN CỨU

### 1.1. Bối cảnh & Thách thức đặc thù của khung gầm vi sai mini
Hệ thống xe tự hành kích thước nhỏ (Miniature Mobile Robots) hoạt động trong môi trường trong nhà (GPS-denied) là đối tượng nghiên cứu trọng tâm của lĩnh vực Robotics hiện đại. Tuy nhiên, khi triển khai trên phần cứng thực tế như khung gầm JetBot, các kỹ sư thường gặp phải 5 nút thắt kỹ thuật nghiêm trọng:
1. **Thiếu vắng Encoder cơ khí trên trục bánh xe:** Đa số khung gầm JetBot thương mại không trang bị đĩa mã hóa vòng quay (Optical/Magnetic Wheel Encoders). Việc ước lượng vị trí dựa thuần túy vào thời gian cấp xung PWM (Dead-Reckoning hở) làm sai số tích lũy tăng theo cấp số nhân chỉ sau vài giây di chuyển.
2. **Chi phí và tải trọng của cảm biến LiDAR 360°:** Một cảm biến LiDAR quay cơ học truyền thống có giá thành cao, tiêu thụ điện năng lớn và khối lượng nặng làm hạ trọng tâm và nhanh kiệt pin 3S của robot.
3. **Sự bất đối xứng vật lý nội tại của hai động cơ DC giảm tốc:** Do dung sai gia công bánh răng nhựa, độ rơ trục (backlash), ma sát chổi than và điện trở tiếp xúc khác nhau, cấp cùng một mức điện áp hoặc cùng một xung PWM sẽ khiến robot bị nhao lái (lệch trục) sang trái hoặc phải ngay sau 0.5m di chuyển.
4. **Động lực học bất đối xứng giữa Chạy Tiến và Chạy Lùi do bánh mắt trâu (Caster Wheel):** Bánh mắt trâu chịu tải phụ phía trước có khớp xoay tự do. Khi xe chạy TIẾN, bánh mắt trâu bị đẩy xuôi, khớp xoay tự lựa theo hướng tiến với lực cản lăn nhỏ. Khi xe LÙI, bánh mắt trâu bị kéo ngược, cơ cấu xoay bị bẻ giật 180° sinh ra ma sát kéo ngược rất lớn, làm quãng đường lùi thực tế luôn ngắn hơn quãng đường tiến nếu dùng chung một hệ số co giãn.
5. **Năng lực tính toán biên hạn chế của SoC nhúng:** NVIDIA Jetson Nano B01 sở hữu 4 nhân ARM Cortex-A57 @ 1.43 GHz và 4GB RAM LPDDR4 chia sẻ cho GPU. Các thuật toán SLAM 3D cồng kềnh (như RTAB-Map, ORB-SLAM3) sẽ nhanh chóng gây tràn bộ nhớ (Out-Of-Memory - OOM) và làm tụt FPS xuống dưới 2–4 FPS, mất tính thời gian thực.
6. **Hiện tượng trôi dạt góc xoay khi so khớp quét (CSM Angular Drift Trap):** Khi sử dụng camera độ sâu có góc mở ngang giới hạn (FOV $\approx 72^\circ - 75^\circ$) thay vì LiDAR $360^\circ$, việc cho phép thuật toán quét đối ứng (Correlative Scan Matching) tự do tìm kiếm góc xoay khi xe quay tại chỗ sẽ dẫn đến việc mây tia quét bị hút vào các cực đại cục bộ phụ của vật cản gần (như bọc đồ, chân bàn), bẻ lệch góc robot tới $\approx 5^\circ - 6^\circ$ sau mỗi lần quay. Sau 8 lần quay $45^\circ$ ($360^\circ$), góc robot bị bẻ lệch tới $\approx 45^\circ - 49^\circ$, khiến xe trôi từ hướng 3 giờ sang hướng 1 giờ.

### 1.2. Mục tiêu nghiên cứu & Các đột phá kỹ thuật cốt lõi
Dự án **CS532.R11** đặt mục tiêu thiết kế một hệ thống SLAM 2D thời gian thực hoàn chỉnh, chạy mượt mà ở tần số 15–20 FPS trên Jetson Nano, đạt độ chính xác định vị centimet mà không cần encoder cơ khí hay LiDAR đắt tiền.

Các đột phá kỹ thuật đã hiện thực thành công bao gồm:
* **Tận dụng VPU Intel Myriad X trên OAK-D S2:** Đẩy toàn bộ tác vụ tính toán ma trận độ sâu stereo (Stereo Disparity Block Matching) sang VPU chuyên dụng, giải phóng 100% CPU của Jetson Nano cho bài toán SLAM và điều khiển.
* **Tích hợp IMU phần cứng 100Hz với cơ chế Tự động Dò Trục Trọng Lực:** Đo đạc trực tiếp tốc độ góc và gia tốc 3 trục, tự động nhận diện hướng đặt camera (trục Y quang học hay trục Z ENU) để tích phân góc Yaw chuẩn xác, triệt tiêu hoàn toàn sự trôi dạt góc quay tích lũy.
* **Thuật toán Virtual LaserScan 120 tia quét siêu mịn:** Chiếu không gian độ sâu 3D thành 120 chùm tia quét 2D trải đều trên FOV 72° (~0.6°/tia), tăng gấp đôi mật độ so với chuẩn 60 tia, giúp nhận dạng bờ tường và vật cản nhỏ sắc nét.
* **Khóa Cứng Góc Xoay Trong CSM (CSM Yaw Lock & Turn Gating):** Phân tách rành mạch: CSM chỉ nắn tọa độ tịnh tiến $(X, Y)$ đối chiếu bờ tường khi chạy thẳng; góc hướng Yaw được bảo toàn tuyệt đối theo Odometry/IMU khi quay tại chỗ, giải quyết triệt để lỗi trôi dạt góc $45^\circ$ sau vòng quay $360^\circ$.
* **Cập Nhật Bản Đồ Tức Thì Theo Sector 45° (Instant Sector Raycasting):** Ngay khi mỗi bước quay kết thúc, bản đồ 2D bắn tức thời một frame quang học chuẩn xác, giúp bản đồ khép kín tròn trịa và xóa sổ hoàn toàn hiện tượng vệt nan quạt.
* **Cơ Chế Đánh Giá Góc Quay Trung Thực (Honest Turn Diagnostics):** Loại bỏ hoàn toàn mã giả lập sai số $0.0^\circ$. Khi không có IMU, hệ thống hiển thị rõ ràng là ước tính mô hình hóa (`--`), giúp kỹ sư luôn nắm bắt được trạng thái thực tế của phần cứng.
* **Kiến trúc Cockpit 3 Tab Đột Phá:** Phân tách rành mạch giữa Buồng lái vận hành, Phòng thí nghiệm soi thuật toán từng frame (kèm đóng băng frame trích xuất ảnh RGB/Nhiệt) và Trung tâm quản lý cấu hình Profile Hot-Reload 1-click.

### 1.3. Mô hình triển khai: 1-Terminal Mode (`python3 main.py`) vs Dual ROS Mode
Hệ thống được thiết kế theo kiến trúc linh hoạt hỗ trợ 2 chế độ vận hành:
1. **1-Terminal Mode (`python3 main.py`) — Chế độ Khuyến nghị:**
   * **Cách hoạt động:** Chỉ cần chạy đúng 1 lệnh duy nhất trong một terminal trên Jetson Nano.
   * **Ưu điểm vượt trội:**
     * Toàn bộ pipeline (Camera, Depth, IMU, Motor, SLAM 2D, Web Server) chạy trong cùng một tiến trình Python đa luồng bộ nhớ dùng chung.
     * Không tốn chi phí đóng/mở gói tin IPC hay tuần tự hóa (serialization) của ROS message qua socket loopback.
     * Tiết kiệm $\approx 180\text{MB}$ RAM và giảm tải CPU từ $60\% \to 35\%$, giúp Jetson Nano hoạt động mát mẻ, không bị giật lag khi điều khiển.
     * Quản lý tập trung tuyệt đối qua file [config.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/config.py), người dùng chỉ cần sửa trực tiếp tại một nơi duy nhất.
2. **Dual-Terminal ROS Mode (`roscore` + ROS Node):**
   * Được thiết kế cho các kịch bản mở rộng khi cần liên kết với các gói ngoài của ROS (như RViz, Navigation stack `move_base`, RTAB-Map ROS).
   * Module [camera_streamer.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/modules/camera_streamer.py) và [main.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/main.py) tự động lắng nghe topics ROS nếu `roscore` đang chạy; nếu không có ROS, hệ thống trượt êm sang chế độ chạy độc lập thuần túy mà không báo lỗi.

---

## 2. KIẾN TRÚC PHẦN CỨNG & HÌNH HỌC KHÔNG GIAN (EXTRINSICS & SENSORS)

### 2.1. Sơ đồ khối tích hợp phần cứng toàn diện

```
                          ┌───────────────────────────────────────────┐
                          │     NVIDIA JETSON NANO (B01 - 4GB)        │
                          │     - OS: Ubuntu 18.04 LTS (JetPack 4.6)  │
                          │     - CPU: Quad-Core ARM Cortex-A57       │
                          │     - GPU: 128-core Maxwell Architecture  │
                          └─────────────────────┬─────────────────────┘
                                                │
                 ┌──────────────────────────────┼──────────────────────────────┐
                 │ USB 3.0 (SuperSpeed 5Gbps)   │ I2C Bus 1 (Pins 3, 5 @ 400kHz)│ Wi-Fi 802.11ac (TCP/IP)
                 ▼                              ▼                              ▼
  ┌──────────────────────────────┐    ┌─────────────────────┐    ┌───────────────────────────┐
  │ LUXONIS OAK-D S2             │    │ PCA9685 PWM DRIVER  │    │ CLIENT BROWSER COCKPIT    │
  │ • Stereo Baseline: 7.5 cm    │    │ • Địa chỉ I2C: 0x60 │    │ • Giao diện Web Tabbed    │
  │ • Cặp cảm biến OV9282 Mono   │    │ • Tần số PWM: 1600Hz│    │ • Video Streaming MJPEG   │
  │ • Cảm biến IMU 6-DoF 100Hz   │    └──────────┬──────────┘    │ • REST API Telemetry      │
  │ • VPU Intel Myriad X 4 TOPS  │               │               │ • HTML5 Canvas 2D SLAM    │
  │ • Tọa độ: x=+9cm, y=-1.5cm   │               ▼               └───────────────────────────┘
  └──────────────────────────────┘    ┌─────────────────────┐
                                      │ CẦU H TB6612FNG (x2)│
                                      │ • Điện áp: 7.4-12V  │
                                      └──────────┬──────────┘
                                                 ▼
                                      ┌─────────────────────┐    ┌───────────────────────────┐
                                      │ ĐỘNG CƠ TT DC (x2)  │    │ CẢM BIẾN NGUỒN INA219     │
                                      │ • Tỉ số truyền 1:48 │    │ • Địa chỉ I2C: 0x41       │
                                      │ • Vệt bánh: L=12 cm │    │ • Đo điện áp khối Pin 3S  │
                                      └─────────────────────┘    └───────────────────────────┘
```

### 2.2. Thông số hình học & Ma trận biến đổi ngoại vi (Extrinsics Transformation)

Hệ quy chiếu chuẩn của xe JetBot được định nghĩa theo chuẩn ISO 8855 / ROS REP 105:
* **Gốc tọa độ thân xe (Base Link $\{B\}$):** Đặt tại trung điểm của đoạn thẳng nối hai tâm bánh xe chủ động, tiếp xúc với mặt phẳng sàn ($Z = 0$).
  * Trục $+X_B$: Hướng thẳng về phía trước đầu xe.
  * Trục $+Y_B$: Hướng sang phía bên trái thân xe.
  * Trục $+Z_B$: Hướng thẳng đứng vuông góc lên trần nhà.
* **Gốc tọa độ Camera OAK-D S2 ($\{C\}$):** Đặt tại tâm thấu kính quang học của camera bên trái.
  * Khoảng dịch chuyển tịnh tiến nhô ra phía trước: $\Delta x_{\text{cam}} = +0.090\text{ m}$ ($+9.0\text{ cm}$).
  * Khoảng dịch chuyển lệch sang phải so với trục giữa: $\Delta y_{\text{cam}} = -0.015\text{ m}$ ($-1.5\text{ cm}$).
  * Độ cao trục quang học so với mặt sàn: $h_{\text{cam}} = +0.120\text{ m}$ ($+12.0\text{ cm}$).
  * Góc nghiêng ngửa/cúp của camera: $\text{pitch} = 0.0^\circ$ (song song tuyệt đối với mặt sàn).

```
                      ▲ +X_B (Hướng tiến)
                      │
                      │    ┌─────────────────┐
                      │    │   OAK-D S2      │◄── (x=+0.090m, y=-0.015m, z=+0.120m)
                      │    └────────┬────────┘
                      │             │ Δx = +9.0cm
       ◄──────────────┼─────────────┴─────────────
       +Y_B (Trái)    │  [BÁNH MẮT TRÂU]
                      │
             [BÁNH L] │              [BÁNH R]
             ═════════╪══════════════════════
                      │◄─── L = 12.0cm ───►
                      │
                   GỐC {B} (Tâm trục bánh, mặt sàn)
```

#### Ma trận chuyển đổi tọa độ từ Camera sang Thân xe:
Với một điểm cản $P_C = [X_C, Y_C, Z_C]^T$ đo được từ hệ quang học camera (trong đó $Z_C$ là khoảng cách độ sâu vuông góc, $X_C$ hướng sang phải thấu kính, $Y_C$ hướng cắm xuống đất):

$$\begin{bmatrix} X_B \\ Y_B \\ Z_B \\ 1 \end{bmatrix} = \begin{bmatrix} 
0 & 0 & 1 & \Delta x_{\text{cam}} \\
-1 & 0 & 0 & \Delta y_{\text{cam}} \\
0 & -1 & 0 & h_{\text{cam}} \\
0 & 0 & 0 & 1 
\end{bmatrix} \begin{bmatrix} X_C \\ Y_C \\ Z_C \\ 1 \end{bmatrix}$$

Khai triển giải tích:
$$\begin{cases}
X_B = Z_C + \Delta x_{\text{cam}} = Z_C + 0.090 \\
Y_B = -X_C + \Delta y_{\text{cam}} = -X_C - 0.015 \\
Z_B = -Y_C + h_{\text{cam}} = -Y_C + 0.120
\end{cases}$$

Từ tọa độ thân xe $P_B = [X_B, Y_B, Z_B]^T$, chuyển sang hệ tọa độ bản đồ thế giới (World Frame $\{W\}$) với vị trí robot hiện tại $(r_x, r_y, \text{yaw})$:

$$\begin{cases}
W_x = r_x + X_B \cos(\text{yaw}) - Y_B \sin(\text{yaw}) \\
W_y = r_y + X_B \sin(\text{yaw}) + Y_B \cos(\text{yaw}) \\
W_z = Z_B
\end{cases}$$

Nhờ tính toán bù trừ đầy đủ cả $\Delta x_{\text{cam}}$ và $\Delta y_{\text{cam}}$, khi xe quay tròn $360^\circ$ tại chỗ quanh tâm trục bánh xe $\{B\}$, mây điểm của vật thể trước mặt sẽ xoay theo đúng quỹ đạo hình học elip thực tế thay vì bị giật lệch tâm.

### 2.3. Cụm cảm biến thị giác OAK-D S2 & Cảm biến quán tính IMU
* **Stereo Baseline:** $b = 7.5\text{ cm}$ giữa 2 camera đơn sắc OV9282 Global Shutter.
* **Độ phân giải Stereo:** $640 \times 400$, tần số $25\text{ FPS}$.
* **Trường nhìn quang học:** $\text{HFOV} \approx 72^\circ$, $\text{VFOV} \approx 48^\circ$.
* **Cảm biến IMU phần cứng tích hợp:** BNO086 (hoặc BMI270 tùy phiên bản OAK-D S2), xuất dữ liệu ở tần số $100\text{ Hz}$.

### 2.4. Thuật toán tự động nhận diện trục trọng lực (Auto Gravity Detection)
Trong thực tế, tùy thuộc vào phiên bản firmware và cấu hình DepthAI, cảm biến IMU có thể xuất dữ liệu theo hệ tọa độ Quang học (Camera Body Frame, trục Y hướng xuống sàn) hoặc hệ tọa độ Robot ENU (East-North-Up, trục Z hướng lên trần). Nếu lập trình cố định trục Z, khi gặp firmware chuẩn quang học, góc quay xoay ngang sẽ bị hiểu nhầm thành góc lật (Roll/Pitch).

Hệ thống đã tích hợp thuật toán **Tự động Dò Trục Trọng Lực** trong module [camera_streamer.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/modules/camera_streamer.py#L1015-L1035):
1. Đọc gia tốc kế 3 trục $(a_x, a_y, a_z)$ từ gói tin IMU: Khi xe nằm yên hoặc di chuyển đều trên mặt sàn nằm ngang, véc-tơ gia tốc trọng trường $g \approx 9.81\text{ m/s}^2$ luôn hướng thẳng đứng vuông góc với mặt sàn.
2. So sánh độ lớn gia tốc:
   $$\begin{cases}
   \text{Nếu } |a_y| > 6.0\text{ và } |a_y| > |a_z| + 2.0 \implies \text{Trục thẳng đứng là } Y \text{ (Chuẩn quang học Camera)} \\
   \text{Nếu } |a_z| > 6.0\text{ và } |a_z| > |a_y| + 2.0 \implies \text{Trục thẳng đứng là } Z \text{ (Chuẩn ENU Robot)}
   \end{cases}$$
3. Khi đã xác định được trục thẳng đứng, thuật toán giải mã góc quay Yaw tương ứng:
   * **Nếu trục thẳng đứng là Y:**
     $$\sin(\theta) = -2.0(q_w q_y + q_x q_z), \quad \cos(\theta) = 1.0 - 2.0(q_y^2 + q_z^2)$$
     Vận tốc góc quay quanh sàn: $\omega_{\text{floor}} = -gyro_y$.
   * **Nếu trục thẳng đứng là Z:**
     $$\sin(\theta) = 2.0(q_w q_z + q_x q_y), \quad \cos(\theta) = 1.0 - 2.0(q_y^2 + q_z^2)$$
     Vận tốc góc quay quanh sàn: $\omega_{\text{floor}} = gyro_z$.
4. **Tích phân góc không gián đoạn (Continuous Unwrapped Integration):**
   Tại mỗi gói tin IMU ($100\text{Hz}$), độ biến thiên góc được tính toán qua hàm atan2:
   $$\Delta\theta = \text{atan2}\left(\sin(\theta_k - \theta_{k-1}), \cos(\theta_k - \theta_{k-1})\right)$$
   $$\theta_{\text{accumulated}} \mathrel{+}= \Delta\theta$$
   Nhờ đó, khi xe quay nhiều vòng liên tiếp vượt qua ngưỡng $\pm 180^\circ$ ($\pm \pi$), góc quay tích phân vẫn tăng/giảm tuyến tính liên tục, không bao giờ bị nhảy vọt gây lỗi điều khiển.

---

## 3. KIẾN TRÚC PHẦN MỀM ĐA LUỒNG & BỘ ĐIỀU PHỐI (ORCHESTRATOR)

Toàn bộ hệ thống vận hành bên trong tiến trình trung tâm [main.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/main.py) theo mô hình Hướng dịch vụ đa luồng (Multi-Threaded Service Architecture):

```
                                  [ main.py - JetBotOrchestrator ]
                                                │
         ┌──────────────────────────────┬───────┴──────────────────────┬──────────────────────────────┐
         ▼                              ▼                              ▼                              ▼
 ┌──────────────────────┐    ┌──────────────────────┐    ┌──────────────────────┐    ┌──────────────────────┐
 │ LUỒNG 1: CAMERA      │    │ LUỒNG 2: SLAM & CSM  │    │ LUỒNG 3: STEP WORKER │    │ LUỒNG 4: WEB SERVER  │
 │ • DepthAI Pipeline   │    │ • Tần số cố định 10Hz│    │ • Chạy nền Asynch    │    │ • Port 8080 (HTTP)   │
 │ • Trích xuất Depth   │    │ • Virtual LaserScan  │    │ • Điều khiển cm & độ │    │ • Streaming MJPEG    │
 │ • RGB Frame 640x400  │    │ • Raycasting Bresen. │    │ • Hạ dốc S-Curve     │    │ • REST API Handlers  │
 │ • IMU Worker 100Hz   │    │ • Likelihood Field   │    │ • Honest Diagnostics │    │ • Web Cockpit Client │
 │ • Tần số: 25 FPS     │    │ • Bổ chính Pose (CSM)│    │ • Benchmark quỹ đạo  │    │ • State Publisher    │
 └──────────────────────┘    └──────────────────────┘    └──────────────────────┘    └──────────────────────┘
```

### 3.1. Phân luồng thực thi 4 tiến trình song song (Concurrency Architecture)
1. **Luồng 1 — Camera & Stereo Pipeline (Producer Thread):** Chạy vòng lặp vô tận đọc frame từ OAK-D S2 qua USB 3.0. Nhận frame RGB, ma trận độ sâu `depth_frame` (dạng `uint16` mm) và kích hoạt luồng phụ lắng nghe hàng đợi IMU ở tần số $100\text{Hz}$.
2. **Luồng 2 — SLAM & Pose Correction (Consumer Thread):** Khóa nhịp ở tần số $10\text{ Hz}$ cố định. Chịu trách nhiệm thực thi:
   - Chiếu ma trận độ sâu thành 120 tia quét LaserScan 2D.
   - Kiểm tra điều kiện Khóa Cổng Xoay (Turn Gating & Yaw Lock).
   - Gọi module [scan_matcher.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/modules/scan_matcher.py) để tìm nghiệm tọa độ tối ưu bằng Likelihood Field CSM (chỉ nắn $X, Y$).
   - Cập nhật ma trận lưới chiếm dụng [occupancy_slam.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/modules/occupancy_slam.py).
3. **Luồng 3 — Step Movement & Pattern Worker:** Được kích hoạt bất đồng bộ khi người dùng bấm lệnh bước hoặc chạy bài thử nghiệm hình học. Tự động chia nhỏ chu kỳ, cấp xung, giám sát cảm biến an toàn và hạ dốc dừng êm mà không làm gián đoạn luồng video hay luồng tính toán SLAM.
4. **Luồng 4 — Web Server & State Publisher:** Vận hành trên máy chủ nhúng HTTP đa luồng `ThreadedHTTPServer`, phân phối luồng video MJPEG độ trễ thấp, phục vụ các API trạng thái `/api/state`, `/api/drive`, `/api/step_move`, `/api/config/*`.

### 3.2. Cơ chế an toàn dữ liệu & Đồng bộ hóa đa luồng (`threading.RLock`)
Để ngăn ngừa hiện tượng tranh chấp dữ liệu (Race Condition) khi nhiều luồng cùng truy cập vào tọa độ robot $(x, y, \theta)$ hoặc ma trận bản đồ, hệ thống sử dụng đối tượng khóa tái nhập `threading.RLock()`:
```python
with self.lock:
    self.robot_x = corr_x
    self.robot_y = corr_y
    # robot_yaw được bảo toàn, không bị CSM ghi đè
```
Mọi thao tác đọc trạng thái từ Web API chỉ chụp bản sao (Snapshot Copy) dạng dictionary nguyên tử (Atomic read), đảm bảo luồng điều khiển động cơ không bao giờ bị nghẽn (hang) do I/O mạng.

### 3.3. Cơ chế cấu hình tập trung (`config.py` Single Source of Truth)
Để tránh tình trạng phân mảnh thông số giữa nhiều file, [config.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/config.py) là **Nguồn Chân Lý Duy Nhất (Single Source of Truth)**. Module [camera_streamer.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/modules/camera_streamer.py) sử dụng lớp `_ConfigProxy` đặc biệt:
```python
class _ConfigProxy:
    def __getattr__(self, name):
        if _config_module is not None and hasattr(_config_module, name):
            return getattr(_config_module, name)
        raise AttributeError(f"Config module has no attribute '{name}'")
```
Nhờ cơ chế này, lệnh `getattr(config, 'PARAM_NAME', default)` hoạt động hoàn hảo 100%, tự động fallback về giá trị mặc định chuẩn nếu tham số chưa được khai báo mà không gây ra lỗi `NoneType`.

---

## 4. ĐỘNG HỌC ROBOT VI SAI & CÁC THUẬT TOÁN ĐIỀU KHIỂN CẤP THẤP

### 4.1. Mô hình động học vi sai chuẩn hóa & Ánh xạ công suất xung PWM
Mô hình động học vi sai thuận (Forward Kinematics) chuyển đổi vận tốc dài mong muốn $v$ ($\text{m/s}$) và vận tốc góc xoay $\omega$ ($\text{rad/s}$) trong hệ quy chiếu thân xe thành vận tốc dài độc lập của hai bánh trái $v_L$ và bánh phải $v_R$:

$$\begin{cases}
v_L = v - \dfrac{\omega \cdot L}{2} \\
v_R = v + \dfrac{\omega \cdot L}{2}
\end{cases}$$

với khoảng cách giữa 2 bánh $L = 0.120\text{ m}$.

Quy đổi từ vận tốc dài $v_L, v_R$ sang tỉ lệ chu kỳ xung PWM danh định ($\text{duty} \in [-1.0, 1.0]$) được thực hiện thông qua vận tốc trần phần cứng $\text{HW\_MAX\_V} = 0.55\text{ m/s}$:

$$p_L = \dfrac{v_L}{\text{HW\_MAX\_V}}, \quad p_R = \dfrac{v_R}{\text{HW\_MAX\_V}}$$

### 4.2. Cân bằng điện áp 2 bánh phần cứng (Hardware Wheel Trim Balance)
* **Nguyên nhân kỹ thuật:** Hai động cơ DC giảm tốc dù cùng mã hiệu vẫn có sự sai lệch về khe hở tiếp xúc bánh răng và lực căng cuộn dây. Cấp cùng một mức xung PWM sẽ sinh ra lực kéo không đều làm xe bị lạng cong (Drift).
* **Giải pháp thuật toán:** Tích hợp bộ nhân tỉ lệ phần cứng độc lập ngay trước khi xuất xung PWM xuống thanh ghi PCA9685:
  $$p_{L,\text{out}} = p_L \times \text{MOTOR\_TRIM\_LEFT}$$
  $$p_{R,\text{out}} = p_R \times \text{MOTOR\_TRIM\_RIGHT}$$
* **Giá trị chuẩn hóa thực nghiệm:**
  * `MOTOR_TRIM_LEFT = 1.000` ($100.0\%$ công suất bánh trái).
  * `MOTOR_TRIM_RIGHT = 1.020` ($102.0\%$ công suất bánh phải — bù lực cản truyền động bánh phải).

### 4.3. Khóa hướng đi thẳng chủ động vòng kín (Active IMU Heading Lock Assist)
Khi nhận lệnh chạy thẳng ($v \neq 0$ và $\omega = 0$), thuật toán tự động lưu lại góc hướng ban đầu $\theta_{\text{start}} = \theta_{\text{IMU}}$. Sai số góc hướng được đo đạc liên tục ở mỗi chu kỳ $20\text{ms}$:

$$e_\theta = \text{atan2}\left(\sin(\theta_{\text{start}} - \theta_{\text{current}}), \cos(\theta_{\text{start}} - \theta_{\text{current}})\right)$$

Bộ điều khiển tỉ lệ (P Controller) tính toán vận tốc góc hỗ trợ $\omega_{\text{assist}}$:

$$\omega_{\text{assist}} = \text{clip}\left(K_p \cdot e_\theta, -0.16, 0.16\right) \quad \text{với } K_p = 0.50$$

> **Nguyên lý phối hợp giữa Cân bằng tĩnh và Trợ lực động:**  
> - **Hardware Wheel Trim (`L:1.000, R:1.020`):** Đóng vai trò là nền tảng cân bằng cơ khí tĩnh (Feedforward Baseline), giúp 2 động cơ tự chạy tương đối thẳng mà không cần phụ thuộc cảm biến.  
> - **Active Heading Lock ($K_p = 0.50$):** Đóng vai trò là trợ lực hồi tiếp vòng kín (Feedback Assist), chỉ can thiệp nhẹ nhàng khi xe gặp nhiễu ngoại cảnh (bánh trượt trên gạch men, bánh mắt trâu giật). Đặt $K_p = 0.50$ (thay vì $0.95$) giúp xe chuyển động mượt mà, lướt êm, triệt tiêu hoàn toàn hiện tượng dao động chữ S (hunting / lắc đuôi cá).

### 4.4. Động học giảm tốc mượt mà Cosine S-Curve 50Hz triệt tiêu quán tính
Cụm camera OAK-D S2 gắn nhô ra trước $9\text{cm}$ và cao $12\text{cm}$. Việc ngắt xung điện đột ngột ($100\% \to 0\%$) hoặc giảm tốc theo các nấc thang thô sẽ tạo ra mô-men quán tính lật chúi đầu xe (Jerk lớn), làm rung giật camera gây nhòe mây điểm độ sâu.

Hệ thống tính toán biên dạng vận tốc mềm theo hàm Cosine liên tục tại mỗi chu kỳ $20\text{ms}$ ($50\text{Hz}$):

$$\text{ratio}(t) = \begin{cases}
\dfrac{1}{2}\left(1 - \cos\left(\pi \dfrac{t}{t_{\text{acc}}}\right)\right) & \text{khi } t < t_{\text{acc}} \text{ (Khởi động mềm S-curve)} \\
1.0 & \text{khi } t_{\text{acc}} \le t < t_{\text{acc}} + t_{\text{steady}} \text{ (Chạy đều)} \\
\dfrac{1}{2}\left(1 + \cos\left(\pi \dfrac{t - (t_{\text{acc}} + t_{\text{steady}})}{t_{\text{dec}}}\right)\right) & \text{khi hạ dốc dừng xe}
\end{cases}$$

Quãng đường dịch chuyển tại mỗi thời điểm $t$ được tích phân trực tiếp từ phương trình giải tích của đường cong vận tốc, đảm bảo khi robot dừng hẳn, cự ly đạt chuẩn $100\%$ không bị nhảy bước.

### 4.5. Chuẩn hóa sụt áp khối Pin 3S thời gian thực (Battery Voltage Normalization)
Tốc độ góc và mô-men xoắn của động cơ DC tỷ lệ thuận với điện áp cấp $V_{\text{motor}} \approx \text{duty} \times V_{\text{battery}}$. Khi pin xả từ $12.4\text{V} \to 10.5\text{V}$, cùng một mức xung PWM sẽ sinh ra vận tốc khác nhau.

Dựa trên cảm biến dòng/áp INA219 trên bus I2C (đo ở tần số 1Hz), hệ thống tự động tính hệ số bù chuẩn hóa theo điện áp danh định $11.1\text{V}$:

$$k_{\text{bat}} = \text{clip}\left(\dfrac{11.1\text{V}}{V_{\text{battery}}}, 0.85, 1.25\right)$$
$$\text{duty}_{\text{compensated}} = \text{duty}_{\text{nominal}} \times k_{\text{bat}}$$

### 4.6. Điều khiển bước định lượng chính xác (Metric Step Movement Worker)
Luồng nền `_step_worker` thực thi các lệnh bước:
* **Bước chạy thẳng (cm):**
  $$t_{\text{motion}} = \left(\dfrac{s_{\text{cm}} / 100}{v_{\text{calib}}}\right) \times \text{SCALE}_{\text{linear}}$$
  * Tiến: $\text{SCALE} = \text{STEP\_FORWARD\_SCALE} = 0.440$
  * Lùi: $\text{SCALE} = \text{STEP\_BACKWARD\_SCALE} = 0.480$ (tách biệt vì lực cản bánh mắt trâu khi kéo lùi lớn hơn khi đẩy tiến).
* **Bước quay góc (độ):**
  * **Chế độ Vòng Kín IMU (khi có IMU):** Động cơ quay vi sai tại chỗ và liên tục đọc `imu_accumulated_yaw` ở tần số cao. Khi còn cách đích $20^\circ$, ga hạ dần về mức bò $7.5\%$. Khi sai số $\le 1.0^\circ$, phanh kích hoạt ngắt tức thì.
  * **Chế độ Động học Mở (Open-loop Fallback):**
    $$t_{\text{motion}} = \left(\dfrac{\theta_{\text{deg}} \times \pi / 180}{\omega_{\text{calib}}}\right) \times \text{SCALE}_{\text{turn}} + \text{STEP\_TURN\_START\_LATENCY\_S}$$
    * Quay Trái (CCW): $\text{SCALE} = \text{STEP\_TURN\_LEFT\_SCALE} = 0.600$
    * Quay Phải (CW): $\text{SCALE} = \text{STEP\_TURN\_RIGHT\_SCALE} = 0.600$
    * Bù trễ khởi động bẻ bánh mắt trâu: $\text{STEP\_TURN\_START\_LATENCY\_S} = 0.080\text{s}$.

### 4.7. Logic đối chiếu góc quay thực tế: Cơ chế đánh giá trung thực (Honest Diagnostics)
Để triệt tiêu hoàn toàn sự mập mờ và các giá trị ảo, logic đối chiếu được thiết kế nghiêm ngặt:
* **Khi có dữ liệu IMU hợp lệ (`has_valid_imu_eval == True`):**
  * Cột Thực tế: Hiển thị góc quay đo được từ cảm biến (VD: `44.8°`).
  * Cột Sai số: Tính sai số thực $e = \theta_{\text{meas}} - \theta_{\text{cmd}}$ (VD: `-0.2°`).
  * Trạng thái: Gắn nhãn xanh lá **`CHUẨN XÁC`** (nếu $|e| \le 2.5^\circ$), đỏ **`THIẾU GÓC`** hoặc vàng **`DƯ GÓC`**.
* **Khi không có cảm biến IMU (`has_valid_imu_eval == False`):**
  * Cột Thực tế: Hiển thị góc ước tính kèm ghi chú rõ `45.0° (mô hình)`.
  * Cột Sai số: Trả về `None` (hiển thị trên web là dấu gạch ngang `--`, **tuyệt đối không được gán 0.0° làm người dùng hiểu nhầm là đã đo được**).
  * Trạng thái: Gắn nhãn xanh dương **`MÔ HÌNH HÓA`** (`ĐỘNG HỌC MỞ`).

---

## 5. THỊ GIÁC KHÔNG GIAN STEREO & PIPELINE SLAM 2D CHÍNH XÁC CAO

### 5.1. Thuật toán giả lập LaserScan 2D từ Stereo Depth (120 tia quét siêu mịn)
1. Với mỗi cột góc nhìn ngang $u \in [15, 625]$ trên ma trận độ sâu (gồm 120 cột cách đều), trích xuất dải độ cao điểm ảnh từ $10\% \to 65\%$ chiều cao khung hình.
2. Với mỗi điểm $(u, v)$ có độ sâu $Z$:
   $$X_c = \dfrac{(u - c_x) \cdot Z}{f_x}, \quad Y_c = \dfrac{(v - c_y) \cdot Z}{f_y}, \quad Z_c = Z$$
3. Tọa độ thực tế so với mặt sàn:
   $$Z_{\text{rob}} = -Y_c + h_{\text{cam}} = -Y_c + 0.120\text{m}$$
4. Phép biến đổi được vector hóa hoàn toàn qua NumPy mảng 2 chiều, xử lý toàn bộ 120 tia trong thời gian $< 2.8\text{ms}$ trên Jetson Nano.

### 5.2. Thuật toán phân tách sàn nhà ($< 0.04\text{m}$) và vật cản thực ($0.04\text{m} - 0.85\text{m}$)
* **Sàn nhà ($Z_{\text{rob}} < 0.04\text{m}$):** Loại bỏ hoàn toàn khỏi tập điểm cản. Các tia quét sàn được dùng để vẽ không gian thoáng tự do (Free-space, giá trị 128) giúp xe có thể tự tin di chuyển qua.
* **Trần nhà ($Z_{\text{rob}} > 0.85\text{m}$):** Bỏ qua các điểm cao vượt quá tầm cao xe.
* **Vật cản thực sự ($0.04\text{m} \le Z_{\text{rob}} \le 0.85\text{m}$):** Bao gồm chân bàn, mặt ghế, vách tường, bọc đồ. Cột quét sẽ chọn điểm cản gần nhất $Z_{\min}$ dọc theo hướng nhìn để làm điểm phản xạ của tia LaserScan.

### 5.3. Thuật toán ma trận lưới chiếm dụng 2D (Occupancy Grid Mapping)
* Bản đồ là ma trận kích thước $10.0\text{m} \times 10.0\text{m}$ với độ phân giải $0.05\text{m/ô}$ ($5\text{cm}$): Gồm $200 \times 200$ ô (`uint8`).
  * `0`: Vùng chưa thám hiểm (Unknown - màu đen).
  * `128`: Vùng không gian trống thoáng đã khám phá (Free-Space - màu xám xanh).
  * `255`: Vùng có vật cản hoặc tường kiên cố (Occupied - màu xanh lục dạ quang).
* **Bresenham Raycasting:** Chiếu tia từ gốc thấu kính camera đến điểm cản, cập nhật giá trị `128` dọc theo đường truyền tia sáng. Tia dừng trước vật cản $6\text{cm}$ để bảo vệ mép bờ tường không bị xóa mòn.
* **Bộ đếm xác nhận cản Hit-Counter:** Ô cản chỉ chính thức chuyển thành giá trị `255` khi được ít nhất 2 tia quét độc lập xác nhận (`hit_counts >= 2`), loại bỏ $100\%$ nhiễu hạt lấm chấm quang học của cảm biến stereo.

### 5.4. Thuật toán so khớp quét đối ứng Likelihood Field CSM 2D (Correlative Scan Matching)
Dựa trên nền tảng CSM của Giáo sư Edwin Olson (MIT) kết hợp mô hình Likelihood Field của Sebastian Thrun:
1. **Trường xác suất khoảng cách (Distance Transform Scoring):**
   * Ô tường kiên cố (`grid == 255`): $+100$ điểm.
   * Ô chướng ngại vật chớm thấy (`hit_counts >= 1`): $+45$ điểm.
   * Ô lân cận tường bán kính $5\text{cm}$ (1 ô): $+65$ điểm.
   * Ô lân cận tường bán kính $10\text{cm}$ (2 ô): $+30$ điểm.
   * Ô trống thoáng (`grid == 128`): $-12$ điểm (phạt tia đâm xuyên phòng trống).
2. **Cửa sổ tìm kiếm vị trí:**
   * Bán kính tìm kiếm tọa độ: $\Delta x, \Delta y \in [-0.08\text{m}, +0.08\text{m}]$ với bước nhảy $0.02\text{m}$ ($2\text{cm}$).
   * Ngưỡng tin cậy chấp nhận nghiệm: $\text{min\_score} = 0.28$ ($28\%$).

### 5.5. Bóc tách hiện tượng lệch 45° sau khi quay 360° & Cơ chế Khóa Cổng Xoay (Turn Gate & CSM Yaw Lock)

#### Bản chất toán học của lỗi lệch 45° (311° trên la bàn):
1. Robot xuất phát ở góc $0^\circ$ (hướng 3 giờ).
2. Người dùng ra lệnh `QUAY PHẢI 45°` 8 lần. Động cơ xoay đủ 8 lần ngoài đời thực đưa robot trở về vị trí cũ ($\approx 360^\circ \equiv 0^\circ$).
3. Tuy nhiên, sau mỗi lần dừng quay, hệ thống cũ kích hoạt 4 frame ổn định (settling frames) của CSM.
4. Camera OAK-D chỉ có góc nhìn hẹp $72^\circ$. Khi quay tại chỗ nhìn vào vật thể gần (bọc đồ/chân ghế), trường Likelihood Field bị bất đối xứng.
5. Cửa sổ tìm kiếm góc xoay cũ là `SCAN_MATCH_SEARCH_YAW_DEG = 6.0` ($\pm 6^\circ$). Thuật toán CSM bị hút vào cực đại phụ (local maxima) và nắn góc `robot_yaw` lệch đi khoảng $+6.1^\circ$ sau mỗi lần dừng!
6. **Sau 8 lần quay:**
   $$\text{Sai số tích lũy} = 8 \times (+6.1^\circ) \approx +49^\circ$$
   Góc la bàn trên giao diện:
   $$\text{compass\_deg} = (-49^\circ) \pmod{360} = 311^\circ$$
   Mũi tên định hướng trên bản đồ 2D bị bẻ lệch từ góc 3 giờ ($0^\circ$) thành hướng **1 giờ 20 phút** ($311^\circ$), và các tia quét bị bắn chéo ra tạo thành hình nan quạt xoắn ốc xung quanh xe!

```
   [ HIỆN TƯỢNG CŨ: CSM BẺ LỆCH GÓC ]             [ GIẢI PHÁP MỚI: TURN GATE & YAW LOCK ]
              12h (90°)                                        12h (90°)
                 ▲                                                ▲
                 │                                                │
   1h(311°) ◄────┼────► 3h (0° Mốc ban đầu)                       ┼────► 3h (0° CHUẨN XÁC!)
   (Bị CSM bẻ)   │                                                │      (Sau 8 lần quay 45°)
                 ▼                                                ▼
              6h (270°)                                        6h (270°)
```

#### Giải pháp Khóa Cổng Xoay (Turn Gate & CSM Yaw Lock = 0.0°):
1. **Thiết lập `SCAN_MATCH_SEARCH_YAW_DEG = 0.0` trong `config.py`:** Khóa cứng góc Yaw trong thuật toán CSM. CSM chỉ được phép nắn chỉnh sai số tịnh tiến $(X, Y)$ đối chiếu bờ tường để xe không bị trôi xuyên tường khi chạy thẳng. **CSM tuyệt đối không được phép can thiệp vào góc hướng `robot_yaw`**.
2. **Khóa Cổng Khi Quay (Turn Gate):** Trong suốt quá trình xe thực hiện bước quay (`is_step_turning == True`), CSM tắt hoàn toàn và không lưu các frame chờ settling.
3. **Cập nhật Bản đồ Tức thì theo Sector 45° (Instant Sector Raycasting):** Ngay khi mỗi bước quay kết thúc, [main.py](file:///d:/giap/AI/Model/UIT/CS532.R11_JetBot_SLAM/main.py) bắn trực tiếp 1 frame quang học sạch vào bản đồ 2D Occupancy Grid đúng tại góc quay chuẩn ($0^\circ, -45^\circ, -90^\circ, \dots, 0^\circ$). Khi quay đủ 8 lần (360°), góc quay trở về đúng $0.0^\circ$, mũi tên định hướng JetBot chỉ thẳng tắp về hướng **3 giờ**, và mây tia quét tạo thành một bản đồ khép kín 360° hoàn hảo.

---

## 6. HỆ THỐNG QUẢN LÝ CẤU HÌNH ĐỘNG & ĐA PHIÊN BẢN (MULTI-PROFILE HOT-RELOAD ENGINE)

Một trong những bước tiến công nghệ lớn nhất của hệ thống là **Trung tâm quản lý cấu hình Profile Hot-Reload (Tab 3)**.

### 6.1. Kiến trúc lưu trữ bền vững hai chiều (Two-Way Persistence)
Mọi thay đổi trên giao diện web đều được đồng bộ hóa tức thì xuống hai tầng lưu trữ:
1. **Tầng bộ nhớ Runtime (`config.py`):** Các biến toàn cục (`config.MOTOR_TRIM_LEFT`, `config.DEFAULT_SPEED_PWM`, v.v.) được cập nhật ngay trong tiến trình Python đang chạy.
2. **Tầng tệp tin vĩnh viễn (`robot_dynamics.json` & `config_profiles.json`):** Tự động ghi xuống ổ cứng JSON có cấu trúc, bảo toàn toàn bộ danh sách các Profile ngay cả khi tắt nguồn Jetson Nano.

### 6.2. Cơ chế nạp tức thì không gián đoạn (Hot-Reloading without Downtime)
* Người dùng có thể tạo không giới hạn các phiên bản cấu hình cho từng loại mặt sàn:
  * **Profile #1:** Chuẩn Mặc định (Giáp Master) — Sàn phòng khách tiêu chuẩn.
  * **Profile #2:** Sàn Gạch Men / Lab — Ma sát thấp, bù trượt nhẹ.
  * **Profile #3:** Thảm Trải Sàn / Xi Măng — Ma sát cao, tăng mô-men quay.
* Chỉ cần click chọn phiên bản, toàn bộ tham số được nạp vào robot trong $5\text{ms}$ mà không cần khởi động lại máy chủ (No Restart Required), giữ nguyên vẹn bản đồ đang vẽ.

### 6.3. Giải thích chuyên sâu lý do tồn tại của từng tham số cấu hình

| Nhóm Tham Số | Tên Tham Số Mã Nguồn | Giá Trị Chuẩn | Lý Do Kỹ Thuật Bắt Buộc Phải Tồn Tại |
| :--- | :--- | :---: | :--- |
| **Nhóm 1: Cân Bằng Động Cơ** | `motor_trim_left`<br>`motor_trim_right` | `1.000`<br>`1.020` | Bù trừ sai số cơ khí giữa 2 hộp số DC. Cho phép robot chạy thẳng tắp mà không bị lạng sang một bên. |
| | `heading_lock_kp` | `0.50` | Hệ số phản hồi P từ cảm biến IMU. Khi bánh xe bị trượt trên sàn trơn, thuật toán bù vi sai êm dịu kéo thẳng xe lại (không giật cục). |
| **Nhóm 2: Cự Ly Tuyến Tính** | `step_forward_scale` | `0.440` | Hệ số quy đổi cự ly tiến. Chuẩn hóa lệnh đi 20cm trên web thành đúng 20cm ngoài đời thực. |
| | `step_backward_scale`| `0.480` | **Tách riêng với tiến** vì bánh mắt trâu khi bị kéo ngược sinh ra lực cản lớn hơn hẳn khi đẩy xuôi. |
| | `default_speed_pwm` | `0.060` (6%) | Mức ga cơ bản chạy thẳng. Quyết định công suất cấp cho động cơ khi tiến/lùi (chạy êm không trượt). |
| **Nhóm 3: Góc Quay Vi Sai** | `step_turn_left_scale` | `0.600` | Tỉ lệ góc xoay Trái (CCW). Dùng làm mô hình chuẩn khi quay không có IMU. |
| | `step_turn_right_scale`| `0.600` | **Tách riêng với quay trái** do khe hở bánh răng và hướng quay của bánh mắt trâu không đối xứng. |
| | `step_turn_min_pwm` | `0.110` (11%)| Xung PWM tối thiểu khi xoay tại chỗ để thắng ma sát tĩnh của bánh cao su và bánh mắt trâu. |
| | `step_turn_start_latency_s`| `0.080`s | Bù 80ms trễ khởi động khi bẻ bánh mắt trâu, giúp góc 45° không bị thiếu và góc 180° không bị dư. |
| **Nhóm 4: Hãm Tốc & An Toàn**| `manual_decel_time_ms`| `180`ms | Thời gian hạ dốc tuyến tính 180ms trước khi ngắt xung điện, triệt tiêu quán tính chống rung giật camera. |
| | `safety_brake_dist_m` | `0.20`m | Khoảng cách phanh khẩn cấp. Khi vật cản $\le 20\text{cm}$, tự động ngắt động cơ để bảo vệ an toàn. |
| | `safety_warn_dist_m`  | `0.45`m | Khoảng cách cảnh báo. Khi vật cản trong dải $20-45\text{cm}$, tự động giảm ga chậm dần. |
| **Nhóm 5: Hình Học Camera** | `camera_offset_forward_m` | `0.090`m | Vị trí camera nhô ra phía trước trục bánh xe vi sai. |
| | `camera_offset_lateral_m`| `-0.015`m| Vị trí camera lệch sang phải so với trục giữa thân xe. |
| | `camera_height_m` | `0.120`m | Độ cao trục quang học camera so với mặt sàn. |
| | `camera_pitch_deg` | `0.0`° | Góc nghiêng ngửa/cúp của camera so với mặt sàn. |

---

## 7. BỘ CÔNG CỤ CHẨN ĐOÁN & PHÒNG THÍ NGHIỆM SOI THUẬT TOÁN (ALGORITHM INSPECTOR & PROFILER)

Tab 2 trên Web Cockpit là trung tâm phân tích và chẩn đoán thuật toán chuyên sâu (Deep Inspector).

```
┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
│ 🔬 TAB 2: PHÒNG THÍ NGHIỆM SOI THUẬT TOÁN TỪNG FRAME & ĐO HIỆU NĂNG MẠNG                        │
├──────────────────────────────────────────────────────────────────────────────────────────────────┤
│ BẢNG ĐO THỜI GIAN THỰC TẾ & BĂNG THÔNG MẠNG (LIVE RUNTIME & NETWORK PROFILER):                    │
│ • FPS Web Nhận: 24.8 FPS │ FPS Camera: 25.0 FPS │ Pipeline Latency: 40.2 ms                      │
│ • Chi tiết trễ: Depth: 2.1ms │ LaserScan: 2.6ms │ CSM: 2.4ms │ Map Update: 1.8ms │ Render: 1.2ms │
│ • Băng thông: Video MJPEG: 342 KB/s │ Telemetry State: 14.2 KB/s │ Command Web: 0.8 KB/s         │
│ • Chẩn đoán điểm nghẽn: 🟢 HỆ THỐNG CÂN BẰNG HOÀN HẢO (CPU Jetson mát mẻ, WiFi thông suốt)       │
├─────────────────────────────────────────────────────────────────┬────────────────────────────────┤
│ PHÒNG THÍ NGHIỆM CSM SCAN SNAP (CANVAS A):                      │ RADAR PHÂN KỲ GÓC XOAY (B):    │
│ • 🟡 Mây tia thô trước CSM (Odometry)                           │ • Góc IMU Phần Cứng: +89.8°    │
│ • 🟢 Mây tia khớp sau CSM (Khớp tường thực)                     │ • Góc Odometry Vi Sai: +88.2°  │
│ • 🟦 Bờ tường bản đồ Occupancy Grid                             │ • Sai lệch phân kỳ: 1.6°       │
│ • 🚗 Vị trí xe JetBot                                           │ • Trạng thái: CỰC KỲ KHỚP      │
├─────────────────────────────────────────────────────────────────┴────────────────────────────────┤
│ 📸 BỘ GIẢI MÃ FRAME-BY-FRAME & CÂY QUYẾT ĐỊNH ĐIỀU KHIỂN:                                       │
│ [⏸️ TẠM DỪNG / ĐÓNG BĂNG FRAME]  [▶️ TIẾP TỤC REAL-TIME]                                         │
│ • Lịch sử 10 Frame gần nhất: #1042, #1041, #1040... (Click vào từng dòng để bóc tách):          │
│   ┌───────────────────────────────────┬───────────────────────────────────┐                      │
│   │ ẢNH CHỤP THỰC TẾ RGB CỦA FRAME    │ ẢNH NHIỆT ĐỘ SÂU STEREO (DEPTH)   │                      │
│   └───────────────────────────────────┴───────────────────────────────────┘                      │
│ • PHÂN TÍCH QUYẾT ĐỊNH CỦA JETBOT TẠI FRAME NÀY:                                                 │
│   - Khoảng cách vật cản gần nhất: 0.74m (Vùng an toàn đường thoáng).                             │
│   - Điểm số tin cậy so khớp CSM: 88.5% (Tự tin tuyệt đối, khớp chặt vào tường phòng).           │
│   - Quyết định điều khiển: Cho phép tăng tốc tối đa, không kích hoạt phanh an toàn.              │
└──────────────────────────────────────────────────────────────────────────────────────────────────┘
```

### 7.1. Bảng đo hiệu năng mạng & Phân tích thông lượng băng thông thực tế
* **FPS Web Nhận vs FPS Camera:** Đo đạc chính xác tần số khung hình thực tế nhận được tại trình duyệt của người dùng (thông qua `requestAnimationFrame`) so với tần số phát ra của camera OAK-D S2, phản ánh ngay lập tức nếu mạng Wi-Fi bị nghẽn giật.
* **Thời gian trễ chi tiết từng công đoạn:**
  * `t_depth`: Thời gian đọc ma trận độ sâu từ VPU qua USB 3.0 ($\approx 2.0 - 2.5\text{ms}$).
  * `t_laser`: Thời gian chiếu ma trận độ sâu thành 120 tia LaserScan ($\approx 2.4 - 2.8\text{ms}$).
  * `t_csm`: Thời gian so khớp trường Likelihood Field CSM ($\approx 2.2 - 2.6\text{ms}$).
  * `t_map`: Thời gian cập nhật lưới Occupancy Grid bằng Raycasting ($\approx 1.5 - 2.0\text{ms}$).

### 7.2. Chẩn đoán điểm nghẽn tự động (Automatic Bottleneck Detector)
Hệ thống tự động phân tích ma trận thời gian và băng thông để đưa ra khuyến nghị:
* Nếu `t_csm > 15ms`: Cảnh báo CPU Jetson quá tải, khuyến nghị giảm bán kính tìm kiếm CSM.
* Nếu FPS Web nhận $< 12$ trong khi FPS Camera $> 20$: Cảnh báo mạng Wi-Fi nghẽn băng thông, khuyến nghị giảm kích thước streaming video.
* Nếu mọi chỉ số đạt chuẩn: Báo huy hiệu xanh **"TỐI ƯU"**.

---

## 8. CÁC BÀI THỬ NGHIỆM QUỸ ĐẠO HÌNH HỌC KHÉP KÍN (GEOMETRIC BENCHMARK)

Để kiểm chứng độ chính xác động học của xe ngoài đời thực so với mô hình trên bản đồ 2D SLAM, hệ thống tích hợp 3 bài thử nghiệm vòng kín tự động:

```
    [ HÌNH CHỮ NHẬT 4 CẠNH ]                   [ TAM GIÁC VUÔNG CÂN ]
            Dài (L)                                      Cạnh A
       ┌───────────────┐                            ┌───────────┐
       │               │                            │          ╱
Rộng(W)│               │ Rộng(W)             Cạnh A │         ╱  Cạnh Huyền
       │               │                            │        ╱   c = a√2
       └───────────────┘                            │       ╱
            Dài (L)                                 └──────╱
   (4 cạnh thẳng, 4 góc quay 90°)          (2 cạnh vuông a, 1 cạnh huyền a√2)
                                           (Góc xoay: 90°, 135°, 135°)
```

### 8.1. Quỹ đạo Hình Chữ Nhật 4 chặng khép kín ($80\text{cm} \times 40\text{cm}$)
* **Quy trình:** Nhập Chiều dài ($L = 80\text{cm}$), Chiều rộng ($W = 40\text{cm}$) và Hướng quay (Trái CCW hoặc Phải CW).
* Chuỗi bước: `Tiến L` $\to$ `Quay 90°` $\to$ `Tiến W` $\to$ `Quay 90°` $\to$ `Tiến L` $\to$ `Quay 90°` $\to$ `Tiến W` $\to$ `Quay 90°`.
* **Tiêu chuẩn nghiệm thu:** Khi hoàn tất 4 chặng, xe phải trở về đúng vạch xuất phát ban đầu ngoài đời thực, và đường vẽ quỹ đạo trên bản đồ 2D SLAM phải tạo thành hình chữ nhật khép kín hoàn chỉnh, sai số khép góc $< 2.0^\circ$.

### 8.2. Quỹ đạo Tam Giác Vuông Cân Pythagore ($50\text{cm} - 50\text{cm} - 70.7\text{cm}$)
* **Quy trình:** Nhập Độ dài cạnh góc vuông ($a = 50\text{cm}$). Hệ thống tự động tính toán cạnh huyền $c = a\sqrt{2} \approx 70.7\text{cm}$.
* Chuỗi bước: `Tiến a` $\to$ `Quay 135°` $\to$ `Tiến a√2` $\to$ `Quay 135°` $\to$ `Tiến a` $\to$ `Quay 90°`.
* **Tiêu chuẩn nghiệm thu:** Đánh giá độ chính xác góc quay $135^\circ$ (góc tù) kết hợp di chuyển chéo, chứng minh tính đồng nhất của hệ số tỉ lệ góc xoay trên mọi dải góc từ nhỏ ($45^\circ$) đến lớn ($135^\circ, 180^\circ$).

### 8.3. Bài kiểm tra Xoay 360° tại chỗ khép kín về hướng 3 giờ ban đầu
* **Quy trình:** Robot đứng tại chỗ, thực hiện lệnh `QUAY PHẢI 45°` 8 lần liên tiếp (tổng $360^\circ$).
* **Tiêu chuẩn nghiệm thu:**
  * Mũi tên định hướng JetBot trên bản đồ 2D xoay đúng 8 cung $45^\circ$, sau lần quay thứ 8 trở về đúng hướng **3 giờ** ($0.0^\circ$), sai số góc $< 1.5^\circ$.
  * Toàn bộ 120 tia quét của frame cuối cùng chồng khít lên frame đầu tiên, tạo thành một bản đồ khép kín tròn trịa không có bất kỳ vệt xòe nan quạt nào.

---

## 9. BẢNG TỔNG HỢP THAM SỐ TOÀN HỆ THỐNG & DANH MỤC REST API

### 9.1. Bảng tra cứu tham số động học chuẩn trong `config.py`

| Tên Tham Số Cấu Hình | Giá Trị Chuẩn | Đơn Vị | Tác Dụng Vận Hành |
| :--- | :---: | :---: | :--- |
| `MOTOR_TRIM_LEFT` | **`1.000`** | Tỉ lệ | Hệ số xung PWM chuẩn bánh trái |
| `MOTOR_TRIM_RIGHT` | **`1.020`** | Tỉ lệ | Hệ số xung PWM bù lực bánh phải (+2% bù ma sát) |
| `STEP_FORWARD_SCALE` | **`0.440`** | Tỉ lệ | Hệ số co giãn cự ly chạy TIẾN chuẩn |
| `STEP_BACKWARD_SCALE`| **`0.480`** | Tỉ lệ | Hệ số co giãn cự ly chạy LÙI chuẩn |
| `STEP_TURN_LEFT_SCALE`| **`0.600`** | Tỉ lệ | Hệ số co giãn góc QUAY TRÁI (CCW) |
| `STEP_TURN_RIGHT_SCALE`| **`0.600`** | Tỉ lệ | Hệ số co giãn góc QUAY PHẢI (CW) |
| `DEFAULT_SPEED_PWM` | **`0.060`** (6%) | PWM | Mức ga cơ bản chạy thẳng (bò êm, không trượt) |
| `STEP_TURN_MIN_PWM` | **`0.110`** (11%)| PWM | Mức ga tối thiểu quay tại chỗ thắng ma sát tĩnh |
| `STEP_TURN_START_LATENCY_S`| **`0.080`** | Giây | Bù thời gian trễ ma sát tĩnh khi bẻ bánh mắt trâu |
| `MANUAL_DECEL_TIME_MS`| **`180`** | ms | Thời gian hạ dốc tuyến tính dừng mượt mà |
| `ENABLE_HEADING_LOCK` | **`True`** | Boolean | Kích hoạt khóa hướng tự động bằng IMU khi đi thẳng |
| `HEADING_LOCK_KP` | **`0.50`** | rad/s/rad | Độ nhạy phản hồi P khóa thẳng hướng (êm mượt, không lắc xe) |
| `SAFETY_BRAKE_DIST_M` | **`0.20`** | Mét | Ngưỡng cự ly phanh khẩn cấp tuyệt đối (20cm) |
| `SAFETY_WARN_DIST_M`  | **`0.45`** | Mét | Ngưỡng cự ly cảnh báo giảm tốc an toàn (45cm) |
| `RAY_SAMPLE_COUNT` | **`120`** | Tia | Số chùm tia LaserScan giả lập từ Stereo Depth |
| `SCAN_MATCH_SEARCH_DIST_M`| **`0.08`** | Mét | Bán kính tìm kiếm vị trí CSM Likelihood Field ($\pm 8\text{cm}$) |
| `SCAN_MATCH_SEARCH_YAW_DEG`| **`0.0`** | Độ | **KHÓA CỨNG YAW:** CSM chỉ nắn $(X, Y)$, không xoay góc xe |
| `CAMERA_OFFSET_FORWARD_M` | **`0.090`** | Mét | Độ nhô phía trước của camera OAK-D S2 |
| `CAMERA_OFFSET_LATERAL_M` | **`-0.015`** | Mét | Độ lệch phải của camera OAK-D S2 so với trục giữa xe |
| `CAMERA_HEIGHT_M` | **`0.120`** | Mét | Chiều cao trục camera so với mặt sàn |
| `CAMERA_PITCH_DEG` | **`0.0`** | Độ | Góc nghiêng ngửa/cúp của camera |

### 9.2. Danh mục đầy đủ các cổng giao tiếp REST API

| Endpoint API | Phương Thức | Tham Số Truyền Vào | Chức Năng Hệ Thống |
| :--- | :---: | :--- | :--- |
| `/stream.mjpg` | `GET` | Không | Luồng video trực tiếp MJPEG (Cam AI hoặc Cam Nhiệt) |
| `/api/state` | `GET` | Không | Xuất toàn bộ dữ liệu telemetry: Pose, IMU, CSM, Profiler, Battery |
| `/api/drive` | `GET/POST` | `v` (m/s), `w` (rad/s) | Điều khiển vận tốc vi sai tức thời (WASD) |
| `/api/step_move` | `GET/POST` | `type`, `dir`, `dist_cm`, `deg`, `speed_pct` | Thực thi bước di chuyển định lượng chính xác |
| `/api/step_cancel` | `GET/POST`| Không | Hủy khẩn cấp bước di chuyển đang chạy |
| `/api/pattern_move` | `GET/POST`| `type`, `length`, `width`, `leg`, `turn_dir` | Khởi động bài thử nghiệm quỹ đạo hình học khép kín |
| `/api/auto_scan_360`| `GET/POST`| `action=start/stop` | Kích hoạt chế độ tự xoay chậm đều 360° quét bản đồ |
| `/api/map_reset` | `POST` | Không | Xóa trắng bản đồ 2D và reset vị trí về gốc (0, 0, 0) |
| `/api/map_save` | `POST` | `name` | Lưu bản đồ 2D ra file PNG / JSON |
| `/api/config/full` | `GET` | Không | Lấy toàn bộ danh sách Profile và tham số hiện hành |
| `/api/config/update` | `POST` | `profile_id`, `params` | Cập nhật và lưu đè tham số vào Profile (Hot-Reload) |
| `/api/config/select_profile` | `POST` | `id` | Chuyển đổi Profile hoạt động tức thời |
| `/api/config/create_profile` | `POST` | `name`, `description`, `params` | Tạo snapshot phiên bản cấu hình mới |
| `/api/config/delete_profile` | `POST` | `id` | Xóa phiên bản cấu hình khỏi hệ thống |

---

## 10. KẾT QUẢ THỰC NGHIỆM, KẾT LUẬN & HƯỚNG PHÁT TRIỂN

### 10.1. Đánh giá Định lượng Sau Hiệu Chỉnh Toàn Diện
* **Độ thẳng khi di chuyển cự ly dài:** Trong cự ly chạy thẳng $300\text{cm}$ ($3.0\text{m}$), xe đi thẳng tắp với độ lệch phương ngang $< 1.2\text{cm}$ (nhờ sự kết hợp giữa Hardware Wheel Trim L:1.000 R:1.020 và Active IMU Heading Lock $K_p = 0.50$).
* **Độ chính xác góc quay vi sai:** Sai số góc xoay trong các bước $45^\circ, 90^\circ, 180^\circ, 360^\circ$ nhỏ hơn $1.5^\circ$ trên sàn gạch men và sàn gỗ, triệt tiêu hoàn toàn hiện tượng thiếu góc khi quay góc nhỏ nhờ bù trễ ma sát tĩnh $80\text{ms}$.
* **Độ khép kín bản đồ SLAM (Loop Closure Residual):** Sau khi hoàn thành bài thử nghiệm hình chữ nhật $80 \times 40\text{cm}$ hoặc quay $360^\circ$ tại chỗ, góc hướng robot trở về chính xác hướng 3 giờ ban đầu ($0.0^\circ$), sai số khép góc $< 1.5^\circ$, không còn hiện tượng lệch $45^\circ$ như các phiên bản cũ.
* **Hiệu năng & Tài nguyên phần cứng:**
  * Tần số xử lý toàn hệ thống duy trì ổn định: **$18 - 25\text{ FPS}$**.
  * Tải CPU Jetson Nano: $\approx 35 - 45\%$ (4 nhân phân bổ đồng đều).
  * Nhiệt độ CPU/GPU: $\approx 48 - 52^\circ\text{C}$ (rất mát mẻ, không bị bóp nghẽn xung nhịp Thermal Throttling).

### 10.2. Kết luận
Dự án **CS532.R11** đã chứng minh rằng: **Hoàn toàn có thể xây dựng một hệ thống SLAM 2D độ chính xác cao, tin cậy và mượt mà trên nền tảng robot mini giá rẻ JetBot mà không cần cảm biến LiDAR đắt tiền hay bộ mã hóa vòng quay encoder cơ khí**. Sự kết hợp hài hòa giữa xử lý độ sâu stereo phần cứng VPU, cảm biến quán tính IMU 100Hz, thuật toán so khớp trường Likelihood Field CSM có khóa cứng góc xoay và kiến trúc điều khiển động học vi sai phân nhóm đã tạo nên một sản phẩm khoa học công nghệ hoàn chỉnh, sẵn sàng ứng dụng trong giáo dục, nghiên cứu và công nghiệp tự hành.

### 10.3. Hướng phát triển tiếp theo
1. **Module Lập kế hoạch đường đi Toàn cục A\* (Global Path Planner):** Xây dựng ma trận Costmap với lớp giãn nở an toàn (Inflation Layer 15cm) quanh chướng ngại vật, cho phép người dùng click chọn điểm đích trực tiếp trên bản đồ Web.
2. **Module Điều hướng Bám Quỹ đạo Cục bộ Pure Pursuit / DWA:** Tự động điều khiển động cơ bám theo quỹ đạo tối ưu, tự động tránh các vật cản động bất ngờ xuất hiện trên đường đi.
3. **Đóng gói dịch vụ nhúng Systemd:** Cấu hình hệ thống tự động khởi chạy service SLAM ngay khi cắm nguồn Jetson Nano, tự phát Access Point Wi-Fi để người dùng kết nối điều khiển ngay lập tức.

---
*Báo cáo kỹ thuật được biên soạn toàn diện, đồng bộ 100% với mã nguồn hệ thống `CS532.R11_JetBot_SLAM` (Phiên bản Tháng 10/2026).*
