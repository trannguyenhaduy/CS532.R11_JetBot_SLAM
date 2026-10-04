# 📖 HƯỚNG DẪN LẤY CODE VÀ KHỞI CHẠY WEB COCKPIT TRÊN JETBOT

Tài liệu này dành cho các thành viên trong nhóm (đặc biệt là bạn đang giữ robot JetBot) để lấy code từ GitHub về và khởi chạy **Trạm điều khiển Web 3D & Giám sát pin (Web Cockpit)**.

---

## 📌 BƯỚC 1: Lấy địa chỉ IP của JetBot

1. Bật nguồn JetBot và đảm bảo JetBot đã kết nối vào mạng Wi-Fi (ví dụ Wi-Fi nhà bạn hoặc phát từ điện thoại).
2. Xem địa chỉ IP:
   - Cách 1: Nhìn trực tiếp trên **màn hình OLED nhỏ** gắn trên xe JetBot.
   - Cách 2: Mở terminal trên JetBot gõ:
     ```bash
     hostname -I
     ```
   *(Ví dụ IP nhận được: `192.168.1.150`)*

---

## 📌 BƯỚC 2: Tải code từ GitHub về JetBot

Mở terminal trên JetBot và chạy các lệnh sau:

### Trường hợp 1: Lần đầu tiên tải code về JetBot
```bash
# 1. Đi vào thư mục mã nguồn ROS
cd ~/catkin_ws/src

# 2. Clone repo của nhóm về
git clone https://github.com/trannguyenhaduy/CS532.R11_JetBot_SLAM.git jetbot_slam

# 3. Đi vào thư mục vừa tải và cấp quyền chạy cho các file Python
cd ~/catkin_ws/src/jetbot_slam
chmod +x *.py
```

### Trường hợp 2: Đã tải trước đó, giờ cập nhật code mới nhất
```bash
cd ~/catkin_ws/src/jetbot_slam
git pull
chmod +x *.py
```

---

## 📌 BƯỚC 3: Cài đặt thư viện phụ trợ (Chỉ cần chạy 1 lần nếu thiếu)

Đảm bảo JetBot có đủ các thư viện cơ bản để chạy web và đọc pin INA219:
```bash
pip3 install opencv-python numpy smbus2
```

---

## 📌 BƯỚC 4: Khởi chạy Web Dashboard

Bạn có **2 lựa chọn** để chạy:

### Lựa chọn A: Chạy RIÊNG Web Dashboard (Khuyên dùng để test trước)
Chạy cách này để kiểm tra ngay giao diện web, kiểm tra đồng hồ đo pin và thử phím lái xe:
```bash
cd ~/catkin_ws/src/jetbot_slam
python3 slam_web_dashboard.py
```
*Khi terminal hiện dòng thông báo:*  
`🌐 JetBot 3D Cockpit Web Server: http://0.0.0.0:8080` $\rightarrow$ Là web đã khởi động thành công!

---

### Lựa chọn B: Khởi chạy TOÀN BỘ Hệ thống Đồ án (Full Launch)
Chạy cách này khi đã sẵn sàng kết nối camera OAK-D S2, thuật toán SLAM và Động cơ:
```bash
roslaunch jetbot_slam master_system.launch
```
*(Lệnh này sẽ tự động bật toàn bộ node của 3 thành viên kèm theo Web Dashboard)*

---

## 📌 BƯỚC 5: Mở Web trên Máy tính hoặc Điện thoại

1. Kết nối Laptop hoặc Điện thoại của bạn vào **cùng mạng Wi-Fi** với JetBot.
2. Mở trình duyệt (Google Chrome, Microsoft Edge, Safari) và gõ vào thanh địa chỉ:
   ```text
   http://<IP_CỦA_JETBOT>:8080
   ```
   *(Ví dụ: `http://192.168.1.150:8080`)*

---

## 🎮 TRÊN GIAO DIỆN WEB BẠN CÓ THỂ LÀM GÌ?

- **Camera OAK-D S2:** Xem video trực tiếp 15 FPS có khung nhận diện AI (Người, Ghế...).
- **Không gian 3D Holographic:** Dùng chuột xoay $360^\circ$, phóng to/thu nhỏ xem mây điểm 3D phòng thật.
- **Bản đồ 2D Grid:** Bấm nút `2D GRID` để xem bản đồ phẳng Occupancy Grid, vệt bánh xe di chuyển và góc quay robot.
- **Điều khiển xe từ xa:** Bấm các phím `W`, `A`, `S`, `D` trên bàn phím laptop để lái robot chạy trong phòng.
- **Đồng hồ Pin 3S Li-ion Real-Time:** Hiển thị trực tiếp điện áp Vôn, % pin, dòng xả Ampe, công suất Watt và thời lượng còn lại (khi lái xe dòng xả sẽ tăng vọt theo thời gian thực!).
- **Hệ thống điểm chuẩn (Benchmark):** Xem thang điểm 100 tự động đánh giá sức khỏe robot.

---

## ❓ CÁC LỖI THƯỜNG GẶP & CÁCH XỬ LÝ (FAQS)

1. **Gõ IP trên trình duyệt nhưng báo "Không thể truy cập trang web"?**
   - Kiểm tra xem Laptop và JetBot đã bắt chung 1 mạng Wi-Fi chưa.
   - Thử mở terminal trên laptop gõ: `ping <IP_CỦA_JETBOT>` xem có thông mạng không.
2. **Báo lỗi `Address already in use` (Cổng 8080 đang bị chiếm)?**
   - Do có một tiến trình web cũ chưa tắt. Chạy lệnh sau trên JetBot để giải phóng cổng:
     ```bash
     sudo fuser -k 8080/tcp
     ```
3. **Nếu muốn test trên laptop của mình (không có robot) thì sao?**
   - Chỉ cần mở terminal trong thư mục code trên máy tính của bạn gõ:
     ```bash
     python slam_web_dashboard.py
     ```
   - Mở trình duyệt gõ `http://localhost:8080`. Hệ thống sẽ tự động kích hoạt **Chế độ Giả lập 3D Digital Twin** để bạn test giao diện!
