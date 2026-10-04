# SLAM — Định vị và Dựng bản đồ đồng thời

> **SLAM = Simultaneous Localization And Mapping.** Cho một con robot vào một căn phòng nó chưa từng thấy, không có GPS, không có bản đồ sẵn — SLAM là bộ kỹ thuật giúp nó vừa **vẽ ra bản đồ căn phòng**, vừa **biết mình đang đứng ở đâu trên chính bản đồ đó**, cùng một lúc.

## 1. Tại sao SLAM khó? Bài toán "con gà và quả trứng"

Hãy thử tưởng tượng bạn bị bịt mắt thả vào một toà nhà lạ:

- Muốn biết **mình đang ở đâu**, bạn cần một tấm **bản đồ** để đối chiếu.
- Nhưng muốn **vẽ bản đồ** cho chính xác, bạn lại cần biết **mình đang đứng ở đâu** khi ghi lại mỗi bức tường.

Hai việc phụ thuộc lẫn nhau — đó chính là cái khó cốt lõi. SLAM giải quyết bằng cách **làm cả hai đồng thời và liên tục sửa cho nhau**: mỗi khi robot di chuyển, nó đoán vị trí mới, ghi thêm vào bản đồ, rồi dùng bản đồ vừa cập nhật để chỉnh lại chính vị trí của mình.

![Vòng lặp SLAM](/itcourses/api/uploads/3c156da68858_slam_loop.svg)

## 2. Vòng lặp SLAM chạy như thế nào

| Bước | Việc làm | Ví dụ cụ thể |
|------|----------|--------------|
| 1. Thu cảm biến | Đọc LiDAR / camera / IMU / encoder bánh xe | LiDAR quét 360° khoảng cách tới tường |
| 2. Trích đặc trưng | Tìm các "điểm mốc" ổn định, dễ nhận lại | góc tường, cạnh bàn, điểm ảnh đặc biệt |
| 3. Ước lượng chuyển động | So khớp khung hiện tại với khung trước để suy ra robot đã đi bao xa, quay bao nhiêu | "so với 0,1s trước, tôi tiến 3cm, quay 2°" |
| 4. Cập nhật bản đồ | Đưa các điểm mốc mới vào bản đồ theo vị trí vừa ước lượng | thêm một đoạn tường mới vào map |
| 5. Khép vòng (loop closure) | Nhận ra "chỗ này mình đã đi qua rồi" và sửa lại sai lệch tích luỹ trên cả đường đi | về lại cửa phòng, kéo bản đồ khớp lại |

> **Điểm mấu chốt — loop closure:** Mỗi bước ước lượng chuyển động đều có sai số nhỏ. Đi càng lâu, sai số **cộng dồn** (drift) làm bản đồ bị méo, cong. Khi robot quay lại một nơi từng thấy và **tự nhận ra điều đó**, nó có một ràng buộc mạnh để "kéo" toàn bộ bản đồ về đúng — giống như khép kín một vòng dây thừng. Đây là thứ phân biệt một hệ SLAM tốt với một hệ chỉ cộng dồn odometry.

![Drift và cách loop closure sửa sai số](/itcourses/api/uploads/656bb2f7bbec_slam_drift.svg)

## 3. Các loại SLAM theo cảm biến

- **LiDAR SLAM** — dùng cảm biến laser đo khoảng cách. Chính xác về hình học, ổn định trong tối, nhưng cảm biến đắt. *Thuật toán tiêu biểu: Cartographer, Hector SLAM, GMapping (2D); LOAM (3D).*
- **Visual SLAM (vSLAM)** — chỉ dùng camera. Rẻ, giàu thông tin (màu, vân), nhưng nhạy với ánh sáng và vùng trơn không đặc trưng. *Tiêu biểu: ORB-SLAM3, LSD-SLAM.*
- **RGB-D SLAM** — camera có kèm chiều sâu (Realsense, Kinect, OAK-D). Cho luôn khoảng cách từng điểm ảnh, rất hợp trong nhà. *Tiêu biểu: RTAB-Map.*
- **Visual-Inertial SLAM (VIO/VI-SLAM)** — camera + IMU (con quay hồi chuyển + gia tốc kế). IMU lấp khoảng trống khi camera "mù" (chuyển động nhanh, mờ). *Tiêu biểu: VINS-Fusion, ORB-SLAM3 (chế độ VI).*

> **Trong thực tế robot hay ghép nhiều cảm biến:** LiDAR + IMU + odometry bánh xe, hoặc camera + IMU. Càng nhiều nguồn độc lập thì càng ít bị "lừa" khi một cảm biến sai.

## 4. Bản đồ SLAM có dạng gì?

- **Occupancy grid (lưới chiếm dụng)** — chia sàn thành các ô nhỏ, mỗi ô đánh dấu *trống / có vật cản / chưa biết*. Đây là dạng phổ biến nhất cho robot di chuyển trong nhà và là đầu vào cho thuật toán tìm đường.
- **Feature/landmark map** — chỉ lưu toạ độ các điểm mốc, nhẹ, hợp với vSLAM.
- **Point cloud / mesh** — đám mây điểm 3D dày đặc, dùng cho tái dựng 3D và robot trong không gian phức tạp.

## 5. SLAM trong hệ sinh thái robot của lab

SLAM là nền tảng cho **điều hướng tự động (autonomous navigation)**: có bản đồ + biết vị trí → mới lập được đường đi tránh vật cản tới đích. Trong bộ robot đã giới thiệu ở mục Tài liệu:

- **flatbot / JetBot** dùng LiDAR hoặc camera OAK-D để dựng occupancy grid rồi tự đi trong phòng lab.
- **PuppyPi / TonyPi** (Hiwonder, RPi5, ROS2) có sẵn hệ SLAM + navigation dựa trên LiDAR để đi theo bản đồ.
- Bản đồ do SLAM tạo ra chính là thứ được nạp vào bước **path planning** (A*, Dijkstra) và **tránh vật cản** động.

> **Gợi ý thực hành:** Trên robot chạy ROS2, gói `slam_toolbox` (LiDAR 2D) hoặc `rtabmap_ros` (RGB-D) cho phép chạy SLAM chỉ với vài câu lệnh, xem bản đồ dựng dần theo thời gian thực trong RViz. Hãy lái robot đi **chậm** và **khép kín một vòng** quanh phòng để loop closure kích hoạt — bạn sẽ thấy bản đồ "giật" một cái rồi thẳng thớm lại.

## 6. Tự kiểm tra

1. Vì sao SLAM được gọi là bài toán "con gà và quả trứng"?
2. Drift (trôi sai số) là gì, và loop closure sửa nó bằng cách nào?
3. Cho một robot đi trong phòng tối không đèn, giữa LiDAR SLAM và Visual SLAM bạn chọn loại nào? Vì sao?
4. Occupancy grid khác feature map ở điểm nào, và vì sao occupancy grid hợp cho việc tìm đường?
5. IMU giúp gì cho Visual SLAM khi camera bị mờ do robot quay nhanh?

---
*Bài viết thuộc mục Tài liệu — Phần cứng & IoT của ITCourses.*
