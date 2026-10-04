# Lái tự động bằng camera trên robot và camera trần

> Robot lab tự đi trong phòng nhờ **hai con mắt khác nhau**: một **camera trần** (PTZ gắn tường, nhìn từ trên xuống) cho biết robot **đang ở đâu trong phòng**, và **camera trên chính robot** (nhìn phía trước) cho biết **ngay trước mặt có gì**. Bài này giải thích cách hai nguồn này phối hợp để robot điều hướng tự động — mô tả theo đúng **hệ robot fleet của lab**.

## Sau bài này bạn sẽ

- Hiểu vì sao cần **cả hai** camera và mỗi cái đảm nhiệm gì.
- Biết camera trần định vị robot thế nào (phát hiện, tỉ lệ pixel↔mét, hướng).
- Biết camera trên robot làm gì (bám line, tránh va chạm, đo chiều sâu).
- Hiểu cách **kết hợp** hai nguồn và vai trò của **SLAM**.
- Nắm những **cái bẫy** kinh điển khi triển khai thật.

# 1. Vì sao dùng hai camera?

Một camera duy nhất khó vừa biết *"tôi đang đứng chỗ nào trong phòng"* vừa thấy *"trước mũi có vật cản gì"*. Nên hệ dùng hai góc nhìn bổ sung nhau:

- **Camera trần (nhìn từ trên):** thấy cả căn phòng và robot như một chấm trên bản đồ → cho **vị trí tuyệt đối** (toàn cục). Không bị trôi sai số khi còn nhìn thấy robot.
- **Camera trên robot (nhìn trước):** thấy chi tiết ngay trước mặt (vạch line, vật cản, khoảng trống) → cho **cảm nhận cục bộ** mà camera trần bị khuất.

![Kiến trúc lái tự động hai camera](/itcourses/api/uploads/0d1490eb0b49_autodrive_arch.svg)

# 2. Camera trần (PTZ) — định vị toàn cục

Camera PTZ gắn tường (điều khiển kiểu Hikvision ISAPI) nhìn bao quát sàn phòng. Chuỗi xử lý:

1. **Phát hiện robot:** mô hình **YOLO phân đoạn (segmentation)** với **2 lớp: `robot` và `dog_robot`** (không dùng bản một lớp). Bản nâng cao dùng dịch vụ **panoptic (EoMT)** cho hàm `detect()`, có YOLO làm phương án dự phòng; một mô hình *oversegment* riêng phục vụ lớp phủ trực quan trên luồng video.
2. **Đổi pixel ra mét:** dùng hằng số **OVERHEAD_PX_PER_M = 201.6** (đo từ **2 mốc trên sàn**, cho khung ảnh trần **960×540**). Vị trí robot = **tâm hộp phát hiện × tỉ lệ px/m** → toạ độ thật trong phòng (mét).
3. **Xác định hướng (heading):** có **ba nguồn** — (a) mô hình **pose** (điểm mốc trước/sau của robot), (b) **"hướng theo chuyển động"** (so vị trí giữa hai cú tiến thẳng ngắn), (c) **IMU** trên robot. flatbot dùng chủ yếu (b) + (c).

> **Lưu ý:** vị trí "home" của PTZ (góc quay/zoom) **bị trôi** theo thời gian — luôn kiểm tra lại preset trước khi thu dữ liệu hay định vị.

# 3. Camera trên robot — cảm nhận cục bộ

- **JetBot:** dùng camera trước cho **bám line/bám đường** và **tránh va chạm** — phản ứng cục bộ theo hình ảnh (xem bài "JetBot — Kết nối, điều khiển và phát triển").
- **flatbot:** gắn **camera chiều sâu OAK-D**. Quy trình `depth_map_drive`: lấy mặt sàn bằng **RANSAC** → suy ra **quạt không gian trống trước mặt** → **hợp vào khung toạ độ của camera trần** theo hướng đã đo, tạo **hàng rào ảo (geofence)** vùng đi được.

> **Bẫy kỹ thuật:** phải liên tục gọi **`/depth_keepalive`**, nếu không pipeline OAK-D bị "kẹt" (wedge).

**Bổ sung cho nhau:** camera trần cho biết **ở đâu** (tuyệt đối); camera robot cho biết **trước mặt có gì** — thứ mà góc nhìn từ trên bị che.

# 4. Lập đường đi: chính sách RL (không phải A*)

Điểm đáng chú ý: việc "đi tới đích" (**goto**) dùng **chính sách học tăng cường (RL)** cho **mọi robot** — **không dùng A\*** hay Dijkstra. Mỗi lần gọi sẽ sinh một tiến trình điều hướng.

- **flatbot** có bộ điều khiển riêng: **quay tại chỗ tới đúng phương** → **đi thẳng giữ hướng bằng IMU**, với hướng lấy từ **dịch chuyển thật thấy trên camera trần**.

![Hai tỉ lệ pixel/mét — cái bẫy quan trọng nhất](/itcourses/api/uploads/6104f550969d_autodrive_scale.svg)

> ### ⚠️ Cái bẫy quan trọng nhất: RL_PIX_PER_M = 90
> Có **hai** tỉ lệ pixel/mét khác nhau: **201.6** dùng để **hiển thị và định vị** (ảnh 960×540), nhưng **90** (`RL_PIX_PER_M`) là tỉ lệ mà **chính sách RL được huấn luyện**. Quan sát đưa vào policy **phải dùng 90**. Nếu lỡ nạp 201.6 vào policy, robot **không tới đích mà cứ lượn vòng quanh nó**. Đây là lỗi hay gặp và khó đoán nhất.

# 5. Kết hợp hai nguồn (fusion)

- Camera trần là **mốc tuyệt đối, không trôi** khi còn nhìn thấy robot; cảm nhận cục bộ (ground/LiDAR) **đưa robot qua những điểm mù** của camera trần.
- **flatbot còn chạy SLAM 2D** (thuần numpy, không ROS): **LiDAR LD19P** + **IMU (MPU-6050 trên một Pico RP2040)** + **mốc từ camera trần**. Dùng **khớp quét tương quan (csm)** dựng **lưới chiếm dụng log-odds**; phát hiện từ camera trần **căn cứng (rigid-align)** hệ SLAM về hệ phòng và **sửa trôi** cho pose SLAM.
- Thêm: `map_register` **"ROAM-Fuse"** — căn khớp bền với xoay (Fourier–Mellin + tìm kiếm mồi bằng IMU), hợp nhất có kiểm soát độ tin cậy; **khép vòng (loop-closure)** là bước nâng độ chính xác tiếp theo.

# 6. Những cái bẫy đáng học

- **RL_PIX_PER_M = 90** cho policy (khác 201.6 để hiển thị) — nạp nhầm → robot lượn vòng. *(Quan trọng nhất.)*
- **PTZ trôi home:** kiểm tra preset trước khi định vị/thu dữ liệu.
- **Đo đường (odometry):** bánh xe **6.5 cm**; chuẩn khoảng cách bằng **2 mốc sàn** trên ảnh trần.
- **Giữ hướng khi đi thẳng:** bộ điều khiển **P+D phía server**, mỗi bot có **dấu lái riêng** (flatbot = −1).
- **Khoảng cách sim→real:** bộ RL đi thẳng có sai số hướng thật **~5.8×** so với mô phỏng; thử "nới rộng mô phỏng" một cách mù quáng đã **làm tệ hơn và phải hoàn tác** — bài học cảnh giác.
- **flatbot — dấu IMU:** hướng IMU **thuận chiều kim đồng hồ**, còn toán SLAM **ngược chiều** → phải **đảo dấu** delta IMU, nếu không bản đồ bị "toè" thành hình sao.
- **Nguồn/USB flatbot:** dây USB + pin 3S chập chờn (sụt áp + USB tự cắm lại khi tải nặng).

# 7. Các đầu mối cụ thể (hệ fleet)

- **Máy chủ fleet:** FastAPI tại `192.168.20.150:8080` (service `uit-robotic`).
- **API tiêu biểu:** `GET /api/robots`; `POST /api/goto/start`; `POST /api/automap/start`, `/api/slam/start|stop`, `GET /api/slam/status`, `/api/slam/map.png`; `GET /api/detect/status`; `GET /api/cameras/room/snapshot` (960×540); mỗi robot: `/api/robots/{id}/lidar | /imu | /drive_twist | /snapshot | /stream`.
- **Trạm sạc:** cọc ArUco `DICT_4X4_50`; preset PTZ dock az600 / el310 / zoom77.
- **flatbot = bot13**, Jetson Orin Nano Super; motor+encoder+IMU+cảm biến pin INA226 gắn trên Pico; pin 3S Li-ion.
- **LiDAR:** LDROBOT **LD19P** (~9.9 Hz, tầm 0.25–3.6 m).

## Tự kiểm tra

1. Camera trần và camera trên robot mỗi cái trả lời câu hỏi gì? Vì sao cần cả hai?
2. Vị trí robot (mét) được tính thế nào từ ảnh trần?
3. Vì sao dùng `RL_PIX_PER_M = 90` cho policy trong khi hiển thị dùng 201.6? Nạp nhầm thì hiện tượng gì?
4. Camera trần giúp SLAM của flatbot ra sao?
5. Kể hai cái bẫy phần cứng/điều khiển và cách xử lý.

---
*Bài viết mô tả hệ robot fleet của lab (nguồn: nhóm nanobot). Một số con số (preset PTZ, tỉ lệ px/m) gắn với cấu hình cụ thể và có thể thay đổi. Xem thêm: "SLAM", "ROS", "Nhập môn LiDAR", "JetBot — Kết nối, điều khiển và phát triển".*
