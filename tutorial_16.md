# ROS — Hệ điều hành cho Robot

> **ROS (Robot Operating System)** không phải là một hệ điều hành như Windows hay Linux. Nó là một **bộ khung phần mềm (framework)** giúp các phần khác nhau của một con robot — camera, cảm biến, động cơ, thuật toán — nói chuyện với nhau một cách gọn gàng, thay vì phải tự viết lại mọi thứ từ đầu cho mỗi robot.

## Sau bài này bạn sẽ

- Hiểu ROS giải quyết vấn đề gì và vì sao gần như mọi robot nghiên cứu đều dùng nó.
- Nắm 4 khái niệm cốt lõi: **node, topic, message, service**.
- Phân biệt **ROS 1** và **ROS 2**, biết nên học cái nào.
- Biết ROS liên quan thế nào tới các robot trong lab (PuppyPi, TonyPi, JetBot).

## 1. Vấn đề ROS giải quyết

Một con robot đơn giản đã cần rất nhiều mẩu phần mềm chạy **cùng lúc**: đọc camera, quét LiDAR, tính đường đi, điều khiển động cơ, nhận lệnh từ xa... Nếu viết tất cả thành một chương trình khổng lồ thì:

- Sửa một chỗ dễ làm hỏng chỗ khác.
- Không tái dùng được cho robot khác.
- Khó chia cho nhiều người cùng làm.

ROS đề xuất cách chia nhỏ: **mỗi việc là một chương trình độc lập (node)**, và ROS lo phần "đường ống" để các node trao đổi dữ liệu.

![Đồ thị node và topic trong ROS](/itcourses/api/uploads/b2468685b5ae_ros_graph.svg)

## 2. Bốn khái niệm cốt lõi

### 2.1. Node (nút)
Một **node** là một chương trình nhỏ làm **một việc**: ví dụ node đọc camera, node điều khiển motor, node lập kế hoạch đường đi. Một robot chạy hàng chục node cùng lúc.

### 2.2. Topic (chủ đề) — kiểu publish/subscribe
Các node trao đổi dữ liệu qua **topic**. Một node **publish** (đăng) dữ liệu lên topic; node khác **subscribe** (đăng ký nghe) topic đó.

- Ví dụ: node camera publish ảnh lên topic `/image`; node nhận diện vật subscribe `/image`.
- Điểm hay: hai node **không cần biết nhau**. Ta có thể thay node camera bằng node khác miễn nó vẫn publish đúng `/image` — giống như cắm/rút thiết bị mà phần còn lại không đổi.

### 2.3. Message (bản tin)
Dữ liệu chảy trên topic có **kiểu định sẵn** gọi là message: ví dụ `Image` (ảnh), `LaserScan` (dữ liệu LiDAR), `Twist` (lệnh vận tốc: đi thẳng bao nhiêu, quay bao nhiêu). Nhờ chuẩn hoá, các gói phần mềm của cộng đồng ghép được với nhau.

### 2.4. Service (dịch vụ) và Action
Topic hợp cho luồng dữ liệu liên tục. Khi cần **hỏi–đáp một lần** (ví dụ "chụp giúp một tấm ảnh") thì dùng **service** (gọi và chờ trả lời). Với việc **kéo dài có tiến độ** (ví dụ "đi tới điểm A") thì dùng **action** (có phản hồi tiến độ và huỷ được).

> **Một hình dung đơn giản:** topic giống như đài phát thanh (phát cho ai nghe cũng được); service giống như gọi điện hỏi một câu rồi nghe trả lời.

## 3. Vì sao ROS phổ biến đến vậy

- **Kho gói khổng lồ:** SLAM, điều hướng (Nav2), nhận diện, mô phỏng (Gazebo), hiển thị (RViz)... đa số đã có sẵn, chỉ việc lắp.
- **Công cụ gỡ lỗi mạnh:** `ros2 topic echo` xem dữ liệu đang chảy, `rqt_graph` vẽ sơ đồ node, RViz dựng hình 3D những gì robot "thấy".
- **Chạy phân tán:** các node có thể nằm trên nhiều máy (ví dụ robot + laptop) mà vẫn nói chuyện được.
- **Chuẩn chung:** kỹ năng ROS dùng lại được trên hầu hết robot nghiên cứu và nhiều robot công nghiệp.

## 4. ROS 1 hay ROS 2?

| | ROS 1 | ROS 2 |
|---|---|---|
| Trạng thái | Cũ, bản cuối (Noetic) hết hỗ trợ 2025 | **Hiện hành**, đang phát triển |
| Giao tiếp | có `roscore` trung tâm | phi tập trung (DDS), không cần master |
| Thời gian thực / nhiều robot | hạn chế | tốt hơn, thiết kế cho sản phẩm thật |
| Nên học | chỉ khi dự án cũ bắt buộc | **Nên bắt đầu bằng ROS 2** |

> **Lời khuyên:** Người mới nên học thẳng **ROS 2** (bản Humble/Jazzy). PuppyPi và TonyPi thế hệ mới của Hiwonder đều chạy ROS 2.

## 5. Một vòng đời điển hình

1. Viết vài node (hoặc lấy node có sẵn) cho từng nhiệm vụ.
2. Quy ước các topic/message để chúng khớp nhau.
3. Chạy tất cả bằng một **launch file** (một lệnh bật cả hệ thống).
4. Dùng `rqt_graph` / RViz để kiểm tra dữ liệu chảy đúng.
5. Thử trong mô phỏng (Gazebo) trước, rồi mới nạp lên robot thật.

## 6. ROS trong lab của chúng ta

- **PuppyPi & TonyPi** (Hiwonder, Raspberry Pi 5) chạy **ROS 2**: các khối cảm nhận – quyết định – sinh dáng đi – chấp hành trong bài về hai robot này chính là các node ROS trao đổi qua topic.
- **SLAM và điều hướng** trên robot lab dựa trên các gói ROS như `slam_toolbox` và `Nav2`.
- **JetBot** bản gốc dùng thư viện Python riêng của NVIDIA, nhưng cũng có các bản cộng đồng chuyển sang ROS để ghép vào hệ điều hướng chuẩn.

> **Gợi ý thực hành đầu tiên:** cài ROS 2, chạy đúng hai node mẫu `talker` và `listener`, rồi mở `ros2 topic list` và `ros2 topic echo` để "nhìn thấy" bản tin chạy qua. Hiểu được cảnh này là đã nắm 80% trực giác về ROS.

## Tự kiểm tra

1. Node và topic khác nhau thế nào? Cho một ví dụ mỗi loại trên robot.
2. Vì sao cơ chế publish/subscribe giúp thay thế linh kiện phần mềm dễ hơn?
3. Khi nào nên dùng service thay vì topic?
4. Bạn sẽ chọn ROS 1 hay ROS 2 cho một đồ án mới? Vì sao?
5. Kể tên hai công cụ ROS giúp gỡ lỗi và cho biết mỗi cái làm gì.

---
*Bài viết thuộc mục Tài liệu — Phần cứng & IoT của ITCourses. Xem thêm: "PuppyPi", "TonyPi", "SLAM".*
