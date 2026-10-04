# JetBot — Kết nối, điều khiển và phát triển

> Bài này hướng dẫn **thực hành**: từ lúc bật JetBot lên, đưa nó vào mạng WiFi, mở trình duyệt để điều khiển, viết những dòng Python đầu tiên cho robot chạy, cho tới quy trình **phát triển một mô hình AI** hoàn chỉnh. Nếu chưa biết JetBot là gì, hãy đọc bài "JetBot — Tổng quan & bắt đầu" trước.

## Sau bài này bạn sẽ

- Đưa JetBot vào WiFi và tìm được địa chỉ IP của nó.
- Mở **JupyterLab** trên trình duyệt và điều khiển robot mà không cần cài gì thêm.
- Viết Python cơ bản để chạy động cơ và xem camera.
- Nắm **quy trình phát triển AI**: thu thập dữ liệu → huấn luyện → triển khai → chạy.
- Biết các cách kết nối khác (SSH, HTTP/WebSocket) và cách gỡ lỗi thường gặp.

# 1. Chuẩn bị: nạp thẻ nhớ và khởi động

1. Tải **ảnh hệ điều hành JetBot** (image dành cho Jetson Nano) từ trang JetBot/NVIDIA, ghi ra thẻ microSD bằng **balenaEtcher** hoặc **Raspberry Pi Imager**.
2. Gắn thẻ vào Jetson Nano, lắp lên khung JetBot, **sạc đầy pin**.
3. Bật nguồn. Chờ khoảng 1–2 phút để hệ khởi động; nếu JetBot có **màn OLED nhỏ**, nó sẽ hiển thị địa chỉ IP, mức pin và tài nguyên.

> **Lưu ý về nguồn:** động cơ lấy điện từ pin trên JetBot. Pin yếu là robot **không chạy hoặc chạy lệch** dù code đúng — luôn sạc đầy trước khi thử.

# 2. Kết nối JetBot vào mạng WiFi

![Kết nối JetBot qua WiFi và trình duyệt](/itcourses/api/uploads/c3a501e407d9_jetbot_connect.svg)

- **Lần đầu (chưa có WiFi):** cắm màn hình HDMI + bàn phím vào Jetson Nano để đăng nhập và khai báo WiFi; **hoặc** cắm cáp **USB** từ JetBot sang máy tính rồi mở `http://192.168.55.1:8888` (Jetson tạo cổng mạng USB sẵn).
- **Nối WiFi bằng dòng lệnh (jetbot-os):** khi đã vào được terminal (qua HDMI + bàn phím), dùng `nmcli` để nối WiFi ngay:

```bash
sudo nmcli device wifi connect <SSID> password <PASSWORD>

# hoặc để nmcli HỎI mật khẩu tương tác (an toàn hơn — mật khẩu KHÔNG lưu vào lịch sử lệnh):
sudo nmcli --ask device wifi connect <SSID>
```

  Thay `<SSID>` và `<PASSWORD>` bằng tên mạng và mật khẩu WiFi của bạn (SSID có dấu cách thì đặt trong nháy kép). Kiểu **`--ask`** sẽ hiện dấu nhắc để bạn gõ mật khẩu (không hiện trên màn hình, không lộ trong `history`). Kiểm tra đã nối: `nmcli device status` hoặc `nmcli -t -f active,ssid dev wifi`; kết nối này được **lưu và tự nối lại** sau khi khởi động lại.
- Sau khi vào được WiFi, xem **địa chỉ IP** trên màn OLED (hoặc bằng lệnh `ifconfig`/`ip addr`).
- Từ giờ, **máy tính và JetBot phải ở cùng một mạng** thì mới điều khiển qua trình duyệt được.

# 3. Điều khiển qua trình duyệt: JupyterLab

Mở trình duyệt trên máy tính và truy cập:

```
http://<IP-của-JetBot>:8888
```

Mật khẩu mặc định thường là `jetbot`. Bạn sẽ vào **JupyterLab** — một môi trường lập trình chạy ngay trên robot, không cần cài gì trên máy tính. Trong đó có sẵn các **notebook mẫu**: `basic_motion`, `teleoperation`, `collision_avoidance`, `road_following`...

# 4. Điều khiển cơ bản bằng Python (lớp `Robot`)

Mở một notebook và chạy:

```python
from jetbot import Robot
robot = Robot()

robot.forward(0.3)   # đi thẳng, tốc độ 30%
robot.left(0.3)      # quay trái
robot.right(0.3)     # quay phải
robot.backward(0.3)  # lùi
robot.stop()         # dừng

# điều khiển từng bánh (trái, phải), giá trị -1.0 .. 1.0
robot.set_motors(0.4, 0.2)   # rẽ nhẹ vì bánh trái nhanh hơn
```

> **Mẹo an toàn:** đặt JetBot lên **giá kê cho bánh không chạm đất** khi thử code lần đầu, tránh robot lao khỏi bàn. Luôn để một ô `robot.stop()` sẵn để bấm khi cần.

# 5. Xem camera trực tiếp

```python
from jetbot import Camera, bgr8_to_jpeg
import ipywidgets, traitlets

camera = Camera.instance(width=224, height=224)
image = ipywidgets.Image(format='jpeg')
traitlets.dlink((camera, 'value'), (image, 'value'), transform=bgr8_to_jpeg)
display(image)   # xem luồng camera ngay trong notebook
```

Ảnh 224×224 là kích thước quen thuộc để đưa vào mạng nơ-ron.

# 6. Điều khiển từ xa (teleoperation)

![Điều khiển JetBot bằng tay cầm không dây](/itcourses/api/uploads/8b4f19eb865d_jetbot_gamepad.jpg)

Notebook `teleoperation` cho phép lái JetBot bằng **tay cầm game** (hoặc phím) trong khi xem camera. Đây không chỉ để "chơi" — mà là cách **thu thập dữ liệu huấn luyện**: bạn tự lái đúng, robot ghi lại hình ảnh kèm hành động để học theo.

# 7. Quy trình phát triển AI

![Vòng phát triển AI trên JetBot](/itcourses/api/uploads/27b5215e6bfa_jetbot_devflow.svg)

Hầu hết dự án JetBot đi theo bốn bước, lặp lại nhiều vòng:

1. **Thu thập dữ liệu** — lái robot và chụp ảnh, gán nhãn. Ví dụ: ảnh "trống" / "vướng" cho tránh va chạm; điểm lái (x,y) cho bám đường.
2. **Huấn luyện** — train một mạng CNN (thường transfer learning từ ResNet/AlexNet). Có thể train ngay trên JetBot, nhưng **nhanh hơn nhiều nếu train trên máy có GPU** rồi mang mô hình về.
3. **Triển khai** — chép tệp mô hình (`.pth`) lên JetBot; tối ưu bằng **TensorRT** để chạy nhanh trên Jetson.
4. **Chạy thực tế** — robot suy luận theo thời gian thực và điều khiển động cơ theo kết quả.

Kết quả chưa tốt (đâm vào tường, chạy lệch)? Đó là chuyện bình thường — **quay lại bước 1**, thu thêm dữ liệu ở tình huống robot hay sai, rồi huấn luyện lại.

# 8. Các cách kết nối khác

- **SSH:** `ssh jetbot@<IP>` (mật khẩu thường là `jetbot`) để dùng dòng lệnh, cài thư viện, chạy script không cần notebook.
- **HTTP / WebSocket:** có thể viết một dịch vụ nhỏ trên JetBot để điều khiển qua API mạng (nút bấm trên web, hoặc để chương trình khác gọi) — xem bài **"Điều khiển JetBot qua mạng (HTTP / WebSocket)"**.
- **ROS:** có bản cộng đồng đưa JetBot vào ROS để ghép với hệ điều hướng chuẩn — xem bài **"ROS — Hệ điều hành cho Robot"**.

# 9. Mẹo và khắc phục sự cố

- **Robot không chạy dù code đúng:** pin yếu, hoặc chưa cấp đủ dòng cho động cơ → sạc pin.
- **Không mở được `:8888`:** máy tính và JetBot khác mạng, sai IP, hoặc JetBot chưa khởi động xong.
- **Một bánh không quay / chạy lệch:** kiểm tra dây động cơ và địa chỉ I2C của driver; hiệu chỉnh bằng `set_motors` với hệ số bù.
- **Máy nóng, chậm dần:** Jetson bị **giảm xung do nhiệt** → thêm quạt/tản nhiệt.
- **Camera không hiện:** camera đang bị notebook khác giữ → khởi động lại kernel, hoặc gọi `camera.stop()` trước khi mở lại.

## Tự kiểm tra

1. Máy tính và JetBot cần điều kiện gì để điều khiển qua trình duyệt được?
2. `robot.set_motors(0.4, 0.2)` khiến robot làm gì, vì sao?
3. Vì sao nên train mô hình trên máy có GPU thay vì trên JetBot?
4. Kể bốn bước của quy trình phát triển AI trên JetBot.
5. Robot chạy đúng trong code nhưng ngoài đời không nhúc nhích — bạn kiểm tra gì đầu tiên?

---
*Bài viết thuộc mục Tài liệu — Phần cứng & IoT của ITCourses. Xem thêm: "JetBot — Tổng quan & bắt đầu", "Điều khiển JetBot qua mạng (HTTP / WebSocket)".*
