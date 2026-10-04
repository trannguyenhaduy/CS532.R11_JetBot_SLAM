> **Hardware & IoT · Nhập môn** — JetBot là gì, gồm những bộ phận nào, hoạt động ra sao, và cách bắt đầu từ con số không.
> Sau bài này, để điều khiển robot qua mạng thì đọc tiếp tài liệu **"Điều khiển JetBot qua mạng (HTTP / WebSocket)"** trong cùng chủ đề.

## Sau bài này bạn sẽ

- Biết **JetBot là gì** và vì sao nó hay được dùng để học robot + AI.
- Gọi tên được **các bộ phận** của một chiếc JetBot và việc của từng cái.
- Hiểu **cách nó di chuyển** bằng hai bánh (differential drive).
- Biết các bước **cài đặt và khởi động** lần đầu.
- Biết JetBot **làm được những gì** và học tiếp ở đâu.

# 1. JetBot là gì?

![JetBot AI Kit trên nền Jetson Nano — GPU 128 nhân, CPU 4 nhân, 4GB RAM](/itcourses/api/uploads/5f89eb24a4db_jetbot_hero.jpg)


**JetBot** là một robot nhỏ hai bánh, mã nguồn mở, do NVIDIA giới thiệu để dạy **robot học máy** (AI robotics). Trái tim của nó là một máy tính nhúng **Jetson Nano** — nhỏ bằng lòng bàn tay nhưng có **GPU**, nên chạy được mạng nơ-ron ngay trên robot mà không cần gửi dữ liệu lên máy chủ.

Nói ngắn gọn: JetBot = **một máy tính Linux có GPU, gắn thêm camera và hai bánh xe**. Vì là mã nguồn mở, có nhiều phiên bản lắp ráp khác nhau (NVIDIA, Waveshare, SparkFun…) nhưng cấu trúc thì giống nhau.

> <span style="color:#0E7490">**Gợi ý:** Vì sao lại là JetBot chứ không phải một robot Arduino? Vì Jetson Nano đủ mạnh để **nhìn và tự quyết định** — nhận diện vật thể, bám line, tránh vật cản bằng camera — những việc mà vi điều khiển nhỏ không làm nổi.</span>

# 2. Các bộ phận

![Trọn bộ linh kiện JetBot AI Kit (chưa gồm Jetson Nano)](/itcourses/api/uploads/988d84290ee7_jetbot_parts.jpg)

![Card WiFi/Bluetooth hai băng AC8265](/itcourses/api/uploads/74b4517cb198_jetbot_ac8265.jpg)


![Kiến trúc một chiếc JetBot: Jetson Nano điều khiển, pin nuôi toàn bộ, mạch cầu H quay hai bánh](/itcourses/api/uploads/15865684fddf_jetbot_arch.svg)

Bài này lấy **JetBot AI Kit (Waveshare)** — bản robot **do chính NVIDIA khuyến nghị** để nghiên cứu robot AI tự hành trên Jetson Nano — làm ví dụ tham chiếu. Đây là bộ được bán phổ biến ở Việt Nam (ví dụ Hshop.vn, SKU 16909).

| Bộ phận | Chi tiết trong JetBot AI Kit | Việc của nó |
|---|---|---|
| **Jetson Nano** | *Mua rời*, không kèm trong kit — cần bản **Developer Kit B01** (hoặc bản 16GB eMMC) tương thích | Bộ não: chạy Linux, xử lý ảnh, chạy AI, ra lệnh động cơ |
| **Bo mạch mở rộng** (expansion board) | Của Waveshare, **tích hợp sẵn**: quản lý & sạc pin, mạch điều khiển động cơ, các cổng | Gộp mọi mạch phụ trợ vào một board → lắp nhanh, gọn dây |
| **Camera** | **IMX219, góc rộng 160°, độ phân giải 8MP** | "Mắt" robot: tự hành, nhận diện khuôn mặt, nhận diện vật thể |
| **WiFi/Bluetooth** | Card **AC8265** (2 băng tần) + ăng-ten | Điều khiển & xem camera không dây từ xa |
| **Tay cầm** | **Tay điều khiển PS2 không dây** kèm theo | Lái robot bằng tay, không cần code |
| **Hai động cơ + bánh** | Động cơ TT gắn qua bo mạch điều khiển | Di chuyển; mỗi bánh một động cơ (bánh vi sai) |
| **Khung** | **Kim loại** chắc chắn | Giữ mọi thứ; bền hơn khung nhựa/mica |
| **Pin** | Pin **18650** + mạch sạc trên bo mở rộng | Nuôi toàn bộ, sạc ngay trên robot |
| **Màn OLED nhỏ** | Trên bo mở rộng | Hiện địa chỉ IP, mức pin, trạng thái |

> <span style="color:#C0392B">**Bắt buộc:** **Kit KHÔNG kèm Jetson Nano.** Phải mua thêm đúng bản tương thích (Jetson Nano Developer Kit **B01**, hoặc bản 16GB eMMC). Mua kit về mà thiếu Nano thì chưa chạy được gì — đây là nhầm lẫn hay gặp nhất khi đặt hàng.</span>

> <span style="color:#B9770E">**Lưu ý:** Điểm tiện của bản Waveshare là **bo mạch mở rộng tích hợp sẵn** phần sạc pin, điều khiển động cơ và OLED — bạn không phải tự đấu từng mạch rời. Đổi lại, sơ đồ khối bên dưới vẫn đúng về mặt *chức năng*: Nano ra lệnh, mạch cầu H trên bo đó quay động cơ, pin nuôi tất cả.</span>

# 3. Robot di chuyển bằng cách nào?

![Vòng điều khiển JetBot: camera → CNN → động cơ](/itcourses/api/uploads/aa4a82c9d772_jetbot_pipeline.svg)


![Năm kiểu di chuyển của JetBot: thẳng, lùi, rẽ phải, rẽ trái, xoay tại chỗ](/itcourses/api/uploads/f709f8dc0e57_jetbot_move.svg)

JetBot dùng kiểu lái **hai bánh vi sai** (differential drive): không có vô-lăng, hướng đi quyết định bởi **chênh lệch tốc độ hai bánh**.

| Muốn robot… | Bánh trái | Bánh phải |
|---|---|---|
| Đi thẳng | tiến, tốc độ v | tiến, tốc độ v |
| Lùi | lùi | lùi |
| Rẽ phải | tiến nhanh | tiến chậm (hoặc dừng) |
| Rẽ trái | tiến chậm (hoặc dừng) | tiến nhanh |
| Xoay tại chỗ | tiến | lùi |

Mỗi lệnh thực chất chỉ là **đặt tốc độ cho hai động cơ**, giá trị từ −1 (lùi hết) đến +1 (tiến hết). Mọi chuyển động phức tạp đều ghép lại từ hai con số đó.

> <span style="color:#0E7490">**Gợi ý:** Đi thẳng cho *đúng* khó hơn tưởng: hai động cơ không bao giờ giống hệt nhau, nên đặt cùng tốc độ robot vẫn lệch dần. Robot thật thường cần **hiệu chỉnh** (calibration) hoặc một cảm biến (như IMU/gyro) để giữ hướng.</span>

# 4. Bắt đầu lần đầu

![Điều khiển từ xa bằng tay cầm không dây — cách thu thập dữ liệu huấn luyện](/itcourses/api/uploads/8b4f19eb865d_jetbot_gamepad.jpg)


Các bước chung cho hầu hết phiên bản JetBot:

1. **Lắp phần cứng:** khung kim loại, hai động cơ + bánh, gắn **Jetson Nano (mua riêng)** lên bo mở rộng Waveshare, cắm camera IMX219 vào cổng CSI, gắn card WiFi AC8265 + ăng-ten, lắp pin 18650.
2. **Chuẩn bị thẻ nhớ:** tải ảnh hệ điều hành **JetBot SD card image** (Ubuntu + JetPack + sẵn thư viện), ghi vào thẻ microSD bằng balenaEtcher.
3. **Khởi động:** lắp thẻ, cấp nguồn. Lần đầu nên cắm màn hình HDMI + bàn phím để nối WiFi; sau đó **màn OLED sẽ hiện IP**.
4. **Kết nối:** từ máy tính cùng mạng, mở trình duyệt vào `http://<IP-của-JetBot>:8888` — đó là **JupyterLab**, nơi viết và chạy code Python điều khiển robot.
5. **Chạy thử:** mở notebook mẫu `basic_motion` để cho robot tiến/lùi/rẽ, kiểm tra camera hiện hình.

> <span style="color:#C0392B">**Bắt buộc:** Kê robot lên giá cho **bánh không chạm đất** khi chạy lệnh động cơ lần đầu. Một lệnh sai tốc độ có thể làm nó lao khỏi bàn. Chỉ đặt xuống đất khi đã chắc lệnh đúng.</span>

> <span style="color:#B9770E">**Lưu ý:** JetBot ăn pin nhanh và **tụt áp khi pin yếu** — biểu hiện là robot tự khởi động lại hoặc chạy giật khi động cơ tải nặng. Nếu robot "chập chờn" khó hiểu, hãy **sạc pin trước** rồi mới nghi ngờ phần mềm.</span>

# 5. JetBot làm được gì?

![Các chức năng AI: nhận diện khuôn mặt, phát hiện/phân loại vật thể](/itcourses/api/uploads/6a8641844e9f_jetbot_ai.jpg)


Ba bài thực hành kinh điển, tăng dần độ khó:

| Bài | Ý tưởng | Học được gì |
|---|---|---|
| **Teleoperation** | Điều khiển robot bằng tay qua trình duyệt | Làm quen động cơ, camera, mạng |
| **Collision avoidance** | Robot tự né vật cản | Thu dữ liệu ảnh, huấn luyện một mạng phân loại "đi được / bị chặn" |
| **Road following** | Robot tự bám theo đường | Hồi quy ảnh → góc lái; nền tảng của xe tự lái thu nhỏ |

Điểm hay của JetBot là cả ba đều chạy **AI ngay trên robot** nhờ GPU của Nano — thu ảnh, suy luận, ra lệnh động cơ, tất cả trong một vòng lặp trên chính con robot.

# 5b. Ứng dụng ngoài đời thực

JetBot là **"xe tự lái thu nhỏ"**: một robot hai bánh (differential drive) nhìn thế giới bằng một camera và chạy mạng nơ-ron ngay trên máy. Bài toán của nó — *nhìn pixel → quyết định → điều khiển bánh xe trong một vòng lặp* — chính là bài toán của xe tự lái, robot kho hàng và robot giao hàng, chỉ khác quy mô.

![JetBot và lớp robot 2 bánh nhìn bằng camera: ứng dụng thực tế](/itcourses/api/uploads/870c5c50e79a_jetbot_apps.svg)

**Các nhóm ứng dụng thực tế của lớp robot này** (phân biệt quy mô JetBot — giáo dục — với quy mô công nghiệp):

- **📦 Kho hàng &amp; logistics (AGV/AMR):** chở kệ, thùng, kiện trong trung tâm phân phối. *Amazon **Kiva → Proteus**: hơn 500.000 robot; Proteus (2022) là robot tự hành hoàn toàn đầu tiên của Amazon.* Bài "bám đường" của JetBot chính là AGV dò line thu nhỏ.
- **🚚 Giao hàng chặng cuối:** robot vỉa hè chở đồ ăn/bưu kiện tới tận nơi. ***Starship** (đã giao hàng triệu lần ở 20+ nước), **Serve Robotics**, **Nuro**.*
- **🌾 Nông nghiệp chính xác:** đi giữa luống, dùng thị giác phân biệt cây trồng với cỏ dại rồi diệt cỏ chọn lọc (giảm thuốc tới ~95%). *Naïo **Oz/Dino**, **FarmWise Titan**.*
- **🛡️ An ninh &amp; giám sát:** tuần tra theo lộ trình, quay video, phát hiện bất thường. *Knightscope **K5**.*
- **🍽️ Robot phục vụ trong nhà:** bưng đồ ăn, dọn bàn ở nhà hàng/khách sạn, né người trong không gian đông. *Bear **Servi**, Pudu **BellaBot**.*
- **🎓 Giáo dục &amp; đua xe AI:** nền tảng dạy lái tự động và học tăng cường. *Đây là "sân nhà" của JetBot — cùng họ có **JetRacer**, **DonkeyCar**, **AWS DeepRacer**.*

**Bài mẫu có sẵn của JetBot** dẫn dắt bạn đi từ dễ tới khó: điều khiển từ xa + thu thập dữ liệu → **tránh va chạm** (phân loại ảnh "trống/vướng") → **bám đường** (hồi quy điểm lái) → **bám/đuổi vật** (phát hiện vật). Ghép bám đường + tránh va chạm lại là đã có một pipeline xe tự lái mini.

> **Giới hạn cần nhớ:** JetBot chỉ có **một camera đơn** (không đo được chiều sâu — hành vi tránh vật là học từ hình ảnh, không phải đo khoảng cách thật); Jetson Nano chỉ chạy được model nhỏ; model **dễ hỏng khi đổi ánh sáng/căn phòng** (distribution shift); không có chuẩn an toàn, tải trọng ~0, pin ngắn, chỉ chạy trên đường/phòng đã chuẩn bị. Nó dạy đúng **thuật toán và vòng lặp cảm nhận–hành động**, nhưng không có độ bền, dự phòng cảm biến và an toàn của một robot triển khai thật.


# 6. Mô hình 3D (Isaac Sim / Isaac Lab)

Có thể mô phỏng JetBot trong **NVIDIA Isaac Sim / Isaac Lab** trước khi chạy robot thật — huấn luyện và thử thuật toán trong môi trường ảo an toàn. Dưới đây là ảnh dựng từ một mô hình JetBot 3D (định dạng **USD** — Universal Scene Description) có gắn thêm camera FPV, tạo trong Isaac Sim:

![Mô hình 3D JetBot (dựng từ tệp USD)](/itcourses/api/uploads/f4dd641c7c45_jetbot_3d_render.png)

*Ảnh dựng từ mô hình `jetbot.usd`. Thấy rõ khối tản nhiệt/quạt của Jetson, module camera FPV phía trước, hai ăng-ten WiFi và bánh xe.*

- **Tải mô hình 3D:** [jetbot.usd (32 MB)](/itcourses/api/uploads/1cc42a5144bd_jetbot.usd) — mở bằng Isaac Sim, Omniverse, hoặc công cụ đọc USD.
- **Nguồn:** repo [getting-started-with-isaac-lab](https://github.com/neuraljoel/getting-started-with-isaac-lab) của *neuraljoel* (giấy phép **MIT**). Tệp thuộc phần "Adding an FPV Camera to Jetbot".

> **USD là gì?** Universal Scene Description là định dạng mô tả cảnh 3D do Pixar tạo, được NVIDIA Omniverse/Isaac dùng làm chuẩn. Một tệp `.usd` có thể chứa hình học, vật liệu, khớp nối và cảm biến của cả con robot.

# 7. Học tiếp

- **Điều khiển JetBot qua mạng (HTTP / WebSocket)** — tài liệu cùng chủ đề, đi sâu vào cách gửi lệnh điều khiển từ xa.
- **Nhập môn LiDAR** — nếu muốn robot "nhìn" khoảng cách thay vì chỉ dùng camera.
- Tài liệu chính thức: kho **NVIDIA-AI-IOT/jetbot** trên GitHub (hướng dẫn lắp ráp, ảnh thẻ nhớ, notebook mẫu).
- Thông số bộ **JetBot AI Kit (Waveshare)** tham khảo tại Hshop.vn (SKU 16909) và trang nhà sản xuất Waveshare.


> *Ảnh sản phẩm trong bài: hshop.vn / Waveshare (JetBot AI Kit).*

## Tự kiểm tra

- [ ] Trái tim của JetBot là bộ phận nào? Vì sao nó đặc biệt so với vi điều khiển thường?
- [ ] Vì sao giữa Jetson Nano và động cơ phải có mạch cầu H?
- [ ] "Differential drive" nghĩa là gì? Muốn robot xoay tại chỗ thì hai bánh quay thế nào?
- [ ] Truy cập vào JupyterLab của JetBot bằng địa chỉ nào?
- [ ] Vì sao nên kê bánh khỏi mặt đất khi chạy lệnh động cơ lần đầu?
- [ ] Kể ba bài thực hành kinh điển với JetBot theo thứ tự khó dần.
