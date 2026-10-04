# Camera Line Scan và Area Scan: Chọn đúng loại cho bài toán thị giác máy

Khi xây dựng một hệ thống thị giác máy (machine vision) — kiểm tra sản phẩm trên băng chuyền, đọc hoá đơn, soi lỗi bề mặt — quyết định đầu tiên và quan trọng nhất là **chọn loại camera**. Có hai kiến trúc cảm biến cơ bản: **area scan** (quét vùng) và **line scan** (quét dòng). Chọn sai loại thường khiến cả hệ thống phải làm lại từ đầu.

Bài viết này giải thích cách mỗi loại hoạt động, ưu nhược điểm, và một khung ra quyết định thực dụng.

---

## 1. Area scan camera — chụp cả khung hình một lần

Đây là loại camera "quen thuộc" nhất: cảm biến là một **ma trận điểm ảnh 2D** (ví dụ 1920 × 1080). Mỗi lần chụp, toàn bộ khung hình được ghi lại đồng thời, giống như máy ảnh điện thoại.

**Cơ chế:**
- Cảm biến hình chữ nhật, ví dụ `W × H` pixel.
- Một lần phơi sáng (exposure) → một ảnh 2D hoàn chỉnh.
- Tốc độ tính bằng **fps** (frames per second).

**Ưu điểm:**
- Đơn giản, dễ setup, dễ căn chỉnh (align).
- Bắt được vật thể **đứng yên hoặc chuyển động chậm**.
- Hệ sinh thái phong phú: ống kính, đèn, thư viện (OpenCV, ...) đều mặc định cho area scan.
- Chi phí ban đầu thấp.

**Nhược điểm:**
- Độ phân giải bị giới hạn bởi kích thước cảm biến. Muốn soi vật **rất dài và rất chi tiết** thì phải ghép nhiều ảnh (stitching) — phức tạp và dễ lỗi mối nối.
- Vật chuyển động nhanh sẽ bị **nhoè (motion blur)** nếu thời gian phơi sáng không đủ ngắn.

**Dùng khi:**
- Kiểm tra linh kiện điện tử, đọc nhãn/mã QR, nhận diện khuôn mặt.
- Vật thể có kích thước cố định, vừa trong một khung hình.
- **Đọc tài liệu/hoá đơn dạng tờ rời** — cả tờ nằm gọn trong một khung.

---

## 2. Line scan camera — dựng ảnh từng dòng một

Cảm biến của line scan chỉ là **một hàng pixel duy nhất** (1D), ví dụ `8192 × 1`. Bản thân nó không tạo ra ảnh 2D. Ảnh được **dựng dần**: vật thể (hoặc camera) di chuyển, và mỗi thời điểm camera chụp một "lát cắt" một-pixel-cao. Ghép hàng nghìn lát cắt lại theo thời gian → một ảnh 2D liền mạch, cao tuỳ ý.

**Cơ chế:**
- Cảm biến 1 dòng, độ rộng rất lớn (4k, 8k, 16k pixel).
- Vật phải **chuyển động đều** qua vùng quét.
- Tốc độ tính bằng **line rate** (kHz — số dòng/giây), không phải fps.
- Gần như luôn cần **encoder** gắn vào trục băng chuyền để đồng bộ mỗi dòng chụp với một bước dịch chuyển vật lý — nếu không, ảnh sẽ bị giãn/nén theo phương chuyển động.

**Ưu điểm:**
- Độ phân giải theo chiều ngang **cực cao** với chi phí hợp lý — 8192 pixel/dòng là bình thường.
- Ảnh **dài vô hạn** theo chiều chuyển động, không có mối nối.
- Không motion blur theo phương quét (vì mỗi dòng phơi sáng độc lập, đồng bộ với chuyển động).
- Lý tưởng cho vật **liên tục, rất dài, hoặc hình trụ**.

**Nhược điểm:**
- Bắt buộc phải có **chuyển động đều và ổn định** — không chụp được vật đứng yên.
- Phức tạp hơn: cần encoder, đồng bộ, hiệu chỉnh line rate theo tốc độ.
- **Đèn chiếu sáng khắt khe**: toàn bộ năng lượng sáng dồn vào một dải hẹp, thường phải dùng đèn line-light cường độ cao.
- Căn chỉnh cảm biến vuông góc với hướng chuyển động đòi hỏi độ chính xác cao.

**Dùng khi:**
- **Web inspection**: giấy, vải, film, thép cuộn, in ấn chạy liên tục.
- Vật hình trụ quay tròn (soi toàn bộ mặt ngoài).
- Phân loại nông sản, kiểm tra bề mặt trên băng chuyền tốc độ cao.

---

## 3. So sánh nhanh

| Tiêu chí | Area scan | Line scan |
|---|---|---|
| Cảm biến | Ma trận 2D (W×H) | Một dòng 1D (W×1) |
| Đơn vị tốc độ | fps | line rate (kHz) |
| Vật thể | Đứng yên / chậm | Chuyển động đều, liên tục |
| Độ phân giải chiều dài | Giới hạn / cần stitching | Vô hạn, liền mạch |
| Encoder | Không cần | Gần như bắt buộc |
| Yêu cầu đèn | Vừa phải | Cao (line light) |
| Độ khó setup | Thấp | Cao |
| Chi phí hệ thống | Thấp hơn | Cao hơn |
| Ứng dụng điển hình | Linh kiện, khuôn mặt, tờ rời | Cuộn vải/giấy/thép, vật trụ |

---

## 4. Khung ra quyết định

Trả lời 4 câu hỏi theo thứ tự:

1. **Vật thể có chuyển động đều và liên tục không?**
   - Không (đứng yên / rời từng cái) → **area scan**.
   - Có (băng chuyền chạy liên tục, cuộn liệu) → cân nhắc **line scan**.

2. **Vật có "vô tận" theo một chiều không?** (giấy cuộn, vải, dây chuyền không có điểm dừng)
   - Có → **line scan** (area scan sẽ phải stitching).

3. **Cần độ phân giải cực cao trên vật rất rộng/dài không?**
   - Có → **line scan** (8k/16k pixel một dòng rẻ hơn nhiều so với cảm biến area 2D tương đương).

4. **Ngân sách và độ phức tạp cho phép tới đâu?**
   - Nếu bài toán area scan giải được → chọn area scan cho đơn giản, rẻ, ổn định.

> **Nguyên tắc:** Mặc định dùng **area scan**. Chỉ chuyển sang **line scan** khi có lý do bắt buộc: vật liên tục/vô tận, hoặc yêu cầu độ phân giải mà cảm biến 2D không đáp ứng nổi.

---

## 5. Lưu ý thực chiến

- **Đồng bộ encoder (line scan):** dùng tín hiệu encoder để trigger từng dòng, đảm bảo tỉ lệ pixel/mm không đổi khi tốc độ băng chuyền dao động.
- **Ánh sáng:** line scan cần line light mạnh, đặt đúng góc; area scan linh hoạt hơn (dome, backlight, coaxial).
- **Ống kính:** line scan độ phân giải cao đòi hỏi lens chất lượng phủ hết chiều rộng cảm biến, ít méo (distortion) ở rìa.
- **Bandwidth dữ liệu:** cả hai đều có thể tạo luồng dữ liệu lớn (CameraLink, CoaXPress, GigE Vision, USB3 Vision) — tính toán băng thông trước khi chọn giao tiếp.
- **Hiệu chỉnh:** line scan phải hiệu chỉnh line rate ↔ tốc độ vật để ảnh không bị co giãn theo phương chuyển động.

---

## 6. Liên hệ với bài toán đọc hoá đơn / OCR

Khi số hoá **hoá đơn dạng tờ rời**, area scan là lựa chọn tự nhiên: đặt tờ giấy, chụp một phát, đưa vào pipeline detect + OCR. Ngược lại, nếu quét **cuộn giấy in liên tục** hoặc kiểm tra chất lượng bản in tốc độ cao, line scan mới cho ảnh dài liền mạch, độ nét đồng đều — điều kiện cần để tách dòng (text-line detection) và nhận dạng chính xác.

Nói cách khác: **loại camera quyết định chất lượng ảnh đầu vào, và chất lượng ảnh đầu vào quyết định trần độ chính xác của mọi mô hình phía sau.** Chọn camera đúng ngay từ đầu là bước rẻ nhất để cải thiện toàn hệ thống.

---

## Tóm tắt

- **Area scan** = ma trận 2D, chụp cả khung, cho vật đứng yên/tờ rời — đơn giản, rẻ, mặc định nên dùng.
- **Line scan** = một dòng pixel, dựng ảnh theo chuyển động, cho vật liên tục/vô tận và độ phân giải cực cao — mạnh nhưng phức tạp, cần encoder và đèn tốt.
- Bắt đầu bằng area scan; chỉ đổi sang line scan khi bài toán thực sự yêu cầu.
