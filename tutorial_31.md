# Hướng dẫn viết script đọc thông tin pin trong JetBot

> **Mục tiêu.** Viết script Python đọc **điện áp / dòng điện / công suất** và **ước lượng % pin** của JetBot, thông qua cảm biến **INA219** giao tiếp **I²C**. Bài có 2 phiên bản script (dùng thư viện và đọc thanh ghi thô), hàm tính % pin cho pin **3S Li-ion**, cách chạy nền + cảnh báo, và các lỗi hay gặp.

---

## 1. JetBot lấy thông tin pin từ đâu?

JetBot (NVIDIA Jetson Nano + đế robot Waveshare/SparkFun) chạy bằng **3 cục pin 18650 mắc nối tiếp (3S Li-ion)** → điện áp **9.0 V (cạn) → 12.6 V (đầy)**, danh định 11.1 V. Trên bo nguồn/điều khiển động cơ có sẵn con **INA219** — một IC đo **điện áp bus, dòng điện và công suất** qua **I²C**. Chính con này cấp số liệu pin cho màn OLED thống kê của JetBot.

Nói cách khác: đọc pin JetBot = **đọc INA219 qua I²C**. Không cần thêm phần cứng.

<figure style="margin:1.2rem auto;text-align:center;max-width:380px">
  <img src="/itcourses/api/uploads/eae493111e8d_jetbot.jpg" alt="JetBot AI Kit (Jetson Nano + 3x 18650)" style="width:100%;border:1px solid #E5E7EB;border-radius:8px;background:#fff"/>
  <figcaption style="font-size:.82rem;color:#6B7280;margin-top:.4rem">JetBot (Jetson Nano) chạy bằng <b>3 pin 18650 (3S)</b>; bo điều khiển có sẵn <b>INA219</b> đo pin qua I²C.</figcaption>
</figure>

<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 150" width="620" style="max-width:100%;height:auto;display:block;margin:14px auto;background:#fff;border:1px solid #E5E7EB;border-radius:8px">
  <style>.b{rx:6;stroke-width:1.6}.t{font-family:Roboto,sans-serif;font-size:12px;fill:#0f172a;text-anchor:middle}.s{font-family:Roboto,sans-serif;font-size:10px;fill:#6b7280;text-anchor:middle}.a{stroke:#334155;stroke-width:1.6;marker-end:url(#h)}</style>
  <defs><marker id="h" markerWidth="9" markerHeight="9" refX="7" refY="3" orient="auto"><path d="M0,0 L7,3 L0,6 Z" fill="#334155"/></marker></defs>
  <rect class="b" x="16" y="52" width="120" height="46" fill="#e7f9ee" stroke="#059669"/><text class="t" x="76" y="72">Pin 3S Li-ion</text><text class="s" x="76" y="88">9.0–12.6 V</text>
  <rect class="b" x="215" y="50" width="110" height="50" fill="#fdeef2" stroke="#db2777"/><text class="t" x="270" y="72">INA219</text><text class="s" x="270" y="88">đo V / I / P</text>
  <rect class="b" x="400" y="52" width="90" height="46" fill="#e0edff" stroke="#2563eb"/><text class="t" x="445" y="72">I²C bus</text><text class="s" x="445" y="88">0x41</text>
  <rect class="b" x="545" y="50" width="90" height="50" fill="#fff7e6" stroke="#d97706"/><text class="t" x="590" y="72">Jetson</text><text class="s" x="590" y="88">script.py</text>
  <line class="a" x1="136" y1="75" x2="213" y2="75"/><line class="a" x1="325" y1="75" x2="398" y2="75"/><line class="a" x1="490" y1="75" x2="543" y2="75"/>
  <text class="s" x="176" y="70">dòng tải</text>
</svg>

## 2. Nguyên lý INA219

INA219 đo:
- **Bus voltage** — điện áp nguồn (chính là điện áp pin), thanh ghi `0x02`.
- **Shunt voltage** — sụt áp trên điện trở shunt (thường **0.1 Ω**), thanh ghi `0x01`; từ đó suy ra **dòng điện** = V_shunt / R_shunt.
- **Power** — công suất (cần nạp thanh ghi calibration `0x05` trước).

Địa chỉ I²C tuỳ đế: hay gặp **0x41** (JetBot Waveshare) hoặc **0x40** (mặc định). Luôn kiểm tra bằng `i2cdetect` trước.

## 3. Chuẩn bị

```bash
# 1) Kiểm tra I²C đã bật và tìm địa chỉ INA219 (bus 1 trên Jetson Nano)
sudo apt-get install -y i2c-tools
i2cdetect -y -r 1
#   → thấy 40 hoặc 41 trong bảng = địa chỉ INA219

# 2) Cài thư viện (chọn 1)
pip3 install pi-ina219      # cách nhanh, khuyên dùng
pip3 install smbus2         # cho bản đọc thanh ghi thô
```

> Nếu điều khiển JetBot **trong Docker container** (kiểu triển khai phổ biến của JetBot), nhớ chạy container với quyền truy cập I²C: `--device /dev/i2c-1` (hoặc `--privileged`). Người dùng phải thuộc nhóm `i2c` hoặc chạy `sudo`.

## 4. Script đọc pin — dùng thư viện `pi-ina219` (khuyên dùng)

```python
#!/usr/bin/env python3
from ina219 import INA219, DeviceRangeError

SHUNT_OHMS = 0.1          # điện trở shunt trên bo JetBot
I2C_ADDR   = 0x41         # đổi thành 0x40 nếu i2cdetect thấy 40

def read_battery():
    ina = INA219(SHUNT_OHMS, address=I2C_ADDR)
    ina.configure()
    v = ina.voltage()            # điện áp pin (V)
    try:
        i = ina.current()        # dòng điện (mA), >0 = đang xả
        p = ina.power()          # công suất (mW)
    except DeviceRangeError:
        i = p = float("nan")     # dòng vượt dải đo
    return v, i, p

if __name__ == "__main__":
    v, i, p = read_battery()
    print(f"Điện áp : {v:.2f} V")
    print(f"Dòng    : {i:.0f} mA")
    print(f"Công suất: {p:.0f} mW")
```

## 5. Script đọc pin — đọc thanh ghi thô (chỉ cần `smbus2`)

Hữu ích khi không cài được `pi-ina219`, hoặc để hiểu bản chất:

```python
#!/usr/bin/env python3
from smbus2 import SMBus
import time

BUS   = 1        # Jetson Nano: I²C bus 1
ADDR  = 0x41     # địa chỉ INA219
R_SHUNT = 0.1    # ohm

def read16(bus, reg):
    hi, lo = bus.read_i2c_block_data(ADDR, reg, 2)   # INA219 trả big-endian
    return (hi << 8) | lo

def signed16(x):
    return x - 65536 if x & 0x8000 else x

with SMBus(BUS) as bus:
    while True:
        bus_raw   = read16(bus, 0x02)                # BUS_VOLTAGE
        voltage   = (bus_raw >> 3) * 0.004           # LSB 4 mV, bỏ 3 bit thấp
        shunt_raw = signed16(read16(bus, 0x01))      # SHUNT_VOLTAGE
        v_shunt   = shunt_raw * 1e-5                 # LSB 10 µV
        current   = v_shunt / R_SHUNT                # A (định luật Ohm)
        print(f"Pin: {voltage:5.2f} V | Dòng: {current*1000:6.0f} mA")
        time.sleep(1)
```

> **Vì sao `>> 3`?** Thanh ghi bus voltage của INA219 để dữ liệu ở **13 bit cao** (bit 0–2 là cờ trạng thái), mỗi bước **4 mV**. Shunt voltage là số **có dấu**, LSB **10 µV**.

## 6. Ước lượng phần trăm pin (3S Li-ion)

Điện áp Li-ion **không tuyến tính** theo dung lượng, nên dùng **bảng tra + nội suy** cho sát thực tế:

```python
# (điện áp mỗi cell, % còn lại) — Li-ion điển hình
_CURVE = [(4.20,100),(4.10,90),(4.00,80),(3.90,70),(3.80,60),
          (3.70,50),(3.60,35),(3.50,20),(3.40,10),(3.20,5),(3.00,0)]

def battery_percent(pack_voltage, cells=3):
    vc = pack_voltage / cells                 # điện áp trung bình mỗi cell
    if vc >= _CURVE[0][0]:  return 100
    if vc <= _CURVE[-1][0]: return 0
    for (v1,p1),(v2,p2) in zip(_CURVE, _CURVE[1:]):
        if v2 <= vc <= v1:                    # nội suy tuyến tính trong đoạn
            return round(p2 + (p1-p2)*(vc-v2)/(v1-v2))
    return 0

# ví dụ: pin đọc được 11.4 V
print(battery_percent(11.4), "%")             # ~ 45%
```

| Điện áp pack (3S) | % pin (xấp xỉ) | Trạng thái |
|---|---|---|
| 12.6 V | 100% | Đầy |
| 11.7 V | ~70% | Tốt |
| 11.1 V | ~45% | Trung bình (danh định) |
| 10.5 V | ~15% | Nên sạc |
| ≤ 9.6 V | ~5% | **Sắp tắt — sạc ngay** |

> ⚠️ **Không xả kiệt** pin Li-ion dưới ~3.0 V/cell (≈ 9.0 V pack) — hại pin và có thể sập nguồn Jetson đột ngột (hỏng thẻ nhớ). Nên cảnh báo/hạ tải khi < ~10.2 V.

## 7. Chạy nền: log + cảnh báo

```python
#!/usr/bin/env python3
import time
from ina219 import INA219

ina = INA219(0.1, address=0x41); ina.configure()
LOW = 10.2   # ngưỡng cảnh báo (V)

while True:
    v = ina.voltage()
    pct = battery_percent(v)                  # hàm ở mục 6
    line = time.strftime('%H:%M:%S') + f"  {v:.2f}V  {pct}%"
    if v < LOW:
        line += "  ⚠️ PIN YẾU — HÃY SẠC"
        # tuỳ chọn: gửi cảnh báo (Telegram/NotiCenter), hoặc dừng động cơ an toàn
    print(line, flush=True)
    time.sleep(30)
```

Tích hợp thêm: hiển thị lên **OLED** thống kê của JetBot, ghi ra file log, hoặc đẩy vào **API agent** của robot (`/battery`) để dashboard đọc. Muốn chạy tự động khi bật máy → tạo **systemd service** trỏ tới script này.

## 8. Lỗi thường gặp

| Hiện tượng | Nguyên nhân & cách xử lý |
|---|---|
| `i2cdetect` không thấy 40/41 | I²C chưa bật, sai bus (thử `-y 0`/`-y 7`), dây lỏng, đế chưa cấp nguồn pin |
| `OSError: [Errno 121] Remote I/O error` | Sai địa chỉ, xung đột bus, hoặc thiết bị chưa sẵn sàng — kiểm tra lại địa chỉ bằng i2cdetect |
| `Permission denied /dev/i2c-1` | Thêm user vào nhóm `i2c` (`sudo usermod -aG i2c $USER`) rồi đăng nhập lại, hoặc chạy `sudo` |
| Chạy trong Docker không thấy I²C | Thêm `--device /dev/i2c-1` (hoặc `--privileged`) khi `docker run` |
| Điện áp đọc lệch | Sai `SHUNT_OHMS`, hoặc INA219 đo nhánh khác (một số đế đo đầu ra motor, không phải pin) |
| Dòng điện = 0 hoặc sai | Chưa nạp calibration (dùng `pi-ina219` sẽ tự nạp), hoặc shunt sai giá trị |
| Giá trị nhảy loạn khi robot chạy | Động cơ kéo dòng lớn gây nhiễu — lấy trung bình vài mẫu trước khi tính % |

## 9. Tham khảo
- Datasheet **TI INA219** (thanh ghi 0x00–0x05, LSB bus 4 mV / shunt 10 µV).
- Thư viện **`pi-ina219`** (chris-blay) và **`smbus2`**.
- NVIDIA/Waveshare **JetBot** — `jetbot` repo (script `stats`/OLED đọc INA219).
- Pin **18650 / 3S Li-ion** — đường cong xả và ngưỡng an toàn.
