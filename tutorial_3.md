# Điều khiển JetBot qua mạng (HTTP / WebSocket)

Tài liệu này hướng dẫn điều khiển **JetBot** (robot NVIDIA Jetson Nano) qua mạng bằng **agent điều khiển** chạy sẵn trên mỗi bot. Agent (`jetbot_agent.py`) chạy trong container `jetbot_jupyter`, khởi tạo Motor HAT (PCA9685) **một lần** và phục vụ lệnh qua **HTTP thuần, cổng 8100** — độ trễ mili-giây (nếu `import jetbot.Robot` mỗi lần sẽ mất ~4.7s, không dùng cho điều khiển thời gian thực được).

---

## 1. Kiến trúc

![Kiến trúc điều khiển JetBot qua mạng](/itcourses/api/uploads/d25ca025ac87_jetbot_net_arch.svg)

```
[Máy của bạn / script]
      │  HTTP :8100 (trực tiếp trong cùng LAN)
      ▼
[JetBot]  jetbot_agent.py  ──▶  Motor HAT (2 bánh), CSI camera, IMU
      ▲
      │  proxy /api/robots/{bot}/...
[Fleet server]  ◀── mỗi bot tự đăng ký IP/status lên server
```

Hai cách gọi lệnh:
- **Trực tiếp tới bot:** `http://<IP_BOT>:8100/...` (cùng mạng LAN, nhanh nhất).
- **Qua fleet server:** `http://<SERVER>/api/robots/{bot}/...` (server proxy tới bot, tiện khi không biết IP hiện tại của bot).

> Agent bật **CORS-open** nên gọi được từ trình duyệt/JS. Ví dụ dưới dùng IP mẫu `192.168.16.44` (bot2) — thay bằng IP bot của bạn.

---

## 2. Danh sách endpoint của agent

| Method | Đường dẫn | Chức năng |
|---|---|---|
| GET | `/health` | `{ok:true}` — kiểm tra sống |
| GET | `/status` | `{ok,left,right,moving,age_ms,uptime_s,camera,motor,imu,...}` |
| GET | `/version` | phiên bản + SHA agent |
| POST | `/drive` `{left,right}` | đặt tốc độ 2 bánh **[-1..1]**, đồng thời "vỗ" watchdog |
| POST | `/stop` | dừng ngay lập tức |
| GET | `/snapshot` | 1 ảnh JPEG (`?flip=&w=&h=&q=`) |
| GET | `/stream` | luồng MJPEG (`?flip=&w=&h=&fps=&q=`) |
| GET | `/imu` | heading/rate/temp (chỉ bot có MPU-6050) |
| POST | `/imu/zero` | reset heading (bot phải đứng yên) |
| GET | `/ws` (Upgrade) | kênh WebSocket điều khiển liên tục |

**Đọc các trường trong `/status`:** `left`/`right` = tốc độ đang đặt; `moving` = có đang chạy không; `age_ms` = mili-giây kể từ lệnh drive gần nhất (dùng để biết watchdog sắp dừng); `motor.ok` = Motor HAT init thành công chưa; `camera`/`imu` = trạng thái ngoại vi.

---

## 3. Kiểm tra kết nối

```bash
curl http://192.168.16.44:8100/health
# -> {"ok": true}

curl http://192.168.16.44:8100/status
# -> {"ok":true,"left":0,"right":0,"moving":false,"uptime_s":..., "camera":..., "motor":{...}, "imu":{...}}
```

---

## 4. Điều khiển động cơ (HTTP)

Giá trị mỗi bánh trong khoảng **-1.0 → 1.0** (`1`=tiến hết ga, `-1`=lùi, `0`=dừng).

```bash
# Đi thẳng nửa ga
curl -X POST http://192.168.16.44:8100/drive \
     -H "Content-Type: application/json" \
     -d '{"left":0.5,"right":0.5}'

# Quay tại chỗ sang phải
curl -X POST http://192.168.16.44:8100/drive -d '{"left":0.4,"right":-0.4}'

# Dừng
curl -X POST http://192.168.16.44:8100/stop
```

**Bánh trái/phải ánh xạ ra chuyển động thế nào (differential drive):** hai bánh bằng nhau → đi thẳng; trái nhanh hơn phải → rẽ phải; hai bánh ngược dấu → **quay tại chỗ**. Muốn đi thẳng thật thẳng mà bot cứ lệch, hãy bù nhẹ một bánh (ví dụ `{"left":0.5,"right":0.47}`).

### ⚠️ Deadman watchdog (rất quan trọng)

Agent có **watchdog 1 giây**: nếu **không** nhận lệnh `/drive` mới trong vòng **1s**, nó **tự động dừng động cơ**. Đây là cơ chế an toàn — nếu script treo hoặc mất mạng, bot không lao đi mất kiểm soát.

➡️ Hệ quả: để bot chạy **liên tục**, bạn phải gửi lệnh `/drive` lặp lại (ví dụ mỗi 0.2–0.5s), không phải gửi một lần rồi thôi.

---

## 5. Điều khiển bằng Python (vòng lặp giữ tốc độ)

```python
import requests, time

BOT = "http://192.168.16.44:8100"

def drive(left, right):
    requests.post(f"{BOT}/drive", json={"left": left, "right": right}, timeout=0.5)

def stop():
    requests.post(f"{BOT}/stop", timeout=0.5)

try:
    # Đi thẳng 2 giây — phải lặp lệnh để không bị watchdog dừng
    t_end = time.time() + 2.0
    while time.time() < t_end:
        drive(0.5, 0.5)
        time.sleep(0.2)      # < 1s để "vỗ" watchdog
finally:
    stop()                   # luôn dừng khi kết thúc / lỗi
```

---

## 6. Lớp bọc Python gọn gàng (giữ nhịp tự động)

Việc "phải nhớ lặp lệnh mỗi 0.2s" dễ quên. Đóng gói nó vào một **class có luồng nền tự vỗ watchdog** và tự dừng khi thoát (context manager):

```python
import requests, threading, time

class JetBot:
    def __init__(self, ip, hz=5):
        self.base = f"http://{ip}:8100"
        self.s = requests.Session()
        self._lr = (0.0, 0.0)
        self._run = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()
        self._period = 1.0 / hz

    def _loop(self):
        while self._run:
            l, r = self._lr
            try:
                self.s.post(f"{self.base}/drive",
                            json={"left": l, "right": r}, timeout=0.5)
            except requests.RequestException:
                pass                       # mất gói lẻ thì bỏ qua
            time.sleep(self._period)

    def drive(self, left, right):          # chỉ ĐẶT mục tiêu; luồng nền lo gửi
        self._lr = (left, right)

    def stop(self):
        self._lr = (0.0, 0.0)
        try: self.s.post(f"{self.base}/stop", timeout=0.5)
        except requests.RequestException: pass

    def close(self):
        self._run = False; self._t.join(timeout=1); self.stop()

    def __enter__(self): return self
    def __exit__(self, *a): self.close()

# Dùng:
with JetBot("192.168.16.44") as bot:
    bot.drive(0.5, 0.5); time.sleep(2)     # đi thẳng 2s
    bot.drive(0.4, -0.4); time.sleep(1)    # quay phải 1s
# thoát khối with -> tự stop, tự tắt luồng
```

Từ giờ bạn chỉ gọi `bot.drive(l, r)` **một lần** cho mỗi ý định; luồng nền tự lặp để watchdog không dừng.

---

## 7. Điều khiển độ trễ thấp bằng WebSocket

Với điều khiển tay hoặc tần suất cao, dùng **WebSocket** (`/ws`) — một kết nối bền, mỗi khung text là 1 lệnh JSON, không phải bắt tay TCP mỗi lệnh:

```python
# pip install websocket-client
import websocket, json, time

ws = websocket.create_connection("ws://192.168.16.44:8100/ws")
try:
    for _ in range(20):
        ws.send(json.dumps({"cmd": "drive", "left": 0.4, "right": 0.4}))
        time.sleep(0.1)      # 10 Hz
    ws.send(json.dumps({"cmd": "stop"}))
finally:
    ws.close()               # rớt socket = động cơ dừng ngay (watchdog)
```

Các lệnh WS: `{"cmd":"drive","left":f,"right":f[,"ack":true]}`, `{"cmd":"stop"}`, `{"cmd":"status"}`, `{"cmd":"ping","t":n}`. Chỉ nhận khung trả lời khi có `"ack":true` (hoặc `status`/`ping`) — còn lại là fire-and-forget. Watchdog 1s vẫn áp dụng; **rớt socket → dừng động cơ ngay**.

---

## 8. Đo độ trễ mạng & chọn tần suất gửi lệnh

Muốn điều khiển mượt, cần biết **độ trễ khứ hồi (RTT)** giữa máy bạn và bot. Đơn giản nhất là bấm giờ một lệnh `/health`:

```python
import requests, time
BOT = "http://192.168.16.44:8100"
for _ in range(10):
    t0 = time.time()
    requests.get(f"{BOT}/health", timeout=1)
    print(f"RTT = {(time.time()-t0)*1000:.1f} ms")
```

Với WebSocket, dùng `ping` để đo chính xác hơn (không tính chi phí tạo kết nối):

```python
import websocket, json, time
ws = websocket.create_connection("ws://192.168.16.44:8100/ws")
t0 = time.time(); ws.send(json.dumps({"cmd":"ping","t":t0}))
print("pong sau", (time.time()-t0)*1000, "ms", ws.recv())
ws.close()
```

**Quy tắc chọn tần suất:** gửi lệnh với chu kỳ **nhỏ hơn nhiều so với 1s** (watchdog) nhưng đừng nhồi quá mức. RTT ~20–50ms trong WiFi tốt → 5–10 Hz là mượt và an toàn. Nếu RTT gần chạm 1s, mạng quá tệ để điều khiển thời gian thực — hãy lại gần router.

---

## 9. Điều khiển từ trình duyệt (JavaScript)

Vì agent bật **CORS mở**, bạn dựng được một bảng điều khiển web chỉ bằng một tệp HTML tĩnh — không cần backend:

```html
<!doctype html><meta charset="utf-8">
<img id="cam" width="320"><br>
<button onmousedown="go(.5,.5)"  onmouseup="stop()">▲ Tiến</button>
<button onmousedown="go(.4,-.4)" onmouseup="stop()">▶ Phải</button>
<button onmousedown="go(-.4,.4)" onmouseup="stop()">◀ Trái</button>
<button onmousedown="go(-.5,-.5)"onmouseup="stop()">▼ Lùi</button>
<script>
const BOT = "http://192.168.16.44:8100";
document.getElementById("cam").src = BOT + "/stream?w=320&fps=15";  // MJPEG trực tiếp
let timer = null;
function send(l, r){ fetch(BOT+"/drive",{method:"POST",
  headers:{"Content-Type":"application/json"},
  body:JSON.stringify({left:l,right:r})}); }
function go(l, r){ send(l,r); clearInterval(timer);
  timer = setInterval(()=>send(l,r), 200); }   // lặp 200ms để vỗ watchdog
function stop(){ clearInterval(timer); fetch(BOT+"/stop",{method:"POST"}); }
</script>
```

Giữ chuột trên nút để chạy, thả ra là dừng. Ảnh camera nhúng thẳng bằng thẻ `<img>` trỏ vào `/stream` (MJPEG).

---

## 10. Bàn phím điều khiển (teleop)

Lái bằng phím mũi tên từ máy tính (dùng thư viện `pynput`):

```python
# pip install pynput requests
from pynput import keyboard
import requests, threading, time
BOT = "http://192.168.16.44:8100"; lr = [0.0, 0.0]

keys = {'up':(.5,.5), 'down':(-.5,-.5), 'left':(-.4,.4), 'right':(.4,-.4)}
def on_press(k):
    n = getattr(k, 'name', None)
    if n in keys: lr[0], lr[1] = keys[n]
def on_release(k):
    lr[0] = lr[1] = 0.0
    if k == keyboard.Key.esc: return False   # ESC để thoát

def loop():
    while True:
        try: requests.post(f"{BOT}/drive", json={"left":lr[0],"right":lr[1]}, timeout=.5)
        except requests.RequestException: pass
        time.sleep(0.2)
threading.Thread(target=loop, daemon=True).start()
with keyboard.Listener(on_press=on_press, on_release=on_release) as L:
    L.join()
requests.post(f"{BOT}/stop")
```

---

## 11. Quay chính xác bằng IMU (điều khiển vòng kín)

Quay "theo thời gian" (`drive` rồi `sleep`) không chính xác vì pin/ma sát thay đổi. Với bot có **IMU (MPU-6050)**, hãy quay **theo góc thật** đọc từ `/imu`:

```python
import requests, time
BOT = "http://192.168.16.44:8100"

def heading():
    return requests.get(f"{BOT}/imu", timeout=0.5).json()["heading"]

def turn(deg, speed=0.35):
    requests.post(f"{BOT}/imu/zero", timeout=0.5)   # đặt gốc = 0 (bot đứng yên)
    time.sleep(0.2)
    sign = 1 if deg > 0 else -1
    while True:
        err = deg - heading()                       # còn thiếu bao nhiêu độ
        if abs(err) < 3: break                       # đủ gần thì dừng
        s = max(0.25, min(speed, abs(err)/90*speed)) # chậm dần khi tới đích
        requests.post(f"{BOT}/drive",
                      json={"left": sign*s, "right": -sign*s}, timeout=0.5)
        time.sleep(0.1)
    requests.post(f"{BOT}/stop", timeout=0.5)

turn(90)    # quay phải đúng ~90°
```

Đây là ví dụ **điều khiển vòng kín** (đọc cảm biến → tính sai số → điều chỉnh) — nền tảng cho mọi hành vi tự động chính xác.

---

## 12. Camera & cảm biến

```bash
# Chụp 1 ảnh (lật, thu nhỏ, chất lượng 80)
curl "http://192.168.16.44:8100/snapshot?flip=1&w=640&h=480&q=80" -o frame.jpg
```

- Xem luồng MJPEG trực tiếp: mở trong trình duyệt `http://192.168.16.44:8100/stream?w=640&fps=15`.
- Camera mở **lười** (chỉ khi có người xem) và **tự nhả** sau ~10s không ai xem — nên chụp/stream không ảnh hưởng điều khiển động cơ.
- IMU (bot có MPU-6050): `curl http://192.168.16.44:8100/imu` trả heading/rate/temp; `POST /imu/zero` để đặt lại gốc heading (bot đứng yên).

---

## 13. Gọi qua Fleet Server (proxy)

Khi không muốn theo dõi IP từng bot, gọi qua server (bot tự đăng ký IP/status lên server):

```bash
# Liệt kê bot đang online
curl http://<SERVER>/api/robots

# Điều khiển bot theo TÊN (server proxy tới bot đó)
curl -X POST http://<SERVER>/api/robots/bot2/drive -d '{"left":0.5,"right":0.5}'
curl        http://<SERVER>/api/robots/bot2/snapshot -o frame.jpg
curl        http://<SERVER>/api/robots/bot2/imu
```

Mọi ví dụ ở trên chỉ cần đổi `http://<IP>:8100` thành `http://<SERVER>/api/robots/{bot}` là chạy qua proxy — tiện khi IP bot hay đổi.

---

## 14. An toàn & xử lý sự cố

- **Kê bánh khỏi mặt đất** khi test lần đầu, để bot không lao khỏi bàn.
- **Luôn `try/finally` gọi `/stop`** ở cuối script; dựa thêm vào watchdog 1s làm lớp phòng hộ.
- **Bot tự dừng giữa chừng:** bình thường nếu bạn quên lặp `/drive` trong 1s. Nếu đang lặp mà vẫn dừng → kiểm tra độ trễ mạng (mỗi POST phải về < 1s).
- **Pin yếu gây brown-out:** dòng motor lớn làm sụt áp (~11V dưới tải) → Jetson reset, bot rớt WiFi và có thể nhận **IP DHCP mới**. **Sạc đầy trước khi test di chuyển.** Tìm lại bot bằng MAC (`ip neigh` / ping-sweep).
- **`motor.ok=false` trong `/status`:** Motor HAT không init được (thường do rail động cơ mất điện vì pin/HAT lỏng) — agent vẫn phục vụ telemetry/IMU/camera, lệnh drive thành no-op. Kiểm tra pin và cáp HAT.
- **Không kết nối được `:8100`:** bot có thể offline/brown-out; ping IP, kiểm tra container `jetbot_jupyter` còn chạy không.
- **Trình duyệt gọi được nhưng script không (hoặc ngược lại):** kiểm tra tường lửa/mạng — cả hai đều phải cùng LAN với bot; CORS đã mở sẵn nên không phải nguyên nhân.

---

## 15. Tổng kết

Điều khiển JetBot qua mạng gói gọn trong: `POST /drive {left,right}` với giá trị [-1..1], **nhớ lặp lệnh trong 1s** vì watchdog, và luôn `/stop` khi xong. Dùng **HTTP** cho lệnh rời rạc/script, **WebSocket** cho điều khiển tay tần suất cao, và **fleet server `/api/robots/{bot}/...`** khi muốn gọi theo tên thay vì IP. Đóng gói vào một **class giữ nhịp tự động**, đo **RTT** để chọn tần suất, và dùng **IMU vòng kín** khi cần quay chính xác. Camera và IMU tách biệt hoàn toàn với động cơ nên an toàn khi dùng đồng thời.
