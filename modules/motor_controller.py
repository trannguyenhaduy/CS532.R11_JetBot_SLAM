#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 1: Điều khiển động cơ PCA9685 / TB6612 (Motor Controller)
Tương thích 100% phần cứng Waveshare JetBot:
- Hỗ trợ đa tầng: Adafruit_MotorHAT -> Direct SMBus (1600Hz, full pinout) -> jetbot.Robot
- Tích hợp Phanh an toàn Virtual Bumper & Watchdog 0.5s chống trôi xe.
"""

import sys
import time
import math
import threading

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

# 1. Kiểm tra SMBus
try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    try:
        from smbus import SMBus
        HAS_SMBUS = True
    except ImportError:
        HAS_SMBUS = False

# 2. Kiểm tra Adafruit_MotorHAT
try:
    from Adafruit_MotorHAT import Adafruit_MotorHAT
    HAS_ADAFRUIT = True
except ImportError:
    HAS_ADAFRUIT = False

# 3. Kiểm tra jetbot.Robot
try:
    from jetbot import Robot
    HAS_JETBOT_LIB = True
except ImportError:
    HAS_JETBOT_LIB = False


class AdafruitWaveshareDriver:
    """Driver chuẩn chính thức của bo mạch mở rộng Waveshare JetBot qua Adafruit_MotorHAT"""
    def __init__(self, addr=0x60, i2c_bus=1):
        self._hat_api = Adafruit_MotorHAT
        self._hat = Adafruit_MotorHAT(addr=addr, i2c_bus=i2c_bus)
        self._m_left = self._hat.getMotor(1)
        self._m_right = self._hat.getMotor(2)
        self.stop()

    def _drive_one(self, motor, val: float, ina: int, inb: int):
        val = max(-1.0, min(1.0, float(val)))
        mapped = int(255.0 * val)
        speed = min(max(abs(mapped), 0), 255)
        motor.setSpeed(speed)
        if speed < 15:
            motor.run(self._hat_api.RELEASE)
            self._hat._pwm.setPWM(ina, 0, 0)
            self._hat._pwm.setPWM(inb, 0, 0)
        elif mapped < 0:
            motor.run(self._hat_api.FORWARD)
            self._hat._pwm.setPWM(ina, 0, 0)
            self._hat._pwm.setPWM(inb, 0, speed * 16)
        else:
            motor.run(self._hat_api.BACKWARD)
            self._hat._pwm.setPWM(ina, 0, speed * 16)
            self._hat._pwm.setPWM(inb, 0, 0)

    def set_motors(self, left: float, right: float):
        self._drive_one(self._m_left, left, ina=1, inb=0)
        self._drive_one(self._m_right, right, ina=2, inb=3)

    def stop(self):
        for motor, ina, inb in ((self._m_left, 1, 0), (self._m_right, 2, 3)):
            if motor:
                motor.run(self._hat_api.RELEASE)
            if self._hat and hasattr(self._hat, '_pwm'):
                self._hat._pwm.setPWM(ina, 0, 0)
                self._hat._pwm.setPWM(inb, 0, 0)


class DirectPCA9685Driver:
    """Giao tiếp trực tiếp thanh ghi PCA9685 qua SMBus với tần số 1600Hz và kích hoạt đầy đủ chân TB6612"""
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus_num=1, addr=0x60):
        self.bus_num = bus_num
        self.addr = addr
        self._init_chip()

    def _init_chip(self):
        with SMBus(self.bus_num) as bus:
            bus.write_byte_data(self.addr, self.MODE1, 0x00)
            time.sleep(0.005)
            # Đưa chip vào Sleep mode để nạp tần số PWM ~1600 Hz (Prescale = 3) cho động cơ DC
            old_mode = bus.read_byte_data(self.addr, self.MODE1)
            new_mode = (old_mode & 0x7F) | 0x10
            bus.write_byte_data(self.addr, self.MODE1, new_mode)
            bus.write_byte_data(self.addr, self.PRESCALE, 3) # 1600 Hz
            bus.write_byte_data(self.addr, self.MODE1, old_mode)
            time.sleep(0.005)
            bus.write_byte_data(self.addr, self.MODE1, old_mode | 0xA1) # Auto-increment + restart

    def set_pwm(self, channel, on, off):
        reg = self.LED0_ON_L + 4 * channel
        data = [int(on) & 0xFF, (int(on) >> 8) & 0xFF, int(off) & 0xFF, (int(off) >> 8) & 0xFF]
        with SMBus(self.bus_num) as bus:
            bus.write_i2c_block_data(self.addr, reg, data)

    def set_motors(self, left: float, right: float):
        left = max(-1.0, min(1.0, float(left)))
        right = max(-1.0, min(1.0, float(right)))

        pwm_l = int(abs(left) * 4095)
        pwm_r = int(abs(right) * 4095)

        # ── KÊNH TRÁI (LEFT MOTOR) ──
        # Kích cả Channel 8 (PWMA), Channel 1 (INA), Channel 0 (INB) lẫn Channel 9, 10
        if abs(left) < 0.05:
            for ch in [0, 1, 8, 9, 10]: self.set_pwm(ch, 0, 0)
        elif left > 0:
            self.set_pwm(8, 0, pwm_l)      # PWMA (Speed)
            self.set_pwm(1, 0, pwm_l)      # INA
            self.set_pwm(0, 0, 0)          # INB
            self.set_pwm(10, 0, 4095)      # IN1
            self.set_pwm(9, 0, 0)          # IN2
            self.set_pwm(0, 0, pwm_l)      # Dự phòng revision 2 (Ch0=PWMA)
            self.set_pwm(2, 0, 0)
        else:
            self.set_pwm(8, 0, pwm_l)      # PWMA (Speed)
            self.set_pwm(1, 0, 0)          # INA
            self.set_pwm(0, 0, pwm_l)      # INB
            self.set_pwm(10, 0, 0)         # IN1
            self.set_pwm(9, 0, 4095)       # IN2
            self.set_pwm(0, 0, pwm_l)      # Dự phòng revision 2
            self.set_pwm(2, 0, 4095)

        # ── KÊNH PHẢI (RIGHT MOTOR) ──
        # Kích cả Channel 13 (PWMB), Channel 2 (INA), Channel 3 (INB) lẫn Channel 11, 12
        if abs(right) < 0.05:
            for ch in [2, 3, 5, 11, 12, 13]: self.set_pwm(ch, 0, 0)
        elif right > 0:
            self.set_pwm(13, 0, pwm_r)     # PWMB (Speed)
            self.set_pwm(2, 0, pwm_r)      # INA
            self.set_pwm(3, 0, 0)          # INB
            self.set_pwm(11, 0, 4095)      # IN1
            self.set_pwm(12, 0, 0)         # IN2
            self.set_pwm(5, 0, pwm_r)      # Dự phòng revision 2 (Ch5=PWMB)
            self.set_pwm(4, 0, 0)
        else:
            self.set_pwm(13, 0, pwm_r)     # PWMB (Speed)
            self.set_pwm(2, 0, 0)          # INA
            self.set_pwm(3, 0, pwm_r)      # INB
            self.set_pwm(11, 0, 0)         # IN1
            self.set_pwm(12, 0, 4095)      # IN2
            self.set_pwm(5, 0, pwm_r)      # Dự phòng revision 2
            self.set_pwm(4, 0, 4095)

    def stop(self):
        for ch in [0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13]:
            self.set_pwm(ch, 0, 0)


class JetBotLibDriver:
    """Driver thông qua thư viện NVIDIA Jetbot chính thức"""
    def __init__(self):
        self._robot = Robot()
        self.stop()

    def set_motors(self, left: float, right: float):
        self._robot.set_motors(float(left), float(right))

    def stop(self):
        self._robot.stop()


class MotorController:
    """Bộ điều khiển động cơ vi sai với Phanh an toàn, Watchdog & Đảo kênh chuẩn Waveshare"""
    def __init__(self, bus_num=1, addr=0x60, wheel_sep=0.12, max_v=0.35, max_w=1.20, brake_dist=0.35,
                 swap_motors=False, invert_linear=False, invert_left=False, invert_right=False,
                 enable_brake=True):
        self.wheel_sep = wheel_sep
        self.max_v = max_v
        self.max_w = max_w
        self.brake_dist = brake_dist
        self.swap_motors = swap_motors
        self.invert_linear = invert_linear
        self.invert_left = invert_left
        self.invert_right = invert_right
        self.enable_brake = enable_brake

        self.target_v = 0.0
        self.target_w = 0.0
        self.obstacle_distance_m = 99.0
        self.last_cmd_time = time.time()
        self.last_log_time = 0.0
        self.lock = threading.Lock()
        self.is_connected = False
        self.driver_name = "SIMULATOR"
        self.driver = None

        # ── ƯU TIÊN 1: Thử Adafruit_MotorHAT (Driver tương thích chuẩn nhất của Waveshare) ──
        if HAS_ADAFRUIT:
            try:
                drv = AdafruitWaveshareDriver(addr=addr, i2c_bus=bus_num)
                self.driver = drv
                self.driver_name = "Adafruit_MotorHAT"
                self.is_connected = True
                print(f"🤖 [MOTOR] Đã kết nối phần cứng qua 'Adafruit_MotorHAT' (addr=0x{addr:02X}, bus={bus_num})!")
            except Exception as e:
                print(f"ℹ️ [MOTOR] Không dùng Adafruit_MotorHAT: {e}")

        # ── ƯU TIÊN 2: Thử Direct SMBus (1600Hz & Full Channel Mapping) ──
        if not self.is_connected and HAS_SMBUS:
            for test_addr in [addr, 0x40]:
                try:
                    drv = DirectPCA9685Driver(bus_num=bus_num, addr=test_addr)
                    drv.stop()
                    self.driver = drv
                    self.driver_name = f"Direct_SMBus_0x{test_addr:02X}"
                    self.is_connected = True
                    print(f"🤖 [MOTOR] Đã kết nối phần cứng chip PCA9685 qua SMBus trực tiếp (addr=0x{test_addr:02X})!")
                    break
                except Exception as e:
                    print(f"ℹ️ [MOTOR] SMBus addr=0x{test_addr:02X} không phản hồi: {e}")

        # ── ƯU TIÊN 3: Thử jetbot.Robot ──
        if not self.is_connected and HAS_JETBOT_LIB:
            try:
                drv = JetBotLibDriver()
                self.driver = drv
                self.driver_name = "jetbot.Robot"
                self.is_connected = True
                print("🤖 [MOTOR] Đã kết nối phần cứng qua thư viện 'jetbot.Robot'!")
            except Exception as e:
                print(f"ℹ️ [MOTOR] jetbot.Robot không khởi tạo được: {e}")

        if not self.is_connected:
            print("⚠️ [MOTOR] Không phát hiện phần cứng I2C động cơ. Chạy chế độ GIẢ LẬP.")

        # Khởi chạy luồng Watchdog an toàn (0.5s)
        self.running = True
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

    def update_obstacle_distance(self, dist_m: float):
        with self.lock:
            self.obstacle_distance_m = float(dist_m)
            if self.enable_brake and self.obstacle_distance_m < self.brake_dist and self.target_v > 0.0:
                self.target_v = 0.0
                self.stop()

    def set_cmd_vel(self, v: float, w: float):
        with self.lock:
            self.last_cmd_time = time.time()
            if self.enable_brake and self.obstacle_distance_m < self.brake_dist and v > 0.0:
                print(f"🛑 [MOTOR PHANH] Khóa tiến do cản ở {self.obstacle_distance_m:.2f}m (< {self.brake_dist:.2f}m)!")
                v = 0.0

            self.target_v = v
            self.target_w = w

            actual_v = -v if self.invert_linear else v
            actual_w = w

            # Giải động học vi sai (Differential Drive Kinematics)
            v_l = actual_v - (actual_w * self.wheel_sep / 2.0)
            v_r = actual_v + (actual_w * self.wheel_sep / 2.0)

            # Quy đổi từ vận tốc m/s sang tỉ lệ PWM [-1.0, 1.0] (Tốc độ tối đa JetBot ~0.55 m/s)
            HW_MAX_V = 0.55
            p_l = v_l / HW_MAX_V
            p_r = v_r / HW_MAX_V

            # Giới hạn trần tốc độ tối đa 35% PWM để xe chạy đầm, phanh kịp thời và quay không văng
            MAX_DUTY = 0.35
            p_l = max(-MAX_DUTY, min(MAX_DUTY, p_l))
            p_r = max(-MAX_DUTY, min(MAX_DUTY, p_r))

            # Bù lực ma sát tối thiểu (Deadband boost nhẹ) để bánh xe lăn êm
            if abs(p_l) > 0.03 and abs(p_l) < 0.18:
                p_l = 0.18 if p_l > 0 else -0.18
            if abs(p_r) > 0.03 and abs(p_r) < 0.18:
                p_r = 0.18 if p_r > 0 else -0.18

            # Đảo cực tính động cơ nếu cần
            if self.invert_left: p_l = -p_l
            if self.invert_right: p_r = -p_r

            # Đảo 2 kênh Trái <-> Phải (Waveshare M1=Phải, M2=Trái) để sửa lỗi quay ngược
            if self.swap_motors:
                p_l, p_r = p_r, p_l

            now = time.time()
            if (abs(v) > 0.01 or abs(w) > 0.01) and (now - self.last_log_time > 0.3):
                self.last_log_time = now
                print(f"🕹️ [MOTOR] Lệnh: v={v:.2f}, w={w:.2f} -> L={p_l:.2f}, R={p_r:.2f} [{self.driver_name}]")

            if self.is_connected and self.driver:
                self.driver.set_motors(p_l, p_r)

    def set_direct_motors(self, left: float, right: float):
        """Cho phép đặt tốc độ trực tiếp từng bánh [-1.0, 1.0] để kiểm tra hoặc bù lệch bánh"""
        with self.lock:
            self.last_cmd_time = time.time()
            if self.invert_linear:
                left, right = -left, -right
            if self.invert_left: left = -left
            if self.invert_right: right = -right
            if self.swap_motors:
                left, right = right, left
            if self.is_connected and self.driver:
                self.driver.set_motors(left, right)

    def stop(self):
        with self.lock:
            self.target_v = 0.0
            self.target_w = 0.0
            if self.is_connected and self.driver:
                self.driver.stop()

    def _watchdog_loop(self):
        while self.running:
            with self.lock:
                # 1. Tự động ngắt khẩn cấp nếu có vật cản trước mặt khi xe đang chạy tiến
                if self.target_v > 0.0 and self.obstacle_distance_m < self.brake_dist:
                    self.target_v = 0.0
                    self.target_w = 0.0
                    if self.is_connected and self.driver:
                        self.driver.stop()

                # 2. Watchdog ngắt động cơ nếu mất kết nối lái tay quá 0.5s
                elapsed = time.time() - self.last_cmd_time
                if elapsed > 0.5 and (abs(self.target_v) > 0.01 or abs(self.target_w) > 0.01):
                    self.target_v = 0.0
                    self.target_w = 0.0
                    if self.is_connected and self.driver:
                        self.driver.stop()
            time.sleep(0.05)

    def shutdown(self):
        self.running = False
        self.stop()


if __name__ == '__main__':
    print("\n" + "═" * 70)
    print("🧪 [SELF-TEST] BẮT ĐẦU KIỂM THỬ ĐỘNG CƠ JETBOT (MOTOR CONTROLLER)")
    print("═" * 70)
    mc = MotorController()
    print(f"  ├─ Kết nối phần cứng: {'THÀNH CÔNG' if mc.is_connected else 'GIẢ LẬP'}")
    print(f"  ├─ Driver điều khiển: {mc.driver_name}")
    print("  ├─ Đang gửi lệnh chạy thử 2 bánh (Tốc độ 50% trong 1.5 giây)...")
    
    # Gửi lệnh quay trực tiếp 50% lực trong 1.5s
    mc.set_direct_motors(0.5, 0.5)
    time.sleep(1.5)
    mc.stop()
    
    print("  └─ Đã dừng xe an toàn.")
    print("═" * 70)
    print("📌 [CHẨN ĐOÁN PHẦN CỨNG NẾU BÁNH XE KHÔNG QUAY]:")
    print("  1. CÔNG TẮC NGUỒN PIN: Đã gạt sang ON trên bo mạch mở rộng JetBot chưa?")
    print("     (Lưu ý: Cắm sạc/USB chỉ nuôi Jetson Nano, động cơ CẦN BẬT CÔNG TẮC PIN 3S)")
    print("  2. ĐÈN LED NGUỒN: Đèn xanh trên bo mạch motor phía dưới có sáng không?")
    print("  3. ĐIỆN ÁP PIN: Đo thử xem pin có bị sụt dưới 9.5V không (pin cạn motor sẽ đứng yên).")
    print("═" * 70 + "\n")
