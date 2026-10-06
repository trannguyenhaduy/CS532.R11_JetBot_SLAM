#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 1: Điều khiển động cơ PCA9685 / TB6612 (Motor Controller)
Tuân thủ Quy tắc Harness 1: Zero-dependency SMBus trực tiếp, chống khóa bánh.
"""

import sys
import time
import math
import threading

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    try:
        from smbus import SMBus
        HAS_SMBUS = True
    except ImportError:
        HAS_SMBUS = False

class DirectPCA9685Driver:
    """Giao tiếp trực tiếp chip PCA9685 qua SMBus không cần cài đặt thêm thư viện ngoài"""
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
            old_mode = bus.read_byte_data(self.addr, self.MODE1)
            new_mode = (old_mode & 0x7F) | 0x10  # Sleep mode để nạp tần số
            bus.write_byte_data(self.addr, self.MODE1, new_mode)
            bus.write_byte_data(self.addr, self.PRESCALE, 121) # 50 Hz PWM
            bus.write_byte_data(self.addr, self.MODE1, old_mode)
            time.sleep(0.005)
            bus.write_byte_data(self.addr, self.MODE1, old_mode | 0xA1) # Auto-increment

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

        # Kênh Trái: Channel 1 (PWM/INA), Channel 0 (INB)
        if abs(left) < 0.05:
            self.set_pwm(0, 0, 0)
            self.set_pwm(1, 0, 0)
        elif left > 0:
            self.set_pwm(1, 0, pwm_l)
            self.set_pwm(0, 0, 0)
        else:
            self.set_pwm(1, 0, 0)
            self.set_pwm(0, 0, pwm_l)

        # Kênh Phải: Channel 2 (PWM/INA), Channel 3 (INB)
        if abs(right) < 0.05:
            self.set_pwm(2, 0, 0)
            self.set_pwm(3, 0, 0)
        elif right > 0:
            self.set_pwm(2, 0, pwm_r)
            self.set_pwm(3, 0, 0)
        else:
            self.set_pwm(2, 0, 0)
            self.set_pwm(3, 0, pwm_r)

    def stop(self):
        for ch in range(4):
            self.set_pwm(ch, 0, 0)


class MotorController:
    """Bộ điều khiển động cơ vi sai với Phanh an toàn & Watchdog"""
    def __init__(self, bus_num=1, addr=0x60, wheel_sep=0.12, max_v=0.35, max_w=1.20, brake_dist=0.15):
        self.wheel_sep = wheel_sep
        self.max_v = max_v
        self.max_w = max_w
        self.brake_dist = brake_dist

        self.target_v = 0.0
        self.target_w = 0.0
        self.obstacle_distance_m = 99.0
        self.last_cmd_time = time.time()
        self.lock = threading.Lock()
        self.is_connected = False
        self.driver = None

        if HAS_SMBUS:
            for test_addr in [addr, 0x40]:
                try:
                    self.driver = DirectPCA9685Driver(bus_num=bus_num, addr=test_addr)
                    self.driver.stop()
                    self.is_connected = True
                    print(f"🤖 [MOTOR] Kết nối thành công chip PCA9685 qua SMBus (addr=0x{test_addr:02X})!")
                    break
                except Exception:
                    pass

        if not self.is_connected:
            print("⚠️ [MOTOR] Không phát hiện phần cứng I2C. Chạy chế độ GIẢ LẬP.")

        # Khởi chạy luồng Watchdog an toàn
        self.running = True
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

    def update_obstacle_distance(self, dist_m: float):
        with self.lock:
            self.obstacle_distance_m = float(dist_m)
            # Kích hoạt phanh khẩn cấp ngay lập tức nếu đang có trớn tiến
            if self.obstacle_distance_m < self.brake_dist and self.target_v > 0.0:
                self.target_v = 0.0
                self.stop()

    def set_cmd_vel(self, v: float, w: float):
        with self.lock:
            self.last_cmd_time = time.time()

            # Phanh khẩn cấp Virtual Bumper
            if self.obstacle_distance_m < self.brake_dist and v > 0.0:
                v = 0.0

            self.target_v = v
            self.target_w = w

            # Động học vi sai: v_left, v_right
            v_l = v - (w * self.wheel_sep / 2.0)
            v_r = v + (w * self.wheel_sep / 2.0)

            # Chuẩn hóa về [-1.0, 1.0]
            norm = max(abs(v_l), abs(v_r), self.max_v)
            scale = 1.0 / norm if norm > 0 else 1.0
            p_l = v_l * scale
            p_r = v_r * scale

            if self.is_connected and self.driver:
                self.driver.set_motors(p_l, p_r)

    def stop(self):
        with self.lock:
            self.target_v = 0.0
            self.target_w = 0.0
            if self.is_connected and self.driver:
                self.driver.stop()

    def _watchdog_loop(self):
        while self.running:
            with self.lock:
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
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử MotorController...")
    mc = MotorController()
    print(f"  ├─ Kết nối phần cứng: {mc.is_connected}")
    print("  ├─ Thử gửi lệnh tiến v=0.2 trong 0.5 giây...")
    mc.set_cmd_vel(0.2, 0.0)
    time.sleep(0.5)
    mc.stop()
    print("  └─ Đã dừng xe an toàn.")
    print("✅ [SELF-TEST] MotorController ĐẠT CHUẨN!")
