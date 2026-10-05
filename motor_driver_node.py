#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
JETBOT MOTOR DRIVER & SAFETY WATCHDOG NODE (ROS MELODIC / NOETIC)
Thành viên 1 — Cụm Điều khiển Động cơ & An toàn Phần cứng
══════════════════════════════════════════════════════════════════════════════
- Hợp đồng Topic nhận:
  * /cmd_vel (geometry_msgs/Twist): Vận tốc mong muốn (linear.x, angular.z)
  * /obstacle_distance (std_msgs/Float32): Khoảng cách vật cản phía trước (mét)
- Hợp đồng Topic xuất:
  * /battery_telemetry (std_msgs/Float32MultiArray): [V, %, Current(A), Power(W), Rem(min)]
- Tính năng an toàn:
  * Phanh khẩn cấp Virtual Bumper: Chặn tiến khi vật cản < 0.25m
  * Watchdog Timer 0.5s: Tự động ngắt động cơ khi mất tín hiệu
  * Lái vi sai chuẩn xác cho mạch Waveshare TB6612 (I2C addr 0x60)
══════════════════════════════════════════════════════════════════════════════
"""

import time
import math
import threading

try:
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32, Float32MultiArray
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Twist = object
    Float32 = object
    Float32MultiArray = object

try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    try:
        from smbus import SMBus
        HAS_SMBUS = True
    except ImportError:
        HAS_SMBUS = False

# ─── LỚP ĐIỀU KHIỂN ĐỘNG CƠ WAVESHARE TB6612 (BYPASS LỖI TORCH) ──────────────
class WaveshareMotorHAT:
    def __init__(self, addr=0x60, i2c_bus=1):
        try:
            from Adafruit_MotorHAT import Adafruit_MotorHAT
            self._hat_api = Adafruit_MotorHAT
            self._driver = Adafruit_MotorHAT(addr=addr, i2c_bus=i2c_bus)
            self._left = self._driver.getMotor(1)
            self._right = self._driver.getMotor(2)
            self._pins = ((1, 0), (2, 3)) # Left: ina=1, inb=0 | Right: ina=2, inb=3
            self.stop()
            self.is_connected = True
            print("🤖 [MOTOR] Đã kết nối trực tiếp mạch Waveshare Motor HAT (0x60)!")
        except Exception as e:
            try:
                from jetbot import Robot
                self._jetbot_robot = Robot()
                self._hat_api = None
                self.is_connected = True
                print("🤖 [MOTOR] Đã kết nối qua thư viện JetBot mặc định!")
            except Exception as e2:
                print("⚠️ [MOTOR] Không phát hiện phần cứng động cơ. Chạy chế độ GIẢ LẬP.")
                self.is_connected = False
                self._hat_api = None

    def _set_one(self, motor, pins, value):
        if not self.is_connected or self._hat_api is None: return
        value = max(-1.0, min(1.0, float(value)))
        mapped = int(255.0 * value)
        speed = min(max(abs(mapped), 0), 255)
        motor.setSpeed(speed)
        ina, inb = pins
        if mapped < 0:
            motor.run(self._hat_api.FORWARD)
            self._driver._pwm.setPWM(ina, 0, 0)
            self._driver._pwm.setPWM(inb, 0, speed * 16)
        else:
            motor.run(self._hat_api.BACKWARD)
            self._driver._pwm.setPWM(ina, 0, speed * 16)
            self._driver._pwm.setPWM(inb, 0, 0)

    def set_motors(self, left, right):
        if not self.is_connected: return
        if self._hat_api is not None:
            self._set_one(self._left, self._pins[0], left)
            self._set_one(self._right, self._pins[1], right)
        elif hasattr(self, '_jetbot_robot'):
            self._jetbot_robot.set_motors(left, right)

    def stop(self):
        if not self.is_connected: return
        if self._hat_api is not None:
            for motor, pins in ((self._left, self._pins[0]), (self._right, self._pins[1])):
                motor.run(self._hat_api.RELEASE)
                self._driver._pwm.setPWM(pins[0], 0, 0)
                self._driver._pwm.setPWM(pins[1], 0, 0)
        elif hasattr(self, '_jetbot_robot'):
            self._jetbot_robot.stop()

# ─── ĐỌC VÀ TÍNH TOÁN PIN THỜI GIAN THỰC (INA219 3S LI-ION) ───────────────────
_LI_ION_CURVE_3S = [
    (12.60, 100), (12.30, 90), (12.00, 80), (11.70, 70), (11.40, 60),
    (11.10, 50),  (10.80, 35), (10.50, 20), (10.20, 10), (9.60, 5), (9.00, 0)
]

def calculate_battery_metrics():
    if not HAS_SMBUS:
        return 12.0, 85, 0.85, 10.2, 150
    try:
        with SMBus(1) as bus:
            # 1. Đọc điện áp Bus Voltage (thanh ghi 0x02)
            raw_bus = bus.read_i2c_block_data(0x41, 0x02, 2)
            v = (((raw_bus[0] << 8) | raw_bus[1]) >> 3) * 0.004

            # 2. Đọc dòng điện qua Shunt Voltage (thanh ghi 0x01)
            raw_shunt = bus.read_i2c_block_data(0x41, 0x01, 2)
            shunt_raw = (raw_shunt[0] << 8) | raw_shunt[1]
            if shunt_raw > 32767: shunt_raw -= 65536
            curr_a = max(0.05, abs((shunt_raw * 0.00001) / 0.1))

            # 3. Tính % dung lượng theo đường cong Li-ion 3S phi tuyến
            pct = 0
            if v >= _LI_ION_CURVE_3S[0][0]: pct = 100
            elif v <= _LI_ION_CURVE_3S[-1][0]: pct = 0
            else:
                for (v1, p1), (v2, p2) in zip(_LI_ION_CURVE_3S, _LI_ION_CURVE_3S[1:]):
                    if v2 <= v <= v1:
                        pct = int(round(p2 + (p1 - p2) * (v - v2) / (v1 - v2)))
                        break
            
            power_w = round(v * curr_a, 2)
            rem_ah = 2.6 * (pct / 100.0)
            rem_min = int((rem_ah / curr_a) * 60) if curr_a > 0.15 else 240
            return round(v, 2), pct, round(curr_a, 2), power_w, rem_min
    except Exception:
        return 12.0, 85, 0.85, 10.2, 150

# ─── MOTOR DRIVER ROS NODE ───────────────────────────────────────────────────
class JetBotMotorDriverNode:
    def __init__(self):
        self.motor_hat = WaveshareMotorHAT()
        self.last_cmd_time = time.time()
        self.obstacle_distance_m = 99.0
        self.safety_brake_dist_m = 0.25 # Ngưỡng phanh Virtual Bumper 25cm
        self.wheel_separation_m = 0.12 # Khoảng cách 2 bánh JetBot
        self.max_linear_speed = 0.35   # m/s
        self.max_angular_speed = 1.2   # rad/s

        self.target_v = 0.0
        self.target_w = 0.0
        self.lock = threading.Lock()

        if HAS_ROS:
            rospy.init_node('motor_driver', anonymous=False)
            self.sub_cmd = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb, queue_size=1)
            self.sub_obs = rospy.Subscriber('/obstacle_distance', Float32, self.obstacle_cb, queue_size=1)
            self.pub_batt = rospy.Publisher('/battery_telemetry', Float32MultiArray, queue_size=1)
            rospy.loginfo("🚀 [MOTOR] Node motor_driver_node đã khởi chạy sẵn sàng nhận lệnh /cmd_vel")

        # Khởi động luồng Watchdog và Luồng đo Pin
        self.running = True
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        self.battery_thread = threading.Thread(target=self._battery_loop, daemon=True)
        self.battery_thread.start()

    def obstacle_cb(self, msg: Float32):
        with self.lock:
            self.obstacle_distance_m = float(msg.data)
            # Kích hoạt phanh khẩn cấp ngay lập tức nếu đang có trớn tiến
            if self.obstacle_distance_m < self.safety_brake_dist_m and self.target_v > 0.0:
                self.target_v = 0.0
                self.motor_hat.stop()
                if HAS_ROS:
                    rospy.logwarn_throttle(1.0, f"🚨 [PHANH KHẨN CẤP] Vật cản cách {self.obstacle_distance_m:.2f}m (< 0.25m). Ngắt truyền động tiến!")

    def cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self.last_cmd_time = time.time()
            v = msg.linear.x
            w = msg.angular.z

            # BẢO VỆ AN TOÀN: Nếu phía trước có vật cản gần < 25cm, triệt tiêu lệnh tiến
            if self.obstacle_distance_m < self.safety_brake_dist_m and v > 0.0:
                v = 0.0
                if HAS_ROS:
                    rospy.logwarn_throttle(1.0, "⚠️ [VIRTUAL BUMPER] Chặn lệnh tiến do quá sát vật cản! Chỉ cho phép lùi/quay.")

            self.target_v = v
            self.target_w = w

            # Đổi động học vi sai (Differential Drive Kinematics)
            # v_left  = v - (w * L / 2)
            # v_right = v + (w * L / 2)
            v_l = v - (w * self.wheel_separation_m / 2.0)
            v_r = v + (w * self.wheel_separation_m / 2.0)

            # Chuẩn hóa về dải [-1.0, 1.0] cho Motor HAT
            norm_factor = max(abs(v_l), abs(v_r), self.max_linear_speed)
            scale = 1.0 / norm_factor if norm_factor > 0 else 1.0

            p_l = v_l * scale
            p_r = v_r * scale

            self.motor_hat.set_motors(p_l, p_r)

    def _watchdog_loop(self):
        """Watchdog 0.5s: Tự động phanh dừng xe nếu mất kết nối hoặc không có lệnh mới"""
        rate = 20 # 20 Hz
        while self.running and (not HAS_ROS or not rospy.is_shutdown()):
            with self.lock:
                elapsed = time.time() - self.last_cmd_time
                if elapsed > 0.5 and (abs(self.target_v) > 0.01 or abs(self.target_w) > 0.01):
                    self.target_v = 0.0
                    self.target_w = 0.0
                    self.motor_hat.stop()
            time.sleep(1.0 / rate)

    def _battery_loop(self):
        """Luồng phát telemetry pin 1 Hz lên ROS topic /battery_telemetry"""
        while self.running and (not HAS_ROS or not rospy.is_shutdown()):
            try:
                v, pct, curr, p_w, rem_m = calculate_battery_metrics()
                if HAS_ROS and hasattr(self, 'pub_batt'):
                    msg = Float32MultiArray()
                    msg.data = [float(v), float(pct), float(curr), float(p_w), float(rem_m)]
                    self.pub_batt.publish(msg)
            except Exception: pass
            time.sleep(1.0)

    def shutdown(self):
        self.running = False
        self.motor_hat.stop()
        print("🛑 [MOTOR] Đã dừng động cơ an toàn và ngắt node.")

def main():
    driver = JetBotMotorDriverNode()
    if HAS_ROS:
        rospy.on_shutdown(driver.shutdown)
        rospy.spin()
    else:
        print("Đang chạy kiểm thử motor_driver_node không có ROS. Nhấn Ctrl+C để thoát.")
        try:
            while True: time.sleep(1.0)
        except KeyboardInterrupt:
            driver.shutdown()

if __name__ == '__main__':
    main()
