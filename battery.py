#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
ĐO ĐIỆN ÁP & TÍNH PHẦN TRĂM (%) PIN JETBOT (INA219 3S LI-ION)
Tài liệu tham khảo: UIT AI Club - Hardware & IoT / JetBot Course (#31)
Hỗ trợ phần cứng: Waveshare JetBot (NVIDIA Jetson Nano) + Mạch cảm biến INA219
══════════════════════════════════════════════════════════════════════════════
Cách sử dụng:
  1. Chạy 1 lần xem thông số:
     python3 battery.py

  2. Chạy giám sát liên tục mỗi 1 giây (nhấn Ctrl+C để dừng):
     python3 battery.py --loop

  3. Import vào mã nguồn khác:
     from battery import read_battery_percentage, get_battery_info
     pct = read_battery_percentage()
══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import argparse

if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

# Kiểm tra thư viện SMBus
try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    try:
        from smbus import SMBus
        HAS_SMBUS = True
    except ImportError:
        HAS_SMBUS = False

# ── ĐẶC TUYẾN XẢ PIN 3S LI-ION (18650 x 3 Cells) ─────────────────────────────
# Điện áp đầy: 12.60V (4.20V / cell) -> 100%
# Điện áp cạn an toàn: 9.00V (3.00V / cell) -> 0%
LI_ION_3S_TABLE = [
    (12.60, 100),
    (12.40, 95),
    (12.20, 90),
    (12.00, 80),
    (11.80, 70),
    (11.60, 60),
    (11.40, 50),
    (11.20, 40),
    (11.00, 30),
    (10.80, 20),
    (10.50, 10),
    (10.00, 5),
    (9.00,  0)
]

def voltage_to_percentage_curve(voltage: float) -> int:
    """Chuyển đổi điện áp sang phần trăm theo đường đặc tuyến phi tuyến thực tế của pin Li-ion."""
    if voltage >= LI_ION_3S_TABLE[0][0]:
        return 100
    if voltage <= LI_ION_3S_TABLE[-1][0]:
        return 0
    for (v1, p1), (v2, p2) in zip(LI_ION_3S_TABLE, LI_ION_3S_TABLE[1:]):
        if v2 <= voltage <= v1:
            ratio = (voltage - v2) / (v1 - v2)
            return int(round(p2 + ratio * (p1 - p2)))
    return 0

def voltage_to_percentage_linear(voltage: float, v_min: float = 9.0, v_max: float = 12.6) -> float:
    """Chuyển đổi điện áp sang phần trăm theo công thức tuyến tính (Linear Formula)."""
    if voltage >= v_max:
        return 100.0
    if voltage <= v_min:
        return 0.0
    return round(((voltage - v_min) / (v_max - v_min)) * 100.0, 1)

class INA219Battery:
    """
    Trình điều khiển đọc cảm biến nguồn INA219 qua giao tiếp I2C.
    Tự động dò địa chỉ (0x41 trên Waveshare JetBot, hoặc 0x40 trên các bo mạch mở rộng khác).
    """
    # Thanh ghi INA219
    REG_CONFIG       = 0x00
    REG_SHUNTVOLTAGE = 0x01
    REG_BUSVOLTAGE   = 0x02
    REG_POWER        = 0x03
    REG_CURRENT      = 0x04
    REG_CALIBRATION  = 0x05

    def __init__(self, bus_num=1, addr=0x41, shunt_resistor=0.1):
        self.bus_num = bus_num
        self.addr = addr
        self.shunt_resistor = shunt_resistor
        self.detected_addr = None

    def find_device(self):
        """Tự động kiểm tra kết nối với INA219 tại địa chỉ 0x41 hoặc 0x40."""
        if not HAS_SMBUS:
            return None
        candidate_addrs = [self.addr, 0x41, 0x40, 0x42, 0x44]
        seen = set()
        for a in candidate_addrs:
            if a in seen:
                continue
            seen.add(a)
            try:
                with SMBus(self.bus_num) as bus:
                    bus.read_byte_data(a, self.REG_CONFIG)
                    self.detected_addr = a
                    return a
            except Exception:
                continue
        return None

    def read_raw(self):
        """
        Đọc trực tiếp các giá trị thô từ thanh ghi INA219 qua SMBus.
        Trả về dict: {voltage_v, shunt_mv, current_a, power_w}
        """
        if not HAS_SMBUS:
            raise RuntimeError(
                "Chưa cài đặt thư viện 'smbus2'. Hãy chạy: pip3 install smbus2"
            )

        target_addr = self.detected_addr or self.find_device() or self.addr

        try:
            with SMBus(self.bus_num) as bus:
                # 1. Đọc Bus Voltage (Thanh ghi 0x02, 2 bytes)
                raw_bus = bus.read_i2c_block_data(target_addr, self.REG_BUSVOLTAGE, 2)
                bus_raw = (raw_bus[0] << 8) | raw_bus[1]
                # LSB = 4mV, 3 bits cuối là cờ trạng thái (CNVR, OVF)
                bus_voltage_v = (bus_raw >> 3) * 0.004

                # 2. Đọc Shunt Voltage (Thanh ghi 0x01, 2 bytes)
                raw_shunt = bus.read_i2c_block_data(target_addr, self.REG_SHUNTVOLTAGE, 2)
                shunt_raw = (raw_shunt[0] << 8) | raw_shunt[1]
                if shunt_raw > 32767:
                    shunt_raw -= 65536
                # LSB = 10uV = 0.01mV
                shunt_voltage_mv = shunt_raw * 0.01

                # 3. Tính dòng điện (I = V / R)
                current_a = (shunt_voltage_mv / 1000.0) / self.shunt_resistor
                current_a = abs(current_a)

                # 4. Công suất (P = V * I)
                power_w = bus_voltage_v * current_a

                return {
                    "voltage_v": round(bus_voltage_v, 2),
                    "shunt_mv": round(shunt_voltage_mv, 2),
                    "current_a": round(current_a, 3),
                    "power_w": round(power_w, 2),
                    "addr": target_addr
                }
        except PermissionError:
            raise PermissionError(
                "Lỗi quyền truy cập cổng I2C! Hãy cấp quyền bằng lệnh:\n"
                "  sudo usermod -aG i2c $USER\n"
                "sau đó khởi động lại hoặc đăng xuất rồi đăng nhập lại."
            )
        except OSError as e:
            raise OSError(
                f"Không tìm thấy cảm biến INA219 trên I2C bus {self.bus_num}! ({e})\n"
                "Gợi ý kiểm tra:\n"
                "  1. Bật công tắc nguồn pin trên bo mạch JetBot sang ON.\n"
                "  2. Chạy lệnh: sudo i2cdetect -y -r 1 để kiểm tra địa chỉ 0x41."
            )

    def get_battery_status(self):
        """Trả về toàn bộ thông tin tình trạng pin dưới dạng Dictionary hoàn chỉnh."""
        raw = self.read_raw()
        v = raw["voltage_v"]
        pct_curve = voltage_to_percentage_curve(v)
        pct_linear = voltage_to_percentage_linear(v)
        cell_v = round(v / 3.0, 2)

        # Trạng thái an toàn của pin 3S
        if v >= 11.1:
            status = "BÌNH THƯỜNG / TỐT"
            color_code = "GREEN"
        elif v >= 10.2:
            status = "PIN YẾU (CẦN SẠC SỚM)"
            color_code = "YELLOW"
        else:
            status = "CỰC YẾU - NGUY HIỂM (HÃY TẮT ROBOT VÀ SẠC NGAY!)"
            color_code = "RED"

        # Ước lượng thời lượng sử dụng còn lại (giả định dung lượng pin 3S = 2600mAh)
        current_a = raw["current_a"]
        if current_a > 0.15:
            remaining_ah = 2.6 * (pct_curve / 100.0)
            remaining_minutes = int((remaining_ah / current_a) * 60)
        else:
            remaining_minutes = 180

        return {
            "voltage": v,
            "cell_voltage": cell_v,
            "percentage": pct_curve,
            "percentage_linear": pct_linear,
            "current_a": raw["current_a"],
            "power_w": raw["power_w"],
            "remaining_minutes": remaining_minutes,
            "status": status,
            "color_code": color_code,
            "i2c_addr": hex(raw["addr"])
        }

# ── HÀM TIỆN ÍCH DÀNH CHO CÁC MODULE KHÁC GỌI NHANH ───────────────────────────
def read_battery_percentage() -> int:
    """Hàm tiện ích: Trả về trực tiếp số nguyên % pin (0-100%)."""
    try:
        ina = INA219Battery()
        return ina.get_battery_status()["percentage"]
    except Exception:
        return 0

def get_battery_info() -> dict:
    """Hàm tiện ích: Trả về từ điển thông số pin đầy đủ."""
    ina = INA219Battery()
    return ina.get_battery_status()

# ── GIAO DIỆN TERMINAL ĐẸP MẮT ────────────────────────────────────────────────
def render_battery_bar(pct: int, length: int = 25) -> str:
    """Tạo thanh hiển thị trực quan dung lượng pin: [████████░░░░] 65%"""
    filled = int(round(length * (pct / 100.0)))
    bar = "█" * filled + "░" * (length - filled)
    return f"[{bar}] {pct}%"

def print_report(info: dict):
    os.system('cls' if os.name == 'nt' else 'clear')
    bar_str = render_battery_bar(info['percentage'])
    print("=" * 65)
    print("🔋 BÁO CÁO TÌNH TRẠNG PIN JETBOT (INA219 3S LI-ION)")
    print("   Trường ĐH Công Nghệ Thông Tin - ĐHQG-HCM (UIT AI Club)")
    print("=" * 65)
    print(f"  ├─ Địa chỉ I2C phát hiện : {info['i2c_addr']}")
    print(f"  ├─ Mức pin hiện tại      : {bar_str}")
    print(f"  ├─ Công thức phi tuyến   : {info['percentage']}% (Khuyên dùng cho Li-ion)")
    print(f"  ├─ Công thức tuyến tính  : {info['percentage_linear']}%")
    print(f"  ├─ Tổng điện áp pin (3S) : {info['voltage']} V")
    print(f"  ├─ Điện áp trung bình/Cell: {info['cell_voltage']} V / cell")
    print(f"  ├─ Dòng điện tiêu thụ    : {info['current_a']:.3f} A ({info['current_a']*1000:.1f} mA)")
    print(f"  ├─ Công suất tiêu thụ    : {info['power_w']} W")
    print(f"  ├─ Thời gian ước tính    : ~{info['remaining_minutes']} phút")
    print(f"  └─ Trạng thái pin        : {info['status']}")
    print("=" * 65)

def main():
    parser = argparse.ArgumentParser(description="Đo điện áp và % pin JetBot INA219 (UIT AI Club)")
    parser.add_argument("-l", "--loop", action="store_true", help="Chạy vòng lặp giám sát liên tục")
    parser.add_argument("-i", "--interval", type=float, default=1.0, help="Chu kỳ cập nhật khi chạy loop (giây, mặc định: 1.0s)")
    parser.add_argument("-b", "--bus", type=int, default=1, help="Số bus I2C (mặc định: 1)")
    parser.add_argument("-a", "--addr", type=lambda x: int(x, 0), default=0x41, help="Địa chỉ I2C hex (mặc định: 0x41)")
    parser.add_argument("--demo", action="store_true", help="Chạy chế độ thử nghiệm (mô phỏng khi không có phần cứng I2C)")
    args = parser.parse_args()

    # Kiểm tra chế độ demo
    if args.demo:
        print("💡 [DEMO MODE] Đang chạy với dữ liệu mô phỏng pin 12.1V...")
        v_mock = 12.10
        info_mock = {
            "voltage": v_mock,
            "cell_voltage": round(v_mock / 3.0, 2),
            "percentage": voltage_to_percentage_curve(v_mock),
            "percentage_linear": voltage_to_percentage_linear(v_mock),
            "current_a": 0.825,
            "power_w": round(v_mock * 0.825, 2),
            "remaining_minutes": 152,
            "status": "BÌNH THƯỜNG / TỐT",
            "color_code": "GREEN",
            "i2c_addr": "0x41 (DEMO)"
        }
        print_report(info_mock)
        return

    # Kiểm tra xem có thư viện SMBus chưa
    if not HAS_SMBUS:
        print("\n❌ [LỖI]: Chưa tìm thấy thư viện 'smbus2' hoặc 'smbus' trên hệ thống!")
        print("💡 Hãy chạy lệnh sau trên Jetson Nano để cài đặt:")
        print("   sudo apt-get update && sudo apt-get install -y python3-smbus i2c-tools")
        print("   pip3 install smbus2")
        print("\n(Để xem thử giao diện báo cáo mẫu trên máy tính, chạy: python3 battery.py --demo)\n")
        sys.exit(1)

    battery = INA219Battery(bus_num=args.bus, addr=args.addr)

    if args.loop:
        print("🔄 Đang khởi chạy chế độ giám sát liên tục... Nhấn Ctrl+C để thoát.")
        try:
            while True:
                info = battery.get_battery_status()
                print_report(info)
                print(f"⏳ Cập nhật mỗi {args.interval}s. Nhấn Ctrl+C để dừng.")
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n🛑 Đã dừng giám sát pin.")
    else:
        try:
            info = battery.get_battery_status()
            print_report(info)
        except Exception as e:
            print(f"\n❌ [LỖI ĐỌC PIN]: {e}")
            sys.exit(1)

if __name__ == '__main__':
    main()
