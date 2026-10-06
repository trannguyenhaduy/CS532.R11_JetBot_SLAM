#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
JETBOT PERSON FOLLOWER PD CONTROLLER (HRI MODULE)
Thành viên 1 — Tương tác Người - Máy (HRI) & Bám đối tượng
══════════════════════════════════════════════════════════════════════════════
- Hợp đồng Topic nhận:
  * /spatial_objects (std_msgs/String): Mảng JSON tọa độ 3D từ Thành viên 2 (OAK-D VPU)
  * /mode/person_following (std_msgs/Bool): Bật/Tắt tính năng bám người từ Web
- Hợp đồng Topic xuất:
  * /cmd_vel (geometry_msgs/Twist): Lệnh điều khiển lái xe gửi xuống motor_driver_node
- Thuật toán:
  * Bộ điều khiển PD 2 bậc tự do (Khoảng cách Z và Góc xoay X)
  * Vùng chết (Deadzone) chống giật cục: |e_z| < 0.08m, |e_x| < 0.06m
  * Giới hạn vận tốc an toàn: v in [-0.15, 0.25] m/s, w in [-0.8, 0.8] rad/s
  * Tự phanh dừng sau 0.8s nếu mất dấu người
══════════════════════════════════════════════════════════════════════════════
"""

import time
import json
import math

try:
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Bool
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Twist = object
    String = object
    Bool = object

class JetBotPersonFollowerNode:
    def __init__(self):
        # Tham số mục tiêu & Hệ số PD
        self.target_dist_z = 0.85       # Duy trì cự ly cách người 0.85m
        self.kp_dist = 0.45             # Hệ số tỷ lệ khoảng cách tiến/lùi
        self.kd_dist = 0.08             # Hệ số vi phân khoảng cách
        self.kp_angle = 1.30            # Hệ số tỷ lệ góc xoay căn giữa
        self.kd_angle = 0.15            # Hệ số vi phân góc xoay

        # Giới hạn vận tốc an toàn
        self.max_v = 0.25               # m/s
        self.min_v = -0.15              # m/s (cho phép lùi nhẹ nếu người bước tới quá sát)
        self.max_w = 0.80               # rad/s

        # Vùng chết (Deadzone) để xe không bị rung lắc khi đã đúng vị trí
        self.deadzone_dist = 0.08       # 8 cm
        self.deadzone_angle = 0.06      # 6 cm

        # Trạng thái theo dõi
        self.is_enabled = False         # Mặc định TẮT (để ưu tiên chế độ lái tay từ Web)
        self.last_person_time = 0.0
        self.last_ez = 0.0
        self.last_ex = 0.0
        self.last_ctrl_time = time.time()

        if HAS_ROS:
            rospy.init_node('person_follower', anonymous=False)
            self.sub_objects = rospy.Subscriber('/spatial_objects', String, self.objects_cb, queue_size=1)
            self.sub_mode = rospy.Subscriber('/mode/person_following', Bool, self.mode_cb, queue_size=1)
            self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

            # Timer điều khiển cố định 10 Hz
            self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
            rospy.loginfo("🎯 [HRI] Node person_follower_node đã sẵn sàng bám người (Khoảng cách mục tiêu: 0.85m)")

    def mode_cb(self, msg: Bool):
        self.is_enabled = bool(msg.data)
        status = "BẬT" if self.is_enabled else "TẮT"
        if HAS_ROS:
            rospy.loginfo(f"⚙️ [HRI] Chế độ bám người: {status}")

    def objects_cb(self, msg: String):
        """Nhận danh sách vật thể từ camera AI của Thành viên 2"""
        if not self.is_enabled: return

        try:
            detections = json.loads(msg.data)
            if not isinstance(detections, list): return

            # Lọc danh sách người (id == 0 hoặc name == 'PERSON')
            persons = [
                d for d in detections 
                if (d.get('id') == 0 or str(d.get('name', '')).upper() == 'PERSON')
                and 'x' in d and 'z' in d and d.get('z', 0) > 0.3
            ]

            if not persons: return

            # Nếu có nhiều người, chọn người gần trục giữa camera nhất (ưu tiên người trước mặt)
            target_person = min(persons, key=lambda p: abs(p.get('x', 0)))

            now = time.time()
            dt = max(0.01, now - self.last_ctrl_time)

            x = float(target_person['x']) # Tọa độ ngang (âm = lệch trái, dương = lệch phải)
            z = float(target_person['z']) # Khoảng cách chiều sâu phía trước

            # 1. Sai số khoảng cách
            ez = z - self.target_dist_z
            dez = (ez - self.last_ez) / dt

            # 2. Sai số góc lệch tâm
            ex = x
            dex = (ex - self.last_ex) / dt

            # 3. Tính toán bộ điều khiển PD
            # Vận tốc tiến/lùi
            if abs(ez) > self.deadzone_dist:
                cmd_v = (self.kp_dist * ez) + (self.kd_dist * dez)
                cmd_v = max(self.min_v, min(self.max_v, cmd_v))
            else:
                cmd_v = 0.0

            # Vận tốc quay (lệch phải x > 0 -> phải quay phải w < 0)
            if abs(ex) > self.deadzone_angle:
                cmd_w = (-self.kp_angle * ex) - (self.kd_angle * dex)
                cmd_w = max(-self.max_w, min(self.max_w, cmd_w))
            else:
                cmd_w = 0.0

            # Cập nhật lịch sử
            self.last_ez = ez
            self.last_ex = ex
            self.last_ctrl_time = now
            self.last_person_time = now

            # Phát lệnh lái xe
            if HAS_ROS:
                cmd = Twist()
                cmd.linear.x = cmd_v
                cmd.angular.z = cmd_w
                self.pub_cmd.publish(cmd)

        except Exception as e:
            if HAS_ROS:
                rospy.logerr_throttle(2.0, f"Lỗi xử lý bám người: {e}")

    def control_loop(self, event):
        """Bảo vệ: Nếu quá 0.8s không thấy người trong tầm nhìn -> Tự động dừng xe 1 lần"""
        if not self.is_enabled: return

        if self.last_person_time > 0 and (time.time() - self.last_person_time > 0.8):
            if HAS_ROS:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
            self.last_person_time = 0.0  # Dừng 1 lần rồi thôi, không spam cướp quyền điều khiển

    def shutdown(self):
        if HAS_ROS:
            cmd = Twist()
            self.pub_cmd.publish(cmd)
        print("🛑 [HRI] Đã dừng bám người an toàn.")

def main():
    follower = JetBotPersonFollowerNode()
    if HAS_ROS:
        rospy.on_shutdown(follower.shutdown)
        rospy.spin()
    else:
        print("Đang chạy kiểm thử person_follower_node không có ROS. Nhấn Ctrl+C để thoát.")
        try:
            while True: time.sleep(1.0)
        except KeyboardInterrupt:
            follower.shutdown()

if __name__ == '__main__':
    main()
