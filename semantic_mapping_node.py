#!/usr/bin/env python3
"""
══════════════════════════════════════════════════════════════════════════════
JETBOT 3D SEMANTIC MAPPING ENGINE (MEMBER 3 CORE NODE)
══════════════════════════════════════════════════════════════════════════════
- Lắng nghe tọa độ vật thể 3D từ Thành viên 2 (/spatial_objects)
- Tra cứu cây tọa độ TF2: Chiếu từ camera optical frame sang map frame
- Thuật toán gom cụm không gian (Euclidean Clustering, d < 0.4m)
- Bộ lọc bền vững (Persistence Filter >= 5 frames): Chống nhấp nháy / nhãn ảo
- Xuất MarkerArray 3D cho RViz và Web Cockpit (/semantic_markers)
══════════════════════════════════════════════════════════════════════════════
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import json, math, time

try:
    from depthai_ros_msgs.msg import SpatialDetectionArray
    HAS_DEPTHAI = True
except ImportError:
    HAS_DEPTHAI = False

class SemanticObject:
    def __init__(self, obj_id, name, x, y, z, score):
        self.obj_id = obj_id
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.score = score
        self.seen_count = 1
        self.last_seen = time.time()

    def update(self, x, y, z, score):
        # Cập nhật trung bình trọng số vị trí (Exponential Moving Average)
        alpha = 0.3
        self.x = (1 - alpha) * self.x + alpha * x
        self.y = (1 - alpha) * self.y + alpha * y
        self.z = (1 - alpha) * self.z + alpha * z
        self.score = max(self.score, score)
        self.seen_count += 1
        self.last_seen = time.time()

class SemanticMappingNode:
    def __init__(self):
        rospy.init_node('semantic_mapping_node', anonymous=False)

        # Cấu hình ngưỡng gom cụm (Clustering Threshold)
        self.cluster_dist_thresh = 0.40  # 40 cm
        self.min_seen_to_publish = 4     # Thấy >= 4 lần mới chính thức cắm cờ
        self.object_catalog = []          # Danh mục vật thể toàn cục

        # Cây tọa độ TF2
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers
        self.marker_pub = rospy.Publisher('/semantic_markers', MarkerArray, queue_size=5)
        self.catalog_pub = rospy.Publisher('/semantic_catalog_json', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/spatial_objects', String, self.json_objects_cb, queue_size=5)

        # Fallback: nếu Thành viên 2 publish topic gốc của depthai
        if HAS_DEPTHAI:
            rospy.Subscriber('/stereo_inertial_publisher/color/yolov4_Spatial_detections',
                             SpatialDetectionArray, self.depthai_objects_cb, queue_size=5)

        # Timer định kỳ publish Markers & dọn rác
        rospy.Timer(rospy.Duration(0.5), self.publish_markers)

        rospy.loginfo("🧠 JetBot Semantic Mapping Engine Started! Waiting for 3D objects...")

    def transform_to_map(self, x_cam, y_cam, z_cam, frame_id='oak-d_frame'):
        """Chiếu điểm từ Camera Frame sang Map Frame qua TF2"""
        try:
            pt = PointStamped()
            pt.header.frame_id = frame_id
            pt.header.stamp = rospy.Time(0)  # Lấy transform mới nhất
            pt.point.x = x_cam
            pt.point.y = y_cam
            pt.point.z = z_cam

            # Chờ transform tối đa 0.3s
            pt_map = self.tf_buffer.transform(pt, 'map', timeout=rospy.Duration(0.3))
            return pt_map.point.x, pt_map.point.y, pt_map.point.z
        except Exception:
            # Fallback nếu chưa có SLAM/Map frame: tra cứu odom
            try:
                pt_odom = self.tf_buffer.transform(pt, 'odom', timeout=rospy.Duration(0.2))
                return pt_odom.point.x, pt_odom.point.y, pt_odom.point.z
            except Exception:
                return None

    def add_or_update_object(self, obj_id, name, x_m, y_m, z_m, score):
        """Thuật toán Euclidean Clustering"""
        for obj in self.object_catalog:
            if obj.name == name or (obj.obj_id == obj_id and obj_id != 0):
                d = math.hypot(x_m - obj.x, y_m - obj.y)
                if d < self.cluster_dist_thresh:
                    obj.update(x_m, y_m, z_m, score)
                    return

        # Nếu chưa có vật thể nào trong bán kính 0.4m -> Thêm mới
        new_obj = SemanticObject(obj_id, name, x_m, y_m, z_m, score)
        self.object_catalog.append(new_obj)
        rospy.loginfo(f"📍 Phát hiện vật thể mới: [{name}] tại Map (X:{x_m:.2f}, Y:{y_m:.2f}, Z:{z_m:.2f})")

    def json_objects_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            for item in data:
                cid = item.get('id', 0)
                name = item.get('name', 'OBJ')
                score = float(item.get('score', 0.5))
                # Tọa độ trong camera optical frame (X phải, Y xuống, Z tới)
                xc, yc, zc = float(item.get('x', 0)), float(item.get('y', 0)), float(item.get('z', 0))
                
                # Chiếu sang map
                res = self.transform_to_map(xc, yc, zc)
                if res:
                    self.add_or_update_object(cid, name, res[0], res[1], res[2], score)
        except Exception: pass

    def depthai_objects_cb(self, msg):
        labels_map = {0: "PERSON", 56: "CHAIR", 60: "TABLE", 62: "TV", 11: "STOP SIGN"}
        for d in getattr(msg, 'detections', []):
            for res in getattr(d, 'results', []):
                pos = getattr(d, 'position', None)
                if pos:
                    cid = getattr(res, 'id', 0)
                    name = labels_map.get(cid, f"OBJ #{cid}")
                    score = float(getattr(res, 'score', 0.5))
                    # depthai mm sang mét
                    xc, yc, zc = pos.x / 1000.0, pos.y / 1000.0, pos.z / 1000.0
                    map_pos = self.transform_to_map(xc, yc, zc)
                    if map_pos:
                        self.add_or_update_object(cid, name, map_pos[0], map_pos[1], map_pos[2], score)

    def publish_markers(self, event):
        if not self.object_catalog: return

        marker_array = MarkerArray()
        idx = 0
        now = rospy.Time.now()

        # Màu sắc từng loại vật thể
        color_map = {
            "PERSON": (1.0, 0.16, 0.43), # Hồng neon
            "CHAIR":  (0.0, 1.0, 0.64), # Xanh ngọc
            "TABLE":  (0.71, 0.22, 0.95),# Tím
            "TV":     (0.0, 0.94, 1.0),  # Cyan
            "STOP SIGN": (1.0, 0.72, 0.0)# Vàng cam
        }

        for obj in self.object_catalog:
            # Chỉ publish vật thể đã quan sát thấy >= ngưỡng bền vững
            if obj.seen_count < self.min_seen_to_publish:
                continue

            r, g, b = color_map.get(obj.name, (0.5, 0.5, 1.0))

            # 1. Khung hộp 3D (CUBE MARKER)
            box = Marker()
            box.header.frame_id = "map"
            box.header.stamp = now
            box.ns = "semantic_boxes"
            box.id = idx
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = obj.x
            box.pose.position.y = obj.y
            box.pose.position.z = obj.z
            box.scale.x = 0.45; box.scale.y = 0.45; box.scale.z = 0.55
            box.color.r = r; box.color.g = g; box.color.b = b; box.color.a = 0.45
            box.lifetime = rospy.Duration(1.2)
            marker_array.markers.append(box)
            idx += 1

            # 2. Chữ nổi 3D (TEXT VIEW FACING)
            txt = Marker()
            txt.header.frame_id = "map"
            txt.header.stamp = now
            txt.ns = "semantic_text"
            txt.id = idx
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = obj.x
            txt.pose.position.y = obj.y
            txt.pose.position.z = obj.z + 0.35  # Nổi cao hơn hộp
            txt.text = f"{obj.name} [X:{obj.x:.1f}, Y:{obj.y:.1f}] ({obj.seen_count}x)"
            txt.scale.z = 0.16  # Chiều cao chữ
            txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 0.95
            txt.lifetime = rospy.Duration(1.2)
            marker_array.markers.append(txt)
            idx += 1

        self.marker_pub.publish(marker_array)

        # Xuất JSON danh mục
        summary = [{"name": o.name, "x": round(o.x, 2), "y": round(o.y, 2), "z": round(o.z, 2), "count": o.seen_count}
                   for o in self.object_catalog if o.seen_count >= self.min_seen_to_publish]
        self.catalog_pub.publish(json.dumps(summary))

def main():
    node = SemanticMappingNode()
    rospy.spin()

if __name__ == '__main__':
    main()
