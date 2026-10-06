#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 4: Bộ lọc nhận diện không gian 3D Tiny YOLO (Spatial Perception Engine)
Lọc 80 lớp COCO (Người, Ghế, Bàn, Balo, Laptop...), tính cự ly vật cản và vẽ bounding box.
"""

import sys
import numpy as np
import cv2

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

COCO_CLASSES = {
    0: "PERSON", 1: "BICYCLE", 2: "CAR", 3: "MOTORCYCLE", 4: "AIRPLANE", 5: "BUS", 6: "TRAIN", 7: "TRUCK", 8: "BOAT",
    9: "TRAFFIC LIGHT", 10: "FIRE HYDRANT", 11: "STOP SIGN / DOOR", 12: "PARKING METER", 13: "BENCH", 14: "BIRD", 15: "CAT",
    16: "DOG", 17: "HORSE", 18: "SHEEP", 19: "COW", 20: "ELEPHANT", 21: "BEAR", 22: "ZEBRA", 23: "GIRAFFE",
    24: "BACKPACK", 25: "UMBRELLA", 26: "HANDBAG", 27: "TIE", 28: "SUITCASE", 29: "FRISBEE", 30: "SKIS", 31: "SNOWBOARD",
    32: "SPORTS BALL", 33: "KITE", 34: "BASEBALL BAT", 35: "BASEBALL GLOVE", 36: "SKATEBOARD", 37: "SURFBOARD",
    38: "TENNIS RACKET", 39: "BOTTLE", 40: "WINE GLASS", 41: "CUP", 42: "FORK", 43: "KNIFE", 44: "SPOON", 45: "BOWL",
    46: "BANANA", 47: "APPLE", 48: "SANDWICH", 49: "ORANGE", 50: "BROCCOLI", 51: "CARROT", 52: "HOT DOG", 53: "PIZZA",
    54: "DONUT", 55: "CAKE", 56: "CHAIR", 57: "COUCH", 58: "POTTED PLANT", 59: "BED", 60: "TABLE", 61: "TOILET",
    62: "TV / MONITOR", 63: "LAPTOP", 64: "MOUSE", 65: "REMOTE", 66: "KEYBOARD", 67: "CELL PHONE", 68: "MICROWAVE",
    69: "OVEN", 70: "TOASTER", 71: "SINK", 72: "REFRIGERATOR", 73: "BOOK", 74: "CLOCK", 75: "VASE", 76: "SCISSORS",
    77: "TEDDY BEAR", 78: "HAIR DRIER", 79: "TOOTHBRUSH"
}

LABEL_SYNONYMS = {
    "person": (0, "PERSON"), "chair": (56, "CHAIR"), "couch": (57, "COUCH"), "sofa": (57, "COUCH"),
    "table": (60, "TABLE"), "desk": (60, "TABLE"), "tv": (62, "TV / MONITOR"), "monitor": (62, "TV / MONITOR"),
    "backpack": (24, "BACKPACK"), "bag": (24, "BACKPACK"), "handbag": (26, "BACKPACK"),
    "laptop": (63, "LAPTOP"), "phone": (67, "CELL PHONE"), "cell phone": (67, "CELL PHONE"),
    "book": (73, "BOOK"), "bottle": (39, "BOTTLE"), "cup": (41, "CUP")
}

class TemporalTracker:
    """Bộ lọc thời gian làm mượt quỹ đạo và chống chập chờn (Coasting / Anti-Flicker)"""
    def __init__(self, max_age_seconds=0.6):
        self.max_age = max_age_seconds
        self.tracks = {}

    def update(self, new_detections):
        import time, math
        now = time.time()
        updated = {}

        for det in new_detections:
            name = det.get('name', 'OBJ')
            x, y, z = det.get('x', 0.0), det.get('y', 0.0), det.get('z', 0.0)
            bbox = det.get('bbox', None)

            best_k = None
            min_d = 0.85

            for k, trk in self.tracks.items():
                if trk['name'] == name:
                    d = math.sqrt((trk['x'] - x)**2 + (trk['y'] - y)**2 + (trk['z'] - z)**2)
                    if d < min_d:
                        min_d = d
                        best_k = k

            if best_k is not None:
                old = self.tracks[best_k]
                sx = round(0.70 * x + 0.30 * old['x'], 2)
                sy = round(0.70 * y + 0.30 * old['y'], 2)
                sz = round(0.70 * z + 0.30 * old['z'], 2)
                updated[best_k] = {
                    "id": det.get('id', 0),
                    "name": name,
                    "score": max(det.get('score', 0.8), round(old['score'] * 0.96, 2)),
                    "x": sx, "y": sy, "z": sz,
                    "bbox": bbox if bbox else old.get('bbox'),
                    "last_seen": now
                }
            else:
                k = f"{name}_{now}_{len(updated)}"
                updated[k] = {
                    "id": det.get('id', 0),
                    "name": name,
                    "score": det.get('score', 0.8),
                    "x": round(x, 2), "y": round(y, 2), "z": round(z, 2),
                    "bbox": bbox,
                    "last_seen": now
                }

        # Coasting: Giữ lại track vừa mất dấu trong 0.6s để không bị nhấp nháy
        for k, trk in self.tracks.items():
            if k not in updated and (now - trk['last_seen'] < self.max_age):
                trk['score'] = round(trk['score'] * 0.92, 2)
                updated[k] = trk

        self.tracks = updated

        res = []
        for trk in self.tracks.values():
            item = {
                "id": trk["id"],
                "name": trk["name"],
                "score": round(trk["score"], 2),
                "x": trk["x"],
                "y": trk["y"],
                "z": trk["z"]
            }
            if trk.get("bbox"):
                item["bbox"] = trk["bbox"]
            res.append(item)
        return res


class SpatialPerceptionEngine:
    DEPTH_MIN = 0.15
    DEPTH_MAX = 10.00

    def __init__(self):
        self.tracker = TemporalTracker(max_age_seconds=0.6)
        self.face_cascade = None
        self.upper_cascade = None
        try:
            import os
            p_face = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            if os.path.exists(p_face):
                self.face_cascade = cv2.CascadeClassifier(p_face)
            p_upper = cv2.data.haarcascades + 'haarcascade_upperbody.xml'
            if os.path.exists(p_upper):
                self.upper_cascade = cv2.CascadeClassifier(p_upper)
        except Exception:
            pass

    @classmethod
    def filter_detections(cls, raw_list):
        """Lọc, chuẩn hóa đơn vị mm -> m, và ánh xạ nhãn COCO"""
        filtered = []
        for det in raw_list:
            if not isinstance(det, dict): continue
            cid = det.get("id")
            raw_name = str(det.get("name", "")).strip()

            target_id = None
            target_name = None

            if cid is not None and str(cid).isdigit() and int(cid) in COCO_CLASSES:
                target_id = int(cid)
                target_name = COCO_CLASSES[target_id]
            elif raw_name:
                low = raw_name.lower()
                if low in LABEL_SYNONYMS:
                    target_id, target_name = LABEL_SYNONYMS[low]
                else:
                    target_id = int(cid) if (cid is not None and str(cid).isdigit()) else 99
                    target_name = raw_name.upper()

            if not target_name: continue

            try:
                x = float(det.get("x", 0.0))
                y = float(det.get("y", 0.0))
                z = float(det.get("z", 0.0))
                score = float(det.get("score", 0.8))
            except (ValueError, TypeError): continue

            # Tự động phát hiện và chuyển đổi đơn vị nếu xuất milimet (mm -> m)
            if z > 20.0:
                x /= 1000.0
                y /= 1000.0
                z /= 1000.0

            # Lọc cự ly an toàn [15cm - 10m] và ngưỡng tin cậy nhạy hơn (0.20)
            if z < cls.DEPTH_MIN or z > cls.DEPTH_MAX: continue
            if score < 0.20: continue

            d_entry = {
                "id": target_id if target_id is not None else 0,
                "name": target_name,
                "score": round(score, 2),
                "x": round(x, 2),
                "y": round(y, 2),
                "z": round(z, 2)
            }
            if "bbox" in det:
                d_entry["bbox"] = det["bbox"]

            filtered.append(d_entry)
        return filtered

    def detect_fallback(self, bgr_img, depth_frame=None, fx=450.0, fy=450.0, cx=320.0, cy=200.0):
        """Nhận diện dự phòng siêu nhẹ trên CPU khi chưa có topic từ VPU"""
        if bgr_img is None: return []
        h, w = bgr_img.shape[:2]
        if h < 40 or w < 40: return []

        scale = 0.35
        small = cv2.resize(bgr_img, (int(w * scale), int(h * scale)))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        found_boxes = []

        if self.face_cascade and not self.face_cascade.empty():
            faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.25, minNeighbors=4, minSize=(16, 16))
            for (fx_b, fy_b, fw, fh) in faces:
                found_boxes.append((int(fx_b / scale), int(fy_b / scale), int(fw / scale), int(fh / scale), "PERSON", 0.90))

        if not found_boxes and self.upper_cascade and not self.upper_cascade.empty():
            uppers = self.upper_cascade.detectMultiScale(gray, scaleFactor=1.30, minNeighbors=3, minSize=(25, 25))
            for (ux, uy, uw, uh) in uppers:
                found_boxes.append((int(ux / scale), int(uy / scale), int(uw / scale), int(uh / scale), "PERSON", 0.82))

        detections = []
        for (bx, by, bw, bh, name, score) in found_boxes:
            u_center = bx + bw // 2
            v_center = by + bh // 2

            z_m = 0.0
            if depth_frame is not None and isinstance(depth_frame, np.ndarray):
                try:
                    dh, dw = depth_frame.shape[:2]
                    du1 = max(0, min(dw - 1, int(bx * dw / w)))
                    dv1 = max(0, min(dh - 1, int(by * dh / h)))
                    du2 = max(0, min(dw - 1, int((bx + bw) * dw / w)))
                    dv2 = max(0, min(dh - 1, int((by + bh) * dh / h)))
                    roi = depth_frame[dv1:dv2, du1:du2]
                    valid = roi[(roi > 150) & (roi < 6000)]
                    if len(valid) > 8:
                        z_m = float(np.median(valid)) / 1000.0
                except Exception: pass

            if z_m < 0.20 or z_m > 8.0:
                z_m = round(float(fx * 0.16 / max(bw, 1)), 2)
                z_m = max(0.30, min(4.5, z_m))

            x_m = round(float((u_center - cx) * z_m / fx), 2)
            y_m = round(float((v_center - cy) * z_m / fy), 2)

            detections.append({
                "id": 0,
                "name": name,
                "score": score,
                "x": x_m,
                "y": y_m,
                "z": round(z_m, 2),
                "bbox": [bx, by, bx + bw, by + bh]
            })

        return self.filter_detections(detections)

    @staticmethod
    def calculate_obstacle_distance(depth_frame):
        """Tính cự ly vật cản trung tâm phân vị 10% (10th percentile depth)"""
        if depth_frame is None or not isinstance(depth_frame, np.ndarray):
            return 99.0
        h, w = depth_frame.shape[:2]
        if h < 10 or w < 10: return 99.0

        h_start, h_end = int(h / 3.0), int(2.0 * h / 3.0)
        w_start, w_end = int(w / 3.0), int(2.0 * w / 3.0)
        roi = depth_frame[h_start:h_end, w_start:w_end]

        if roi.dtype == np.uint16 or roi.max() > 100.0:
            roi_m = roi.astype(np.float32) / 1000.0
        else:
            roi_m = roi.astype(np.float32)

        valid = roi_m[(roi_m >= 0.15) & (roi_m <= 5.0)]
        if len(valid) >= 15:
            return round(float(np.percentile(valid, 10)), 2)
        return 99.0

    @staticmethod
    def draw_detections(img, detections, fx=450.0, fy=450.0, cx=320.0, cy=200.0):
        """Vẽ khung hộp nhận diện chuẩn 2D từ YOLO với nhãn và cự ly chính xác"""
        if img is None: return img
        h, w = img.shape[:2]
        for det in detections:
            try:
                xm, ym, zm = float(det['x']), float(det['y']), float(det['z'])
                name = str(det.get('name', 'OBJ'))
                score = float(det.get('score', 0.8))

                # Ưu tiên số 1: Bounding Box 2D thật từ mạng YOLO
                if 'bbox' in det and det['bbox'] is not None:
                    bx1, by1, bx2, by2 = det['bbox']
                    x1 = max(0, min(w - 2, int(bx1)))
                    y1 = max(0, min(h - 2, int(by1)))
                    x2 = max(x1 + 5, min(w - 1, int(bx2)))
                    y2 = max(y1 + 5, min(h - 1, int(by2)))
                elif zm > 0.15:
                    u = int(cx + (xm * fx / zm))
                    v = int(cy + (ym * fy / zm))
                    if 10 <= u < w - 10 and 10 <= v < h - 10:
                        bw = max(35, min(400, int(180.0 / zm)))
                        bh = max(50, min(500, int(260.0 / zm)))
                        x1, y1 = max(0, u - bw // 2), max(0, v - bh // 2)
                        x2, y2 = min(w - 1, u + bw // 2), min(h - 1, v + bh // 2)
                    else: continue
                else: continue

                # Bảng màu chuyên nghiệp theo danh mục đối tượng
                if 'PERSON' in name:
                    col = (42, 42, 255) # Đỏ rực
                elif 'LAPTOP' in name or 'TV' in name:
                    col = (255, 180, 0) # Xanh dương / Vàng cam
                elif 'BOTTLE' in name or 'CUP' in name:
                    col = (255, 0, 200) # Hồng tím
                elif 'CHAIR' in name or 'TABLE' in name:
                    col = (0, 215, 255) # Vàng hổ phách
                else:
                    col = (0, 255, 163) # Xanh ngọc neon Cyberpunk

                cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)

                dist_str = f" ({zm:.2f}m)" if zm > 0.05 else ""
                label_txt = f"{name} {int(score*100)}%{dist_str}"
                (tw, th), _ = cv2.getTextSize(label_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
                ty = max(18, y1 - 4)
                cv2.rectangle(img, (x1, ty - th - 3), (x1 + tw + 6, ty + 2), col, -1)
                cv2.putText(img, label_txt, (x1 + 3, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1, cv2.LINE_AA)
            except Exception: pass
        return img


if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử SpatialPerceptionEngine...")
    raw = [
        {"id": 0, "name": "person", "score": 0.88, "x": 0.5, "y": 0.1, "z": 5.8},
        {"id": 24, "name": "backpack", "score": 0.91, "x": -0.1, "y": 0.0, "z": 0.35},
        {"id": 56, "name": "chair", "score": 0.75, "x": 1.2, "y": -0.2, "z": 1.5},
        {"id": 0, "name": "person", "score": 0.85, "x": 0.0, "y": 0.0, "z": 15.0}  # Ngoài 10m -> phải bị lọc
    ]
    res = SpatialPerceptionEngine.filter_detections(raw)
    print(f"  ├─ Số lượng vật thể sau lọc: {len(res)} / {len(raw)}")
    for obj in res:
        print(f"     * [{obj['name']}] Score: {obj['score']} | Z: {obj['z']}m")
    assert len(res) == 3, "Lỗi: Không lọc đúng số lượng vật thể!"
    print("✅ [SELF-TEST] SpatialPerceptionEngine ĐẠT CHUẨN!")
