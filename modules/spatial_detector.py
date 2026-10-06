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

class SpatialPerceptionEngine:
    DEPTH_MIN = 0.20
    DEPTH_MAX = 10.00

    def __init__(self):
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
        """Lọc và chuẩn hóa danh sách vật thể từ VPU"""
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

            # Lọc cự ly an toàn
            if z < cls.DEPTH_MIN or z > cls.DEPTH_MAX: continue
            if score < 0.30: continue

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
        """Nhận diện dự phòng thông minh trên CPU (Face & UpperBody) khi chưa bật VPU YOLO"""
        if bgr_img is None: return []
        h, w = bgr_img.shape[:2]
        if h < 40 or w < 40: return []

        scale = 0.5
        small = cv2.resize(bgr_img, (int(w * scale), int(h * scale)))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        detections = []
        found_boxes = []

        if self.face_cascade and not self.face_cascade.empty():
            faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=4, minSize=(20, 20))
            for (fx_b, fy_b, fw, fh) in faces:
                found_boxes.append((int(fx_b / scale), int(fy_b / scale), int(fw / scale), int(fh / scale), "PERSON", 0.92))

        if not found_boxes and self.upper_cascade and not self.upper_cascade.empty():
            uppers = self.upper_cascade.detectMultiScale(gray, scaleFactor=1.25, minNeighbors=3, minSize=(30, 30))
            for (ux, uy, uw, uh) in uppers:
                found_boxes.append((int(ux / scale), int(uy / scale), int(uw / scale), int(uh / scale), "PERSON", 0.85))

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
                    valid = roi[(roi > 200) & (roi < 6000)]
                    if len(valid) > 10:
                        z_m = float(np.median(valid)) / 1000.0
                except Exception: pass

            # Nếu depth map chưa có hoặc quá gần, ước tính cự ly qua độ rộng khuôn mặt chuẩn ~16cm
            if z_m < 0.25 or z_m > 8.0:
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
        """Vẽ khung hộp nhận diện và cự ly trực tiếp lên khung ảnh camera"""
        if img is None: return img
        h, w = img.shape[:2]
        for det in detections:
            try:
                xm, ym, zm = float(det['x']), float(det['y']), float(det['z'])
                name = str(det.get('name', 'OBJ'))
                score = float(det.get('score', 0.8))

                if 'bbox' in det:
                    x1, y1, x2, y2 = det['bbox']
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

                col = (42, 42, 255) if 'PERSON' in name else (0, 255, 163)
                cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
                cv2.putText(img, f"{name} {int(score*100)}% ({zm:.1f}m)",
                            (x1, max(22, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 2)
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
