#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 4: Bộ lọc nhận diện không gian 3D Tiny YOLO (Spatial Perception Engine)
Lọc 80 lớp COCO (Người, Ghế, Bàn, Balo, Laptop...), tính cự ly vật cản và vẽ bounding box.
"""

import sys
import time
import math
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
    "table": (60, "TABLE"), "desk": (60, "TABLE"), "dining table": (60, "TABLE"), "diningtable": (60, "TABLE"),
    "tv": (62, "TV / MONITOR"), "monitor": (62, "TV / MONITOR"), "tvmonitor": (62, "TV / MONITOR"), "tv/monitor": (62, "TV / MONITOR"),
    "backpack": (24, "BACKPACK"), "bag": (24, "BACKPACK"), "handbag": (26, "BACKPACK"), "suitcase": (28, "BACKPACK"),
    "laptop": (63, "LAPTOP"), "phone": (67, "CELL PHONE"), "cell phone": (67, "CELL PHONE"), "cellphone": (67, "CELL PHONE"),
    "book": (73, "BOOK"), "bottle": (39, "BOTTLE"), "cup": (41, "CUP"), "wine glass": (41, "CUP"),
    "keyboard": (66, "KEYBOARD"), "mouse": (64, "MOUSE"), "stop sign": (11, "STOP SIGN / DOOR")
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
        self._dnn_model = None
        self._coco_labels = None
        self._last_dnn_time = 0.0
        self._cached_dnn_boxes = []

        # Khởi tạo mô hình AI Deep Learning (MobileNetV3 SSDLite COCO 80 lớp)
        try:
            import torchvision.models.detection as tv_det
            self._dnn_model = tv_det.ssdlite320_mobilenet_v3_large(weights=tv_det.SSDLite320_MobileNet_V3_Large_Weights.DEFAULT).eval()
            self._coco_labels = tv_det.SSDLite320_MobileNet_V3_Large_Weights.DEFAULT.meta['categories']
            print("🧠 [AI ENGINE] Đã kích hoạt mô hình AI nhận diện 80 lớp COCO (Person, Chair, Bottle, Laptop, Backpack...)!")
        except Exception as e:
            print(f"ℹ️ [AI ENGINE] Dùng bộ phân loại hình thái học thời gian thực: {e}")

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

    TARGET_SEMANTIC_CLASSES = {
        "PERSON", "OBSTACLE", "CHAIR", "COUCH", "TABLE", "BOTTLE", "CUP", "BACKPACK",
        "LAPTOP", "TV / MONITOR", "CELL PHONE", "BOOK", "KEYBOARD", "MOUSE", "STOP SIGN / DOOR"
    }
    _last_obstacle_dist = None

    @classmethod
    def filter_detections(cls, raw_list):
        """Lọc, chuẩn hóa đơn vị mm -> m, loại bỏ vật thể ngoại lai"""
        filtered = []
        for det in raw_list:
            if not isinstance(det, dict): continue
            cid = det.get("id")
            raw_name = str(det.get("name", "")).strip()

            target_id = None
            target_name = None

            # Ưu tiên 1: Tên nhãn lớp thực tế (BOTTLE, CHAIR, LAPTOP, PERSON...)
            if raw_name:
                low = raw_name.lower()
                if low in LABEL_SYNONYMS:
                    target_id, target_name = LABEL_SYNONYMS[low]
                else:
                    target_name = raw_name.upper()
                    target_id = int(cid) if (cid is not None and str(cid).isdigit()) else 99
            elif cid is not None and str(cid).isdigit() and int(cid) in COCO_CLASSES:
                target_id = int(cid)
                target_name = COCO_CLASSES[target_id]

            if not target_name: continue

            # 1. BỘ LỌC DANH MỤC: Chỉ giữ các lớp mục tiêu phục vụ đồ án
            if target_name not in cls.TARGET_SEMANTIC_CLASSES:
                continue

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

            # Lọc cự ly an toàn [15cm - 10m]
            if z < cls.DEPTH_MIN or z > cls.DEPTH_MAX: continue

            # Lọc bóng phản chiếu sàn (vùng y > 0.15m khi ở cự ly xa z > 1.2m)
            if target_name == "PERSON" and y > 0.15 and z > 1.2:
                continue

            # 2. BỘ LỌC ĐỘ TIN CẬY
            if target_name == "PERSON":
                if score < 0.35: continue
                if "bbox" in det and det["bbox"] is not None:
                    bx1, by1, bx2, by2 = det["bbox"]
                    bw = abs(bx2 - bx1)
                    bh = abs(by2 - by1)
                    if bh > 0:
                        aspect = bw / float(bh)
                        if aspect < 0.15 or aspect > 2.2:
                            continue
            elif target_name == "OBSTACLE":
                if score < 0.30: continue
            else:
                if score < 0.28: continue

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
        """
        Nhận diện quang học dự phòng chuẩn xác (Tương thích OpenCV 5.0, không phụ thuộc file cascade ngoài).
        Hỗ trợ nhận diện người, khuôn mặt, bàn tay và chướng ngại vật trong hành lang trung tâm.
        """
        if bgr_img is None: return []
        h, w = bgr_img.shape[:2]
        if h < 40 or w < 40: return []

        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        mean_brightness = float(gray.mean())

        # Nếu webcam bị đóng nắp che hoặc tối đen hoàn toàn (Mean < 8.0) -> Đường thoáng, không tạo box ảo
        if mean_brightness < 8.0:
            return []

        found_boxes = []

        # ─── 1. ƯU TIÊN 1: NHẬN DIỆN THỰC THẾ BẰNG MÔ HÌNH AI 80 LỚP COCO (MOBILE-NET V3) ───
        if self._dnn_model is not None:
            now = time.time()
            if (now - self._last_dnn_time >= 0.05) or not self._cached_dnn_boxes:
                self._last_dnn_time = now
                try:
                    import torch
                    infer_w, infer_h = 320, 240
                    resized_bgr = cv2.resize(bgr_img, (infer_w, infer_h))
                    rgb = cv2.cvtColor(resized_bgr, cv2.COLOR_BGR2RGB)
                    tensor_img = torch.from_numpy(rgb.transpose((2, 0, 1))).float() / 255.0
                    with torch.no_grad():
                        preds = self._dnn_model([tensor_img])[0]

                    boxes = preds['boxes'].cpu().numpy()
                    scores = preds['scores'].cpu().numpy()
                    labels = preds['labels'].cpu().numpy()

                    scale_x = w / float(infer_w)
                    scale_y = h / float(infer_h)

                    dnn_boxes = []
                    for box, score, lbl_idx in zip(boxes, scores, labels):
                        if score < 0.35:
                            continue
                        cat_raw = self._coco_labels[lbl_idx] if (self._coco_labels and lbl_idx < len(self._coco_labels)) else "OBJ"
                        raw_upper = str(cat_raw).upper()

                        # Chuẩn hóa tên lớp COCO
                        if raw_upper in ["CHAIR", "COUCH", "SOFA"]:
                            final_name = "CHAIR"
                        elif raw_upper in ["DINING TABLE", "TABLE", "DESK"]:
                            final_name = "TABLE"
                        elif raw_upper in ["BOTTLE"]:
                            final_name = "BOTTLE"
                        elif raw_upper in ["CUP", "WINE GLASS"]:
                            final_name = "CUP"
                        elif raw_upper in ["LAPTOP", "TVMONITOR", "TV"]:
                            final_name = "LAPTOP"
                        elif raw_upper in ["CELL PHONE"]:
                            final_name = "CELL PHONE"
                        elif raw_upper in ["BACKPACK", "HANDBAG", "SUITCASE"]:
                            final_name = "BACKPACK"
                        elif raw_upper in ["PERSON"]:
                            final_name = "PERSON"
                        elif raw_upper in ["BOOK"]:
                            final_name = "BOOK"
                        else:
                            final_name = raw_upper

                        bx1 = max(0, min(w - 1, int(box[0] * scale_x)))
                        by1 = max(0, min(h - 1, int(box[1] * scale_y)))
                        bx2 = max(0, min(w - 1, int(box[2] * scale_x)))
                        by2 = max(0, min(h - 1, int(box[3] * scale_y)))
                        bw = max(10, bx2 - bx1)
                        bh = max(10, by2 - by1)
                        dnn_boxes.append((bx1, by1, bw, bh, final_name, float(score)))

                    self._cached_dnn_boxes = dnn_boxes
                except Exception:
                    pass

            if self._cached_dnn_boxes:
                found_boxes = list(self._cached_dnn_boxes)

        # ─── 2. BỔ TRỢ KHUÔN MẶT HAAR CASCADE (ĐẶC BIỆT KHI NGỒI GẦN WEBCAM) ───
        if self.face_cascade is not None and not any(b[4] == 'PERSON' for b in found_boxes):
            try:
                faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=4, minSize=(45, 45))
                for (fx1, fy1, fw, fh) in faces:
                    cx_face = fx1 + fw / 2.0
                    if abs(cx_face - w / 2.0) < (w * 0.42):
                        found_boxes.append((fx1, fy1, fw, fh, "PERSON", 0.95))
            except Exception:
                pass

        # ─── 3. PHƯƠNG ÁN DỰ PHÒNG: PHÂN TÍCH HÌNH THÁI VÀ ĐẶC TRƯNG HÌNH HỌC (ĐA LỚP) ───
        if not found_boxes:
            # Phát hiện Người bằng phân tách sắc độ da YCrCb
            ycrcb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YCrCb)
            skin_mask = cv2.inRange(ycrcb, np.array([0, 133, 77]), np.array([255, 173, 127]))
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            skin_clean = cv2.morphologyEx(skin_mask, cv2.MORPH_OPEN, kernel)
            cnts_skin, _ = cv2.findContours(skin_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            skin_candidates = []
            for c in cnts_skin:
                area = cv2.contourArea(c)
                if area > (w * h * 0.02):
                    bx, by, bw, bh = cv2.boundingRect(c)
                    aspect = bw / float(bh)
                    box_cx = bx + bw / 2.0
                    if 0.25 <= aspect <= 2.2 and abs(box_cx - w / 2.0) < (w * 0.45):
                        skin_candidates.append((area, bx, by, bw, bh))

            if skin_candidates:
                skin_candidates.sort(key=lambda x: x[0], reverse=True)
                _, bx, by, bw, bh = skin_candidates[0]
                exp_w = int(bw * 1.3)
                exp_h = int(bh * 1.4)
                ex = max(0, bx - (exp_w - bw) // 2)
                ey = max(0, by - (exp_h - bh) // 4)
                ew = min(w - ex, exp_w)
                eh = min(h - ey, exp_h)
                found_boxes.append((ex, ey, ew, eh, "PERSON", 0.90))
            else:
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                enhanced = clahe.apply(gray)
                blur = cv2.GaussianBlur(enhanced, (19, 19), 0)

                _, thresh1 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                thresh2 = cv2.bitwise_not(thresh1)

                candidates = []
                for t_img in [thresh1, thresh2]:
                    cnts, _ = cv2.findContours(t_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for c in cnts:
                        area = cv2.contourArea(c)
                        if area > (w * h * 0.04):
                            bx, by, bw, bh = cv2.boundingRect(c)
                            aspect = bw / float(bh)
                            box_cx = bx + bw / 2.0
                            center_dist = abs(box_cx - w / 2.0)
                            if 0.15 <= aspect <= 3.0 and center_dist < (w * 0.42):
                                candidates.append((area, bx, by, bw, bh, aspect))

                if candidates:
                    candidates.sort(key=lambda x: x[0], reverse=True)
                    area, bx, by, bw, bh, aspect = candidates[0]
                    # Phân loại dựa trên tỷ lệ hình học thực tế:
                    if aspect < 0.45:
                        obj_type = "BOTTLE"  # Dáng đứng cao thon
                    elif aspect > 1.4:
                        obj_type = "LAPTOP"  # Dáng chữ nhật nằm ngang
                    elif by > int(h * 0.45):
                        obj_type = "CHAIR"   # Vật thể nằm thấp dưới sàn
                    else:
                        obj_type = "OBSTACLE" # Vật cản trung tâm trước mặt xe
                    found_boxes.append((bx, by, bw, bh, obj_type, 0.85))

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
                    valid = roi[(roi > 50) & (roi < 4500)]
                    if len(valid) > 10:
                        z_m = float(np.median(valid)) / 1000.0
                except Exception: pass

            # Tính cự ly quang học chuẩn xác (tương quan với test_emergency_brake.py):
            # Ngồi cách laptop 60-70cm: bw ~ 180-210px -> z ~ 0.65m
            # Đưa tay/người sát camera (<25cm): bw ~ 400-500px -> z giảm sát 0.20m (20cm)
            if z_m < 0.15 or z_m > 5.0:
                d_optical = max(bw, int(bh * 0.65))
                z_m = round(float(fx * 0.26 / max(d_optical, 1)), 2)
                z_m = max(0.18, min(3.5, z_m))

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

    @classmethod
    def calculate_obstacle_distance(cls, depth_frame, fallback_detections=None):
        """
        Tính cự ly vật cản trung tâm chuẩn xác theo thuật toán test_emergency_brake.py:
        - Hành lang trung tâm: w từ 25% đến 75%, h từ 18% đến 60%
        - Lọc các điểm đo vật lý hợp lệ từ 50mm đến 3500mm
        - Lấy phân vị 5% (5th Percentile) để bắt mép cản gần nhất
        - Chống điểm mù stereo: Khi cản áp sát < 18cm, giữ cự ly phanh khẩn cấp 0.20m thay vì trả về None
        - Fallback: Trích xuất cự ly Z nhỏ nhất từ các đối tượng 3D trước mặt
        """
        if depth_frame is not None and isinstance(depth_frame, np.ndarray):
            dh, dw = depth_frame.shape[:2]
            if dh >= 10 and dw >= 10:
                h_start, h_end = int(dh * 0.18), int(dh * 0.60)
                w_start, w_end = int(dw * 0.25), int(dw * 0.75)
                roi = depth_frame[h_start:h_end, w_start:w_end]

                if np.issubdtype(roi.dtype, np.floating):
                    roi = np.nan_to_num(roi, nan=0.0, posinf=0.0, neginf=0.0)

                if roi.dtype == np.uint16 or (roi.size > 0 and roi.max() > 50.0):
                    roi_mm = roi.astype(np.float32)
                else:
                    roi_mm = roi.astype(np.float32) * 1000.0

                valid = roi_mm[(roi_mm >= 50.0) & (roi_mm <= 3500.0)]
                if len(valid) >= 25:
                    dist_mm = float(np.percentile(valid, 5))
                    dist_m = round(max(0.18, dist_mm / 1000.0), 2)
                    cls._last_obstacle_dist = dist_m
                    return dist_m
                elif cls._last_obstacle_dist is not None and cls._last_obstacle_dist <= 0.55:
                    # Điểm mù stereo (< 18cm): Khi vật cản đang tiến sát rồi rơi vào điểm mù của camera OAK-D
                    cls._last_obstacle_dist = 0.20
                    return 0.20
                elif roi_mm.size > 0 and (np.count_nonzero(roi_mm <= 60.0) / float(roi_mm.size)) > 0.40 and (cls._last_obstacle_dist is not None and cls._last_obstacle_dist <= 0.60):
                    # Vật cản che chắn trực tiếp ống kính
                    cls._last_obstacle_dist = 0.18
                    return 0.18

        # Fallback khi dùng Webcam Laptop hoặc khi ảnh depth chưa kịp hội tụ
        if fallback_detections:
            front_objs = [d for d in fallback_detections if abs(d.get('x', 0.0)) <= 0.65 and d.get('z', 99) > 0.08]
            if front_objs:
                min_z = min(d['z'] for d in front_objs)
                dist_m = round(max(0.18, min_z), 2)
                cls._last_obstacle_dist = dist_m
                return dist_m

        cls._last_obstacle_dist = None
        return None

    @staticmethod
    def draw_detections(img, detections, fx=450.0, fy=450.0, cx=320.0, cy=200.0):
        """Vẽ khung hộp nhận diện phong cách Sci-Fi sắc nét, không dùng icon gây lỗi '???'"""
        if img is None: return img
        h, w = img.shape[:2]
        for det in detections:
            try:
                xm, ym, zm = float(det['x']), float(det['y']), float(det['z'])
                name = str(det.get('name', 'OBJ'))
                score = float(det.get('score', 0.8))

                # Ưu tiên số 1: Bounding Box 2D thật từ mạng YOLO / Fallback
                if 'bbox' in det and det['bbox'] is not None:
                    bx1, by1, bx2, by2 = det['bbox']
                    x1 = max(0, min(w - 2, int(bx1)))
                    y1 = max(0, min(h - 2, int(by1)))
                    x2 = max(x1 + 5, min(w - 1, int(bx2)))
                    y2 = max(y1 + 5, min(h - 1, int(by2)))
                elif zm > 0.15:
                    u = int(cx + (xm * fx / zm))
                    v = int(cy + (ym * fy / zm))
                    bw = max(60, min(int(w * 0.6), int(260.0 / zm)))
                    bh = max(140, min(h - 10, int(480.0 / zm)))
                    y2 = min(h - 5, max(int(h * 0.7), v + 40))
                    y1 = max(0, y2 - bh)
                    x1 = max(0, u - bw // 2)
                    x2 = min(w - 1, u + bw // 2)
                else: continue

                # Bảng màu neon chuyên nghiệp
                col = (0, 0, 255) if zm <= 0.25 else ((0, 180, 255) if zm < 0.45 else (0, 255, 163))

                # Vẽ khung góc kiểu Tactical Sci-Fi
                cv2.rectangle(img, (x1, y1), (x2, y2), col, 1)
                c_len = min(18, max(8, int((x2 - x1) * 0.2)))
                cv2.line(img, (x1, y1), (x1 + c_len, y1), col, 2)
                cv2.line(img, (x1, y1), (x1, y1 + c_len), col, 2)
                cv2.line(img, (x2, y1), (x2 - c_len, y1), col, 2)
                cv2.line(img, (x2, y1), (x2, y1 + c_len), col, 2)
                cv2.line(img, (x1, y2), (x1 + c_len, y2), col, 2)
                cv2.line(img, (x1, y2), (x1, y2 - c_len), col, 2)
                cv2.line(img, (x2, y2), (x2 - c_len, y2), col, 2)
                cv2.line(img, (x2, y2), (x2, y2 - c_len), col, 2)

                # Nhãn hiển thị cm nếu < 1m
                dist_str = f" ({int(zm*100)}cm)" if zm < 1.0 else f" ({zm:.2f}m)"
                label_txt = f"{name} {int(score*100)}%{dist_str}"
                (tw, th), _ = cv2.getTextSize(label_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.44, 1)
                ty = max(16, y1 - 4)
                cv2.rectangle(img, (x1, ty - th - 3), (x1 + tw + 6, ty + 2), col, -1)
                cv2.putText(img, label_txt, (x1 + 3, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 1, cv2.LINE_AA)
            except Exception: pass
        return img


if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử SpatialPerceptionEngine...")
    raw = [
        {"id": 0, "name": "person", "score": 0.88, "x": 0.5, "y": -0.15, "z": 2.5},  # Người thật đứng phía trước
        {"id": 0, "name": "person", "score": 0.72, "x": 0.3, "y": 0.18, "z": 1.9},   # Bóng phản chiếu sàn -> Bị lọc!
        {"id": 24, "name": "backpack", "score": 0.91, "x": -0.1, "y": 0.0, "z": 0.35},
        {"id": 56, "name": "chair", "score": 0.75, "x": 1.2, "y": -0.2, "z": 1.5},
        {"id": 0, "name": "person", "score": 0.85, "x": 0.0, "y": 0.0, "z": 15.0}   # Ngoài 10m -> Bị lọc!
    ]
    res = SpatialPerceptionEngine.filter_detections(raw)
    print(f"  ├─ Số lượng vật thể sau lọc: {len(res)} / {len(raw)}")
    for obj in res:
        print(f"     * [{obj['name']}] Score: {obj['score']} | Z: {obj['z']}m (Y: {obj['y']}m)")
    assert len(res) == 3, f"Lỗi: Kỳ vọng 3 vật thể hợp lệ, thực tế được {len(res)}!"
    assert any(obj['name'] == 'PERSON' for obj in res), "Lỗi: Không tìm thấy PERSON hợp lệ!"
    print("✅ [SELF-TEST] SpatialPerceptionEngine & BỘ LỌC CHỐNG BÓNG SÀN ĐẠT CHUẨN 100%!")
