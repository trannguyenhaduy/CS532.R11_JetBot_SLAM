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
    def __init__(self, max_age_seconds=0.25):
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
            min_d = 0.55

            for k, trk in self.tracks.items():
                if trk['name'] == name:
                    d = math.sqrt((trk['x'] - x)**2 + (trk['y'] - y)**2 + (trk['z'] - z)**2)
                    if d < min_d:
                        min_d = d
                        best_k = k

            if best_k is not None:
                old = self.tracks[best_k]
                sx = round(0.75 * x + 0.25 * old['x'], 2)
                sy = round(0.75 * y + 0.25 * old['y'], 2)
                sz = round(0.75 * z + 0.25 * old['z'], 2)
                updated[best_k] = {
                    "id": det.get('id', 0),
                    "name": name,
                    "score": max(det.get('score', 0.8), round(old['score'] * 0.98, 2)),
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

        # Coasting ngắn (0.25s) chỉ để chống giật giữa 2 frame liên tiếp, không giữ lại ghost box
        for k, trk in self.tracks.items():
            if k not in updated and (now - trk['last_seen'] < self.max_age):
                trk['score'] = round(trk['score'] * 0.85, 2)
                if trk['score'] > 0.40 and trk.get('bbox'):
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
    DEPTH_MIN = 0.06  # Hỗ trợ nhận diện cận cảnh sát mũi xe (6cm)
    DEPTH_MAX = 2.00  # Giới hạn bán kính không gian 2.0m chống loạn camera

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
            import torch
            torch.set_num_threads(4)
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

        # Khởi tạo bộ nhận diện người HOG chuẩn OpenCV (chạy 100% offline, cực nhạy)
        try:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        except Exception:
            self.hog = None
        self._last_hog_time = 0.0
        self._cached_hog_boxes = []

    TARGET_SEMANTIC_CLASSES = {
        "PERSON", "OBSTACLE", "UNKNOWN", "CHAIR", "COUCH", "TABLE", "BOTTLE", "CUP", "BACKPACK",
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

            # Lọc nghiêm ngặt bán kính không gian 2.0 mét [6cm đến 2.0m] chống loạn camera
            radial_dist = math.sqrt(x * x + z * z)
            if radial_dist > cls.DEPTH_MAX or z < cls.DEPTH_MIN or z > cls.DEPTH_MAX:
                continue

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

    @staticmethod
    def refine_box_boundary(bgr_img, bbox):
        """
        Tinh chỉnh viền bounding box ôm sát mép vật thể thật:
        Trích xuất cạnh viền Canny cục bộ trong ROI để loại bỏ phần nền rỗng xung quanh.
        """
        if bbox is None or bgr_img is None:
            return bbox
        bx1, by1, bx2, by2 = bbox
        bw = bx2 - bx1
        bh = by2 - by1
        if bw < 25 or bh < 25:
            return bbox

        h_img, w_img = bgr_img.shape[:2]
        pad = 2
        rx1 = max(0, bx1 - pad)
        ry1 = max(0, by1 - pad)
        rx2 = min(w_img, bx2 + pad)
        ry2 = min(h_img, by2 + pad)

        roi_bgr = bgr_img[ry1:ry2, rx1:rx2]
        if roi_bgr.size == 0:
            return bbox

        try:
            gray_roi = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray_roi, 40, 120)
            pts = cv2.findNonZero(edges)
            if pts is not None and len(pts) > 30:
                ex, ey, ew, eh = cv2.boundingRect(pts)
                if ew > bw * 0.60 and eh > bh * 0.60:
                    ref_x1 = max(0, rx1 + ex)
                    ref_y1 = max(0, ry1 + ey)
                    ref_x2 = min(w_img, ref_x1 + ew)
                    ref_y2 = min(h_img, ref_y1 + eh)
                    return [ref_x1, ref_y1, ref_x2, ref_y2]
        except Exception:
            pass

        return bbox

    @staticmethod
    def apply_nms(boxes, iou_threshold=0.25, max_boxes=4):
        """
        Khử trùng lặp thông minh (Smart Merge NMS):
        - Nếu box AI (PERSON, CHAIR...) trùng lặp với box viền Depth, gộp thành 1 box mở rộng ôm trọn cơ thể
        - Loại bỏ box rác, box mảnh vỡ lọt trong box lớn
        - Giới hạn tối đa 4 vật thể nổi bật nhất trong bán kính 2m
        """
        if not boxes:
            return []

        # Chuyển đổi sang định dạng [x1, y1, x2, y2, name, score]
        formatted = []
        for b in boxes:
            bx, by, bw, bh, name, score = b[0], b[1], b[2], b[3], b[4], b[5]
            formatted.append([bx, by, bx + bw, by + bh, name, float(score)])

        # Ưu tiên các box AI và các box có diện tích lớn / score cao
        def sort_key(item):
            is_ai = 2.0 if item[4] not in ["UNKNOWN", "OBSTACLE"] else 1.0
            area = (item[2] - item[0]) * (item[3] - item[1])
            return (is_ai, item[5], area)

        formatted.sort(key=sort_key, reverse=True)

        merged = []
        for b in formatted:
            bx1, by1, bx2, by2, bname, bscore = b
            b_area = max(1, (bx2 - bx1) * (by2 - by1))
            matched_idx = -1

            for idx, m in enumerate(merged):
                mx1, my1, mx2, my2, mname, mscore = m
                m_area = max(1, (mx2 - mx1) * (my2 - my1))

                ix1, iy1 = max(bx1, mx1), max(by1, my1)
                ix2, iy2 = min(bx2, mx2), min(by2, my2)

                if ix2 > ix1 and iy2 > iy1:
                    inter_a = (ix2 - ix1) * (iy2 - iy1)
                    iou = inter_a / float(b_area + m_area - inter_a)
                    contain_ratio = inter_a / float(min(b_area, m_area))

                    # Nếu trùng lấn IoU > 0.25 hoặc một box nằm lọt trong box kia > 60%
                    if iou > iou_threshold or contain_ratio > 0.60:
                        matched_idx = idx
                        break

            if matched_idx >= 0:
                # Hợp nhất: mở rộng viền bao trùm toàn bộ, giữ tên ưu tiên AI
                m = merged[matched_idx]
                nx1 = min(m[0], bx1)
                ny1 = min(m[1], by1)
                nx2 = max(m[2], bx2)
                ny2 = max(m[3], by2)
                # Giữ nhãn AI nếu một trong hai có
                final_name = m[4] if m[4] not in ["UNKNOWN", "OBSTACLE"] else (bname if bname not in ["UNKNOWN", "OBSTACLE"] else "UNKNOWN")
                final_score = max(m[5], bscore)
                merged[matched_idx] = [nx1, ny1, nx2, ny2, final_name, final_score]
            else:
                merged.append(b)

        res_boxes = []
        for m in merged[:max_boxes]:
            res_boxes.append((m[0], m[1], max(10, m[2] - m[0]), max(10, m[3] - m[1]), m[4], m[5]))
        return res_boxes

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
                    resized_bgr = cv2.resize(bgr_img, (infer_w, infer_h), interpolation=cv2.INTER_AREA)
                    rgb = cv2.cvtColor(resized_bgr, cv2.COLOR_BGR2RGB)
                    tensor_img = torch.from_numpy(np.ascontiguousarray(rgb.transpose((2, 0, 1)))).float().div_(255.0)
                    with torch.inference_mode():
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

        # ─── 1.5. HOG PEOPLE DETECTOR (OPENCV CHUẨN, 100% OFFLINE KHÔNG CẦN MODEL NGOÀI) ───
        if self.hog is not None and not any(b[4] == 'PERSON' for b in found_boxes):
            now = time.time()
            if (now - self._last_hog_time >= 0.08) or not self._cached_hog_boxes:
                self._last_hog_time = now
                try:
                    small_gray = cv2.resize(gray, (320, 240))
                    h_boxes, h_weights = self.hog.detectMultiScale(small_gray, winStride=(8, 8), padding=(4, 4), scale=1.05)
                    sx, sy = w / 320.0, h / 240.0
                    cached_hog = []
                    for (bx, by, bw, bh), wgt in zip(h_boxes, h_weights):
                        if wgt > 0.08:
                            rx = max(0, min(w - 2, int(bx * sx)))
                            ry = max(0, min(h - 2, int(by * sy)))
                            rw = max(10, min(w - rx, int(bw * sx)))
                            rh = max(10, min(h - ry, int(bh * sy)))
                            cached_hog.append((rx, ry, rw, rh, "PERSON", min(0.95, float(wgt) + 0.50)))
                    self._cached_hog_boxes = cached_hog
                except Exception:
                    pass
            if self._cached_hog_boxes:
                found_boxes.extend(self._cached_hog_boxes)

        # ─── 1.6. PHÂN ĐOẠN VIỀN CHUẨN XÁC VẬT CẢN TỪ STEREO DEPTH (DUAL-BAND DEPTH SEGMENTATION) ───
        if depth_frame is not None and isinstance(depth_frame, np.ndarray):
            try:
                dh, dw = depth_frame.shape[:2]
                if dh >= 20 and dw >= 20:
                    d_mat = depth_frame.astype(np.float32)
                    if np.issubdtype(depth_frame.dtype, np.floating) and d_mat.max() < 20.0:
                        d_mat *= 1000.0

                    scale_dw, scale_dh = w / 160.0, h / 120.0
                    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

                    # ─── A. DẢI CẬN CẢNH CỰC ĐỘ [60mm đến 750mm] (Vật cản sát mũi xe, người đứng gần) ───
                    # Tuyệt đối KHÔNG lọc sàn nhà ở dải này để không cắt cụt thân dưới của người/vật cản cận cảnh
                    mask_prox = (d_mat >= 60.0) & (d_mat <= 750.0)
                    mask_prox_small = cv2.resize(mask_prox.astype(np.uint8), (160, 120), interpolation=cv2.INTER_NEAREST)
                    mask_prox_clean = cv2.morphologyEx(mask_prox_small, cv2.MORPH_CLOSE, kernel_close)

                    res_p = cv2.findContours(mask_prox_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cnts_p = res_p[0] if len(res_p) == 2 else res_p[1]

                    for cp in cnts_p:
                        area_p = cv2.contourArea(cp)
                        # Vật cản cận cảnh chiếm từ 1.2% khung hình
                        if area_p > (160 * 120 * 0.012):
                            dbx, dby, dbw, dbh = cv2.boundingRect(cp)
                            real_bx = max(0, min(w - 2, int(dbx * scale_dw)))
                            real_by = max(0, min(h - 2, int(dby * scale_dh)))
                            real_bw = max(10, min(w - real_bx, int(dbw * scale_dw)))
                            real_bh = max(10, min(h - real_by, int(dbh * scale_dh)))
                            found_boxes.append((real_bx, real_by, real_bw, real_bh, "UNKNOWN", 0.90))

                    # ─── B. DẢI TRUNG CẢNH [750mm đến 2000mm] (Bàn, ghế, balô, người ở cự ly xa hơn) ───
                    mask_mid = (d_mat > 750.0) & (d_mat <= 2000.0)
                    # Lọc sàn nhà chỉ khi ở nửa dưới khung hình và xa
                    y_indices, _ = np.indices((dh, dw))
                    y_cam = (y_indices - (cy * dh / float(h))) * (d_mat / 1000.0) / (fy * dh / float(h))
                    z_rob = -y_cam + 0.12
                    is_floor = (d_mat > 800.0) & (y_indices > (dh * 0.65)) & (z_rob < 0.01)
                    mask_mid = mask_mid & (~is_floor)

                    mask_mid_small = cv2.resize(mask_mid.astype(np.uint8), (160, 120), interpolation=cv2.INTER_NEAREST)
                    kernel_mid = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                    mask_mid_clean = cv2.morphologyEx(mask_mid_small, cv2.MORPH_OPEN, kernel_mid)

                    res_m = cv2.findContours(mask_mid_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cnts_m = res_m[0] if len(res_m) == 2 else res_m[1]

                    for cm in cnts_m:
                        area_m = cv2.contourArea(cm)
                        if area_m > (160 * 120 * 0.018):
                            dbx, dby, dbw, dbh = cv2.boundingRect(cm)
                            real_bx = max(0, min(w - 2, int(dbx * scale_dw)))
                            real_by = max(0, min(h - 2, int(dby * scale_dh)))
                            real_bw = max(10, min(w - real_bx, int(dbw * scale_dw)))
                            real_bh = max(10, min(h - real_by, int(dbh * scale_dh)))
                            found_boxes.append((real_bx, real_by, real_bw, real_bh, "UNKNOWN", 0.82))

                    # ─── C. XỬ LÝ ĐIỂM MÙ STEREO CẬN CẢNH (< 18cm) ───
                    # Khi vật cản hoặc người đứng quá sát camera (< 18cm), stereo không tính được độ sâu (depth == 0)
                    center_d = d_mat[int(dh*0.15):int(dh*0.85), int(dw*0.15):int(dw*0.85)]
                    if center_d.size > 0:
                        blind_ratio = np.count_nonzero(center_d == 0) / float(center_d.size)
                        close_ratio = np.count_nonzero((center_d >= 50.0) & (center_d <= 400.0)) / float(center_d.size)
                        if (close_ratio > 0.25) or (blind_ratio > 0.40 and np.count_nonzero((center_d > 0) & (center_d <= 450.0)) > 15):
                            if not any(fb[4] in ["PERSON", "UNKNOWN"] for fb in found_boxes):
                                cw, ch = int(w * 0.75), int(h * 0.75)
                                cx1, cy1 = (w - cw) // 2, (h - ch) // 2
                                found_boxes.append((cx1, cy1, cw, ch, "UNKNOWN", 0.92))
            except Exception:
                pass

        # ─── 2. BỔ TRỢ KHUÔN MẶT HAAR CASCADE (KHI NGỒI GẦN WEBCAM) ───
        if self.face_cascade is not None and not any(b[4] == 'PERSON' for b in found_boxes):
            try:
                faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=4, minSize=(60, 60))
                for (fx1, fy1, fw, fh) in faces:
                    cx_face = fx1 + fw / 2.0
                    if abs(cx_face - w / 2.0) < (w * 0.42):
                        found_boxes.append((fx1, fy1, fw, fh, "PERSON", 0.95))
            except Exception:
                pass

        # ─── 2.5. PHÁT HIỆN VẬT CẢN ÁP SÁT TỪ ĐẦU (PROXIMITY VISUAL FALLBACK KHI AI KHÔNG BẮT KỊP) ───
        if not found_boxes:
            # Khi vật thể (người, bàn tay, mép cản) ở sát camera từ đầu, chiếm trọn tâm nhìn
            center_gray = gray[int(h*0.12):int(h*0.88), int(w*0.12):int(w*0.88)]
            canny_c = cv2.Canny(center_gray, 35, 120)
            pts_c = cv2.findNonZero(canny_c)
            if pts_c is not None and len(pts_c) > 180:
                cx_rel, cy_rel, cw_rel, ch_rel = cv2.boundingRect(pts_c)
                if cw_rel > (w * 0.28) and ch_rel > (h * 0.22):
                    rx = int(w * 0.12) + cx_rel
                    ry = int(h * 0.12) + cy_rel
                    found_boxes.append((rx, ry, cw_rel, ch_rel, "UNKNOWN", 0.85))

        found_boxes = self.apply_nms(found_boxes, iou_threshold=0.25, max_boxes=4)

        detections = []
        for (bx, by, bw, bh, name, score) in found_boxes:
            # Tinh chỉnh viền bounding box ôm khít mép thật của vật thể (bỏ vùng nền thừa)
            tight_bbox = [bx, by, bx + bw, by + bh]
            if name != "UNKNOWN":
                tight_bbox = self.refine_box_boundary(bgr_img, tight_bbox)
                bx, by = tight_bbox[0], tight_bbox[1]
                bw, bh = max(10, tight_bbox[2] - bx), max(10, tight_bbox[3] - by)

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
                    valid = roi[(roi > 50) & (roi <= 2000)]
                    if len(valid) > 10:
                        z_m = float(np.median(valid)) / 1000.0
                except Exception: pass

            # Tính cự ly quang học nếu chưa có depth
            if z_m < 0.06 or z_m > 2.0:
                d_optical = max(bw, int(bh * 0.65))
                z_m = round(float(fx * 0.26 / max(d_optical, 1)), 2)
                z_m = max(0.12, min(1.95, z_m))

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
        - Lọc các điểm đo vật lý hợp lệ từ 50mm đến 2000mm
        - Lấy phân vị 15% để bắt mép cản gần nhất
        - Chống điểm mù stereo: Khi cản áp sát < 18cm, giữ cự ly phanh khẩn cấp 0.15m
        - Fallback: Trích xuất cự ly Z nhỏ nhất từ các đối tượng 3D trước mặt
        """
        if depth_frame is not None and isinstance(depth_frame, np.ndarray):
            dh, dw = depth_frame.shape[:2]
            if dh >= 10 and dw >= 10:
                # Giới hạn 10% đến 40% chiều cao để không chạm sàn nhà
                h_start, h_end = int(dh * 0.10), int(dh * 0.40)
                w_start, w_end = int(dw * 0.25), int(dw * 0.75)
                roi = depth_frame[h_start:h_end, w_start:w_end]

                if np.issubdtype(roi.dtype, np.floating):
                    roi = np.nan_to_num(roi, nan=0.0, posinf=0.0, neginf=0.0)

                if roi.dtype == np.uint16 or (roi.size > 0 and roi.max() > 50.0):
                    roi_mm = roi.astype(np.float32)
                else:
                    roi_mm = roi.astype(np.float32) * 1000.0

                valid = roi_mm[(roi_mm >= 50.0) & (roi_mm <= 2000.0)]
                if len(valid) >= 15:
                    dist_mm = float(np.percentile(valid, 15))
                    dist_m = round(max(0.08, dist_mm / 1000.0), 2)
                    cls._last_obstacle_dist = dist_m
                    return dist_m
                elif roi_mm.size > 0 and (np.count_nonzero(roi_mm < 80.0) / float(roi_mm.size)) > 0.35 and (cls._last_obstacle_dist is not None and cls._last_obstacle_dist <= 0.45):
                    # Điểm mù stereo: vật cản che kín phía trước ở cự ly áp sát
                    cls._last_obstacle_dist = 0.15
                    return 0.15

        # Fallback khi dùng Webcam Laptop hoặc khi ảnh depth chưa kịp hội tụ
        if fallback_detections:
            front_objs = [d for d in fallback_detections if abs(d.get('x', 0.0)) <= 0.65 and 0.06 <= d.get('z', 99) <= 2.0]
            if front_objs:
                min_z = min(d['z'] for d in front_objs)
                dist_m = round(max(0.08, min_z), 2)
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
                if zm > 2.0 or (xm * xm + zm * zm > 4.0):
                    continue

                name = str(det.get('name', 'OBJ'))
                score = float(det.get('score', 0.8))

                # CHỈ VẼ KHI CÓ BOUNDING BOX THẬT TỪ MẠNG AI HOẶC PHÂN ĐOẠN DEPTH
                if 'bbox' in det and det['bbox'] is not None:
                    bx1, by1, bx2, by2 = det['bbox']
                    x1 = max(0, min(w - 2, int(bx1)))
                    y1 = max(0, min(h - 2, int(by1)))
                    x2 = max(x1 + 5, min(w - 1, int(bx2)))
                    y2 = max(y1 + 5, min(h - 1, int(by2)))
                else:
                    continue

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
                label_txt = f"{name}{dist_str}" if name in ["UNKNOWN", "OBSTACLE"] else f"{name} {int(score*100)}%{dist_str}"
                (tw, th), _ = cv2.getTextSize(label_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.44, 1)
                ty = max(16, y1 - 4)
                cv2.rectangle(img, (x1, ty - th - 3), (x1 + tw + 6, ty + 2), col, -1)
                cv2.putText(img, label_txt, (x1 + 3, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 1, cv2.LINE_AA)
            except Exception: pass
        return img


if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử SpatialPerceptionEngine...")
    raw = [
        {"id": 0, "name": "person", "score": 0.88, "x": 0.5, "y": -0.15, "z": 1.6},  # Người thật đứng phía trước (trong bán kính 2m)
        {"id": 0, "name": "person", "score": 0.72, "x": 0.3, "y": 0.18, "z": 1.4},   # Bóng phản chiếu sàn -> Bị lọc!
        {"id": 24, "name": "backpack", "score": 0.91, "x": -0.1, "y": 0.0, "z": 0.35}, # Ba lô gần 35cm
        {"id": 56, "name": "chair", "score": 0.75, "x": 0.8, "y": -0.2, "z": 1.2},   # Ghế 1.2m
        {"id": 0, "name": "person", "score": 0.85, "x": 0.0, "y": 0.0, "z": 3.5}    # Ngoài 2.0m -> Bị lọc!
    ]
    res = SpatialPerceptionEngine.filter_detections(raw)
    print(f"  ├─ Số lượng vật thể sau lọc: {len(res)} / {len(raw)}")
    for obj in res:
        print(f"     * [{obj['name']}] Score: {obj['score']} | Z: {obj['z']}m (Y: {obj['y']}m)")
    assert len(res) == 3, f"Lỗi: Kỳ vọng 3 vật thể hợp lệ, thực tế được {len(res)}!"
    assert any(obj['name'] == 'PERSON' for obj in res), "Lỗi: Không tìm thấy PERSON hợp lệ!"
    print("✅ [SELF-TEST] SpatialPerceptionEngine & BỘ LỌC CHỐNG BÓNG SÀN ĐẠT CHUẨN 100%!")
