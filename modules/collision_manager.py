#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
MODULE COLLISION AVOIDANCE MANAGER (NVIDIA JETBOT DEEP LEARNING WORKFLOW)
Quản lý trọn gói quy trình Tránh Vật Cản theo chuẩn NVIDIA JetBot AI:
  1. Thu thập dữ liệu (Data Collection): Chụp, gán nhãn Free/Blocked, đếm ảnh, undo, zip.
  2. Huấn luyện Mô hình (Deep Learning Training): ResNet-18 / MobileNetV2 Transfer Learning.
  3. Suy luận thời gian thực (Live Inference & Autopilot): Đo xác suất vật cản và né tránh.
══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import uuid
import glob
import zipfile
import threading
import numpy as np
import cv2

TORCH_IMPORT_ERROR = None
try:
    import torch
    import torch.nn as nn
    HAS_TORCH = True
except Exception as _e_torch:
    HAS_TORCH = False
    TORCH_IMPORT_ERROR = str(_e_torch)

HAS_TORCHVISION = False
if HAS_TORCH:
    try:
        import torchvision
        import torchvision.transforms as transforms
        from torchvision.datasets import ImageFolder
        from torch.utils.data import DataLoader, random_split
        HAS_TORCHVISION = True
    except Exception as _e_tv:
        HAS_TORCHVISION = False
        if not TORCH_IMPORT_ERROR:
            TORCH_IMPORT_ERROR = f"Thiếu torchvision: {_e_tv}"


class CollisionManager:
    """Bộ điều phối toàn diện cho bài toán AI Collision Avoidance của JetBot"""

    def __init__(self, workspace_dir=None):
        if workspace_dir is None:
            workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.workspace_dir = workspace_dir

        self.dataset_dir = os.path.join(self.workspace_dir, "dataset", "collision")
        self.free_dir = os.path.join(self.dataset_dir, "free")
        self.blocked_dir = os.path.join(self.dataset_dir, "blocked")
        self.models_dir = os.path.join(self.workspace_dir, "models")

        # Đảm bảo các thư mục luôn sẵn sàng
        for d in [self.free_dir, self.blocked_dir, self.models_dir]:
            os.makedirs(d, exist_ok=True)

        self.model_path = os.path.join(self.models_dir, "best_collision_model.pth")
        self.model = None
        self.device = torch.device('cuda' if (HAS_TORCH and torch.cuda.is_available()) else 'cpu') if HAS_TORCH else None

        # Trạng thái ảnh chụp
        self.last_saved_filepath = None
        self.lock = threading.Lock()

        # Trạng thái Huấn luyện
        self.is_training = False
        self.train_progress = {
            "epoch": 0,
            "total_epochs": 0,
            "train_loss": 0.0,
            "val_loss": 0.0,
            "val_acc": 0.0,
            "status": "SẴN SÀNG",
            "percent": 0,
            "error": None
        }

        # Trạng thái Tự hành né vật cản (Live Autopilot)
        self.is_avoiding = False
        self.last_prob_blocked = 0.0
        self.last_prob_free = 1.0
        self.last_action_desc = "CHỜ KÍCH HOẠT"
        self.avoid_turn_dir = 1.0  # 1: quay trái CCW, -1: quay phải CW

        # Tự động nạp model có sẵn nếu tồn tại
        self._try_load_model()

    # ══════════════════════════════════════════════════════════════════════════
    # 1. THU THẬP DỮ LIỆU (DATA COLLECTION & DATASET MANAGEMENT)
    # ══════════════════════════════════════════════════════════════════════════

    def get_counts(self):
        """Đếm số lượng ảnh trong từng nhãn dataset thời gian thực"""
        try:
            free_cnt = len(glob.glob(os.path.join(self.free_dir, "*.jpg")))
            blocked_cnt = len(glob.glob(os.path.join(self.blocked_dir, "*.jpg")))
            return {
                "free": free_cnt,
                "blocked": blocked_cnt,
                "total": free_cnt + blocked_cnt
            }
        except Exception as e:
            return {"free": 0, "blocked": 0, "total": 0, "error": str(e)}

    def save_sample(self, frame_bgr, label='free'):
        """
        Lưu một mẫu ảnh vào dataset theo nhãn 'free' (đường thoáng) hoặc 'blocked' (vật cản).
        Chuẩn hóa kích thước 224x224 (tiêu chuẩn vàng của ResNet18 NVIDIA JetBot).
        """
        if frame_bgr is None or frame_bgr.size == 0:
            return False, "Không có khung hình camera", self.get_counts()

        label = str(label).lower().strip()
        target_dir = self.blocked_dir if label == 'blocked' else self.free_dir

        try:
            # Resize về 224x224 chuẩn đầu vào nơ-ron
            img_resized = cv2.resize(frame_bgr, (224, 224), interpolation=cv2.INTER_AREA)

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            unique_id = uuid.uuid4().hex[:6]
            filename = f"{label}_{timestamp}_{unique_id}.jpg"
            filepath = os.path.join(target_dir, filename)

            # Lưu ảnh JPEG với chất lượng 95
            cv2.imwrite(filepath, img_resized, [cv2.IMWRITE_JPEG_QUALITY, 95])

            with self.lock:
                self.last_saved_filepath = filepath

            counts = self.get_counts()
            print(f"📸 [DATA COLLISION] Đã thêm mẫu [{label.upper()}]: {filename} -> Tổng: Free={counts['free']}, Blocked={counts['blocked']}")
            return True, filename, counts
        except Exception as e:
            print(f"❌ [DATA COLLISION] Lỗi lưu mẫu: {e}")
            return False, str(e), self.get_counts()

    def undo_last_sample(self):
        """Xóa ảnh vừa chụp gần nhất (nếu người dùng bấm nhầm)"""
        with self.lock:
            fp = self.last_saved_filepath

        if not fp or not os.path.exists(fp):
            return False, "Không có ảnh vừa chụp nào để hoàn tác", self.get_counts()

        try:
            fname = os.path.basename(fp)
            os.remove(fp)
            with self.lock:
                self.last_saved_filepath = None
            counts = self.get_counts()
            print(f"↩️ [DATA COLLISION] Đã hoàn tác/xóa ảnh: {fname}")
            return True, f"Đã xóa ảnh {fname}", counts
        except Exception as e:
            return False, str(e), self.get_counts()

    def clear_dataset(self, label=None):
        """Xóa toàn bộ ảnh trong dataset hoặc trong nhãn chỉ định"""
        targets = []
        if label == 'free':
            targets = [self.free_dir]
        elif label == 'blocked':
            targets = [self.blocked_dir]
        else:
            targets = [self.free_dir, self.blocked_dir]

        deleted_count = 0
        for d in targets:
            files = glob.glob(os.path.join(d, "*.jpg"))
            for f in files:
                try:
                    os.remove(f)
                    deleted_count += 1
                except Exception:
                    pass

        with self.lock:
            self.last_saved_filepath = None

        return True, f"Đã xóa {deleted_count} ảnh", self.get_counts()

    def export_dataset_zip(self):
        """Nén toàn bộ dataset thành file ZIP để tải về máy tính cá nhân hoặc Google Colab"""
        zip_path = os.path.join(self.workspace_dir, "dataset_collision_jetbot.zip")
        try:
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                for root, _, files in os.walk(self.dataset_dir):
                    for file in files:
                        if file.endswith('.jpg'):
                            abs_p = os.path.join(root, file)
                            rel_p = os.path.relpath(abs_p, self.dataset_dir)
                            zipf.write(abs_p, os.path.join("collision", rel_p))
            return True, zip_path
        except Exception as e:
            return False, str(e)

    # ══════════════════════════════════════════════════════════════════════════
    # 2. HUẤN LUYỆN MÔ HÌNH (DEEP LEARNING MODEL TRAINING)
    # ══════════════════════════════════════════════════════════════════════════

    def start_training(self, epochs=15, batch_size=8, lr=0.001, model_type='resnet18'):
        """Kích hoạt tiến trình huấn luyện PyTorch trong luồng nền (non-blocking)"""
        if not HAS_TORCH:
            return False, "Hệ thống chưa cài đặt PyTorch/Torchvision"

        if self.is_training:
            return False, "Tiến trình huấn luyện đang chạy"

        counts = self.get_counts()
        if counts['free'] < 5 or counts['blocked'] < 5:
            return False, f"Dữ liệu quá ít! Cần tối thiểu 5 ảnh mỗi nhãn (Hiện có: Free={counts['free']}, Blocked={counts['blocked']})"

        thread = threading.Thread(
            target=self._train_worker,
            args=(int(epochs), int(batch_size), float(lr), str(model_type)),
            daemon=True
        )
        thread.start()
        return True, f"Bắt đầu huấn luyện mô hình {model_type.upper()} ({epochs} epochs)..."

    def _train_worker(self, epochs, batch_size, lr, model_type):
        """Worker huấn luyện PyTorch Transfer Learning"""
        self.is_training = True
        self.train_progress = {
            "epoch": 0,
            "total_epochs": epochs,
            "train_loss": 0.0,
            "val_loss": 0.0,
            "val_acc": 0.0,
            "status": "CHUẨN BỊ DỮ LIỆU",
            "percent": 0,
            "error": None
        }

        print(f"\n🚀 [TRAIN COLLISION] Khởi động Huấn luyện Model: {model_type.upper()} | Epochs: {epochs} | Batch: {batch_size} | Thiết bị: {self.device}")

        try:
            # 1. Pipeline Augmentation & Preprocessing
            train_transform = transforms.Compose([
                transforms.Resize((224, 224)),
                transforms.RandomHorizontalFlip(p=0.5),
                transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
            ])

            val_transform = transforms.Compose([
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
            ])

            # 2. Đọc Dataset
            dataset = ImageFolder(self.dataset_dir, transform=train_transform)
            class_to_idx = dataset.class_to_idx
            print(f"📦 [DATASET] Các lớp nhận diện: {class_to_idx}")

            total_samples = len(dataset)
            val_size = max(1, int(0.20 * total_samples))
            train_size = total_samples - val_size

            train_ds, val_ds = random_split(
                dataset,
                [train_size, val_size],
                generator=torch.Generator().manual_seed(42)
            )

            train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True, drop_last=False)
            val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False)

            # 3. Khởi tạo Kiến trúc Mạng Nơ-ron (Transfer Learning)
            if 'mobilenet' in model_type.lower():
                try:
                    weights = torchvision.models.MobileNet_V2_Weights.DEFAULT
                    net = torchvision.models.mobilenet_v2(weights=weights)
                except Exception:
                    net = torchvision.models.mobilenet_v2(pretrained=True)
                # Đóng băng feature extractor
                for param in net.features.parameters():
                    param.requires_grad = False
                in_f = net.classifier[1].in_features
                net.classifier[1] = nn.Linear(in_f, 2)
            else:
                try:
                    weights = torchvision.models.ResNet18_Weights.DEFAULT
                    net = torchvision.models.resnet18(weights=weights)
                except Exception:
                    net = torchvision.models.resnet18(pretrained=True)
                for param in net.parameters():
                    param.requires_grad = False
                in_f = net.fc.in_features
                net.fc = nn.Linear(in_f, 2)

            net = net.to(self.device)

            criterion = nn.CrossEntropyLoss()
            optimizer = torch.optim.Adam(filter(lambda p: p.requires_grad, net.parameters()), lr=lr)

            best_val_acc = 0.0

            # 4. Vòng lặp Huấn luyện qua từng Epoch
            for epoch in range(1, epochs + 1):
                net.train()
                train_loss = 0.0
                correct_train = 0
                total_train = 0

                for imgs, labels in train_loader:
                    imgs, labels = imgs.to(self.device), labels.to(self.device)
                    optimizer.zero_grad()
                    outputs = net(imgs)
                    loss = criterion(outputs, labels)
                    loss.backward()
                    optimizer.step()

                    train_loss += loss.item() * imgs.size(0)
                    _, preds = torch.max(outputs, 1)
                    correct_train += torch.sum(preds == labels.data).item()
                    total_train += imgs.size(0)

                train_loss_epoch = train_loss / max(1, total_train)

                # Validation Loop
                net.eval()
                val_loss = 0.0
                correct_val = 0
                total_val = 0
                with torch.no_grad():
                    for imgs, labels in val_loader:
                        imgs, labels = imgs.to(self.device), labels.to(self.device)
                        outputs = net(imgs)
                        loss = criterion(outputs, labels)
                        val_loss += loss.item() * imgs.size(0)
                        _, preds = torch.max(outputs, 1)
                        correct_val += torch.sum(preds == labels.data).item()
                        total_val += imgs.size(0)

                val_loss_epoch = val_loss / max(1, total_val)
                val_acc_epoch = (correct_val / max(1, total_val)) * 100.0

                pct = int((epoch / epochs) * 100)
                self.train_progress = {
                    "epoch": epoch,
                    "total_epochs": epochs,
                    "train_loss": round(train_loss_epoch, 4),
                    "val_loss": round(val_loss_epoch, 4),
                    "val_acc": round(val_acc_epoch, 1),
                    "status": f"EPOCH {epoch}/{epochs} ({pct}%)",
                    "percent": pct,
                    "error": None
                }

                print(f"🔥 Epoch [{epoch}/{epochs}]: Train Loss={train_loss_epoch:.4f} | Val Loss={val_loss_epoch:.4f} | Val Acc={val_acc_epoch:.1f}%")

                # Lưu checkpoint tốt nhất
                if val_acc_epoch >= best_val_acc:
                    best_val_acc = val_acc_epoch
                    torch.save(net.state_dict(), self.model_path)
                    print(f"💾 [SAVED] Đã lưu mô hình tối ưu ({val_acc_epoch:.1f}%) vào {self.model_path}")

            self.train_progress["status"] = f"HOÀN TẤT ({best_val_acc:.1f}% Acc)"
            print(f"✅ [TRAIN COMPLETE] Huấn luyện thành công! Độ chính xác tối ưu: {best_val_acc:.1f}%")

            # Nạp ngay model vừa train xong vào bộ nhớ để sẵn sàng chạy
            self._try_load_model()

        except Exception as e:
            print(f"❌ [TRAIN ERROR]: {e}")
            self.train_progress["status"] = "LỖI HUẤN LUYỆN"
            self.train_progress["error"] = str(e)
        finally:
            self.is_training = False

    # ══════════════════════════════════════════════════════════════════════════
    # 3. SUY LUẬN & ĐIỀU KHIỂN TỰ HÀNH NÉ VẬT CẢN (LIVE AUTOPILOT)
    # ══════════════════════════════════════════════════════════════════════════

    def _build_resnet18_native(self, num_classes=2):
        """Khung mạng ResNet18 thuần PyTorch, không phụ thuộc vào thư viện torchvision"""
        class BasicBlock(nn.Module):
            expansion = 1
            def __init__(self, in_planes, planes, stride=1, downsample=None):
                super().__init__()
                self.conv1 = nn.Conv2d(in_planes, planes, kernel_size=3, stride=stride, padding=1, bias=False)
                self.bn1 = nn.BatchNorm2d(planes)
                self.relu = nn.ReLU(inplace=True)
                self.conv2 = nn.Conv2d(planes, planes, kernel_size=3, stride=1, padding=1, bias=False)
                self.bn2 = nn.BatchNorm2d(planes)
                self.downsample = downsample
            def forward(self, x):
                identity = x
                out = self.relu(self.bn1(self.conv1(x)))
                out = self.bn2(self.conv2(out))
                if self.downsample is not None:
                    identity = self.downsample(x)
                out += identity
                return self.relu(out)

        class ResNetNative(nn.Module):
            def __init__(self, block, layers, num_classes=2):
                super().__init__()
                self.in_planes = 64
                self.conv1 = nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3, bias=False)
                self.bn1 = nn.BatchNorm2d(64)
                self.relu = nn.ReLU(inplace=True)
                self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
                self.layer1 = self._make_layer(block, 64, layers[0])
                self.layer2 = self._make_layer(block, 128, layers[1], stride=2)
                self.layer3 = self._make_layer(block, 256, layers[2], stride=2)
                self.layer4 = self._make_layer(block, 512, layers[3], stride=2)
                self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
                self.fc = nn.Linear(512 * block.expansion, num_classes)
            def _make_layer(self, block, planes, blocks, stride=1):
                downsample = None
                if stride != 1 or self.in_planes != planes * block.expansion:
                    downsample = nn.Sequential(
                        nn.Conv2d(self.in_planes, planes * block.expansion, kernel_size=1, stride=stride, bias=False),
                        nn.BatchNorm2d(planes * block.expansion),
                    )
                layers = [block(self.in_planes, planes, stride, downsample)]
                self.in_planes = planes * block.expansion
                for _ in range(1, blocks):
                    layers.append(block(self.in_planes, planes))
                return nn.Sequential(*layers)
            def forward(self, x):
                x = self.maxpool(self.relu(self.bn1(self.conv1(x))))
                x = self.layer4(self.layer3(self.layer2(self.layer1(x))))
                x = self.avgpool(x)
                x = torch.flatten(x, 1)
                return self.fc(x)

        return ResNetNative(BasicBlock, [2, 2, 2, 2], num_classes=num_classes)

    def _try_load_model(self):
        """Nạp model PyTorch đã train từ đĩa"""
        if not HAS_TORCH:
            if not getattr(self, '_logged_torch_missing', False):
                print(f"⚠️ [AI COLLISION] Không thể nạp mô hình: {TORCH_IMPORT_ERROR}")
                self._logged_torch_missing = True
            return False

        if not os.path.exists(self.model_path):
            if not getattr(self, '_logged_model_missing', False):
                print(f"⚠️ [AI COLLISION] Chưa tìm thấy file mô hình tại: {self.model_path}. Hãy chạy 'git pull' trên robot!")
                self._logged_model_missing = True
            return False

        try:
            # 1. Thử dùng torchvision nếu có sẵn
            net = None
            if HAS_TORCHVISION:
                try:
                    weights = torchvision.models.ResNet18_Weights.DEFAULT
                    net = torchvision.models.resnet18(weights=weights)
                except Exception:
                    try:
                        net = torchvision.models.resnet18(pretrained=False)
                    except Exception:
                        net = None
                if net is not None:
                    net.fc = nn.Linear(net.fc.in_features, 2)

            # 2. Dự phòng: Dùng khung ResNet18 thuần PyTorch (chạy mượt 100% không cần cài torchvision)
            if net is None:
                net = self._build_resnet18_native(num_classes=2)

            state_dict = torch.load(self.model_path, map_location=self.device)
            net.load_state_dict(state_dict)
            net = net.to(self.device)
            net.eval()

            self.model = net
            print(f"🧠 [AI COLLISION] Đã nạp thành công mô hình: {self.model_path} trên {self.device}")
            return True
        except Exception as e:
            print(f"⚠️ [AI COLLISION] Không thể nạp mô hình: {e}")
            self.model = None
            return False

    def predict(self, frame_bgr):
        """
        Dự đoán xác suất [P(free), P(blocked)] từ frame camera hiện tại.
        Trả về: (prob_free, prob_blocked)
        """
        if self.model is None:
            # Tự động nạp lại mô hình nếu file vừa được tải về
            if not self._try_load_model():
                return 1.0, 0.0

        if frame_bgr is None or frame_bgr.size == 0:
            return 1.0, 0.0

        try:
            # Tiền xử lý nhanh bằng OpenCV + NumPy:
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(rgb, (224, 224), interpolation=cv2.INTER_LINEAR)
            img_norm = (resized.astype(np.float32) / 255.0 - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
            tensor = torch.from_numpy(img_norm.transpose(2, 0, 1)).unsqueeze(0).float().to(self.device)

            with torch.no_grad():
                output = self.model(tensor)
                probs = torch.softmax(output, dim=1).cpu().numpy()[0]

            # Chú ý: ImageFolder sắp xếp theo alphabet: [0: 'blocked', 1: 'free']
            prob_blocked = float(probs[0])
            prob_free = float(probs[1])

            self.last_prob_blocked = round(prob_blocked, 3)
            self.last_prob_free = round(prob_free, 3)

            return self.last_prob_free, self.last_prob_blocked
        except Exception as e:
            return 1.0, 0.0

    def compute_avoidance_cmd(self, prob_blocked, base_speed=0.045, turn_speed=0.85):
        """
        Tính toán lệnh vận tốc vi sai (v, w) theo xác suất vật cản P(blocked):
        - Tốc độ tiến dịu dàng (0.045 m/s) giúp triệt tiêu nhòe camera OAK, quét vật cản chuẩn nét.
        - Tốc độ quay êm ái (0.85 rad/s) giúp robot chuyển hướng đầm, không văng lệch khung hình.
        - Phản xạ nhạy: P(blocked) >= 0.30 là dừng hẳn tiến và quay né.
        - Hysteresis: Quay đến khi góc nhìn thoáng sạch (< 18%) mới tiến tiếp.
        """
        # Nếu chưa bật tự hành
        if not self.is_avoiding:
            return None, None, "CHƯA BẬT TỰ HÀNH"

        if not hasattr(self, '_is_turning_away'):
            self._is_turning_away = False

        # ── 1. PHÁT HIỆN CẢN: DỪNG TIẾN VÀ QUAY TẠI CHỖ NÉ ──
        if prob_blocked >= 0.30 or (self._is_turning_away and prob_blocked >= 0.18):
            self._is_turning_away = True
            v = 0.0  # Dừng hẳn tiến để không đâm vào cản
            w = turn_speed * self.avoid_turn_dir
            side_str = "TRÁI ↺" if self.avoid_turn_dir > 0 else "PHẢI ↻"
            desc = f"🛑 GẶP CẢN ({int(prob_blocked*100)}%) -> QUAY {side_str}"
        # ── 2. ĐƯỜNG ĐÃ THẬT SỰ THOÁNG: TIẾN THẲNG TỪ TỐN ──
        else:
            self._is_turning_away = False
            v = base_speed
            w = 0.0
            desc = f"TIẾN THẲNG (Đường thoáng {int(self.last_prob_free*100)}%)"

        self.last_action_desc = desc
        return v, w, desc

    def toggle_avoidance(self):
        """Bật / Tắt chế độ Tự hành né vật cản"""
        self.is_avoiding = not self.is_avoiding
        if self.is_avoiding:
            # Đổi ngẫu nhiên chiều quay né tránh (trái hoặc phải) mỗi lần bắt đầu
            self.avoid_turn_dir = 1.0 if (time.time() % 2 > 1.0) else -1.0
            self.last_action_desc = "ĐANG TỰ HÀNH NÉ VẬT CẢN"
        else:
            self.last_action_desc = "ĐÃ TẮT TỰ HÀNH"
        return self.is_avoiding

    def get_status(self):
        """Tổng hợp toàn bộ trạng thái hệ thống cho Web API"""
        counts = self.get_counts()
        return {
            "counts": counts,
            "has_torch": HAS_TORCH,
            "has_model": (self.model is not None),
            "model_path": self.model_path if (self.model is not None) else None,
            "device": str(self.device) if self.device else "N/A",
            "is_training": self.is_training,
            "train_progress": self.train_progress,
            "is_avoiding": self.is_avoiding,
            "prob_blocked": self.last_prob_blocked,
            "prob_free": self.last_prob_free,
            "action_desc": self.last_action_desc
        }
