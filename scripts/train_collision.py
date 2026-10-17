#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
KỊCH BẢN TỰ HUẤN LUYỆN MÔ HÌNH AI NÉ VẬT CẢN (NVIDIA JETBOT COLLISION AVOIDANCE)
══════════════════════════════════════════════════════════════════════════════
Cách chạy:
  1. Chạy mặc định trên JetBot hoặc PC:
     python scripts/train_collision.py
  2. Tùy chỉnh tham số (epochs, batch, model):
     python scripts/train_collision.py --epochs 20 --batch_size 8 --model resnet18 --lr 0.001
══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import argparse
import time

# Đảm bảo import được các module từ thư mục gốc
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from modules.collision_manager import CollisionManager


def main():
    parser = argparse.ArgumentParser(description="JetBot AI Collision Avoidance Trainer")
    parser.add_argument("--epochs", type=int, default=15, help="Số lượt huấn luyện (default: 15)")
    parser.add_argument("--batch_size", type=int, default=8, help="Kích thước batch (default: 8)")
    parser.add_argument("--lr", type=float, default=0.001, help="Tốc độ học Learning Rate (default: 0.001)")
    parser.add_argument("--model", type=str, default="resnet18", choices=["resnet18", "mobilenet_v2"], help="Kiến trúc mạng (default: resnet18)")
    args = parser.parse_args()

    print("\n" + "═" * 70)
    print("🤖 JETBOT DEEP LEARNING COLLISION AVOIDANCE TRAINER")
    print("═" * 70)

    mgr = CollisionManager(workspace_dir=ROOT_DIR)
    counts = mgr.get_counts()

    print(f"📊 Dữ liệu hiện có: {counts['free']} ảnh Free (thoáng) | {counts['blocked']} ảnh Blocked (vật cản)")

    if counts['free'] < 5 or counts['blocked'] < 5:
        print("\n⚠️ [CHƯA ĐỦ DỮ LIỆU]")
        print("  Bạn cần thu thập tối thiểu 5 ảnh cho mỗi nhãn 'free' và 'blocked'.")
        print("  💡 Hãy mở Web Cockpit (http://<ip-xe>:8080) và dùng phím F (Free) / B (Blocked) để chụp ảnh trước!")
        print("  Hoặc sao chép ảnh vào thư mục:")
        print(f"    - {mgr.free_dir}")
        print(f"    - {mgr.blocked_dir}\n")
        return

    ok, msg = mgr.start_training(epochs=args.epochs, batch_size=args.batch_size, lr=args.lr, model_type=args.model)
    print(f"🚀 {msg}\n")

    # Giám sát tiến trình train cho đến khi hoàn tất
    while mgr.is_training:
        p = mgr.train_progress
        sys.stdout.write(f"\r⏳ [{p['percent']:3d}%] Epoch {p['epoch']}/{p['total_epochs']} | Train Loss: {p['train_loss']:.4f} | Val Loss: {p['val_loss']:.4f} | Val Acc: {p['val_acc']:.1f}%   ")
        sys.stdout.flush()
        time.sleep(0.5)

    print("\n")
    p = mgr.train_progress
    if p['error']:
        print(f"❌ Huấn luyện thất bại: {p['error']}")
    else:
        print("🎉 HUẤN LUYỆN HOÀN TẤT THÀNH CÔNG!")
        print(f"💾 File mô hình tối ưu đã được lưu tại: {mgr.model_path}")
        print("⚡ Bạn có thể kích hoạt chế độ tự hành né vật cản ngay trên Web Cockpit!")


if __name__ == "__main__":
    main()
