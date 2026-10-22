import sys
sys.path.insert(0, '.')
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')
import os
import shutil
import time

from modules.frame_recorder import FrameRecorder
from modules.scan_matcher import CorrelativeScanMatcher2D
import numpy as np

def test_frame_recorder_session_persistence():
    test_logs_dir = "test_logs_tmp"
    if os.path.exists(test_logs_dir):
        shutil.rmtree(test_logs_dir)
    
    recorder = FrameRecorder(base_dir=test_logs_dir)
    sess_id_1 = recorder.current_session_id
    assert sess_id_1 is not None, "Session ID should be created on init"

    # Command 1 (e.g. Turn 45)
    recorder.start_command("STEP_TURN_45")
    assert recorder.is_capturing == True
    dummy_rgb = np.zeros((180, 320, 3), dtype=np.uint8)
    dummy_depth = np.zeros((180, 320), dtype=np.uint16)
    
    recorder.record_frame(1, dummy_rgb, dummy_depth, {"status": "TEST"}, robot_pose=(0, 0, 0))
    time.sleep(0.12)
    recorder.record_frame(2, dummy_rgb, dummy_depth, {"status": "TEST"}, robot_pose=(0, 0, 0.5))
    recorder.stop_command()
    assert recorder.is_capturing == False
    assert recorder.session_frame_count == 2, f"Expected 2 frames, got {recorder.session_frame_count}"

    # Command 2 (e.g. Linear Forward 20cm) - SHOULD STAY IN SAME SESSION!
    recorder.start_command("STEP_LINEAR_FWD_20CM")
    assert recorder.current_session_id == sess_id_1, "Session ID should be preserved across commands!"
    assert recorder.session_frame_count == 2, "Frame count should continue sequentially!"
    time.sleep(0.12)
    recorder.record_frame(15, dummy_rgb, dummy_depth, {"status": "TEST"}, robot_pose=(0.1, 0, 0.5))
    recorder.stop_command()
    assert recorder.session_frame_count == 3, f"Expected 3 frames total, got {recorder.session_frame_count}"

    # Export ZIP
    zip_path = recorder.create_session_zip()
    assert zip_path and os.path.exists(zip_path), "ZIP export should succeed"
    assert os.path.basename(zip_path).startswith(sess_id_1), "ZIP should be for the full session"

    # Cleanup
    if os.path.exists(test_logs_dir):
        shutil.rmtree(test_logs_dir)
    if zip_path and os.path.exists(zip_path):
        os.remove(zip_path)

    print("✅ [TEST] FrameRecorder session persistence & continuous counting passed!")

def test_scan_matcher_rejection_of_tiny_noise():
    sm = CorrelativeScanMatcher2D(search_dist_m=0.08, min_score=0.28)
    grid = np.zeros((200, 200), dtype=np.uint8)
    # A wall at x=1.0m (col = (1.0 - (-5.0))/0.05 = 120)
    grid[:, 120] = 255
    
    # 20 points on the wall
    scan_pts = [[1.0, float(y)] for y in np.linspace(-0.5, 0.5, 20)]
    
    # Pose exactly at (0, 0, 0)
    rx, ry, ryaw, matched, score = sm.align_pose(0.0, 0.0, 0.0, scan_pts, grid, -5.0, -5.0, 200, 200, 0.05)
    # At (0,0,0) it's already an exact match, so score gain over init_hits is 0.0
    # It should not shift!
    assert abs(rx - 0.0) < 0.01, f"Should not drift away from exact match, got rx={rx}"
    assert abs(ry - 0.0) < 0.01, f"Should not drift away from exact match, got ry={ry}"
    print("✅ [TEST] CorrelativeScanMatcher2D stability & noise rejection passed!")

if __name__ == "__main__":
    test_frame_recorder_session_persistence()
    test_scan_matcher_rejection_of_tiny_noise()
    print("🎉 ALL COMPONENT TESTS PASSED!")
