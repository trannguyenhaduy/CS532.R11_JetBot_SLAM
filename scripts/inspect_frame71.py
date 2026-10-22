import zipfile
import json
import math

with zipfile.ZipFile("session_20261021_161609.zip") as z:
    t_name = [x for x in z.namelist() if x.endswith("telemetry.jsonl")][0]
    with z.open(t_name) as f:
        lines = f.readlines()
        print("Total frames in session:", len(lines))
        for i in range(0, min(80, len(lines)), 3):
            d = json.loads(lines[i].decode("utf-8"))
            telem = d.get("telemetry", {})
            pose = d.get("robot_pose", [0, 0, 0])
            csm = telem.get("csm", {})
            r = telem.get("rays", {})
            dists = [x for x in r.get("distances", []) if x and x > 0.15]
            min_d = min(dists) if dists else -1.0
            print(f"Frame #{d.get('record_index'):03d} (i={i:02d}): cmd={d.get('command')} | "
                  f"yaw={math.degrees(pose[2]):+6.1f} deg | map_up={telem.get('map_updated')} | "
                  f"is_turn={telem.get('is_turning')} | min_d={min_d:.2f}m | obs={r.get('obstacle')}")
