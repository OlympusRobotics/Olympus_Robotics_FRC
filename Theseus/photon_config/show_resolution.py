#!/usr/bin/env python3
"""Extract camera resolution info from PhotonVision config dumps."""
import json
import re
import sys
from pathlib import Path

config_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent

for label in ["13", "14"]:
    dump = config_dir / f"coprocessor-{label}_dump.json"
    if not dump.exists():
        print(f"No dump found: {dump}")
        continue

    with open(dump) as f:
        data = json.load(f)

    cameras = data.get("cameras", {})
    if isinstance(cameras, list):
        cam_items = [(f"cam{i}", c) for i, c in enumerate(cameras)]
    else:
        cam_items = list(cameras.items())

    for cam_key, cam in cam_items:
        # Config is already a dict (from our SQLite dump)
        cfg = cam.get("config", {})
        nickname = cfg.get("nickname", cam_key)
        hw_name = cfg.get("matchedCameraInfo", {}).get("PVUsbCameraInfo", {}).get("name", "?")
        quirks = cfg.get("cameraQuirks", {}).get("quirks", {})
        ov9281 = quirks.get("ArduOV9281Controls", False)

        print(f"=== Coprocessor {label} / {nickname} ===")
        print(f"  Hardware: {hw_name}")
        if ov9281:
            print(f"  ArduCam OV9281 controls: YES")

        # Calibrations show what resolutions exist
        cals = cfg.get("calibrations", [])
        if cals:
            for cal in cals:
                res = cal.get("resolution", {})
                w = int(res.get("width", 0))
                h = int(res.get("height", 0))
                print(f"  Calibrated resolution: {w}x{h}")

        # Driver mode
        dm = cam.get("drivermode", {})
        if isinstance(dm, dict):
            dm_idx = dm.get("cameraVideoModeIndex", "N/A")
            print(f"  Driver mode video index: {dm_idx}")

        # Pipelines (stored as JSON strings in SQLite dump)
        for p in cam.get("pipelines", []):
            if isinstance(p, str):
                try:
                    parsed = json.loads(p)
                    if isinstance(parsed, list) and len(parsed) >= 2:
                        ps = parsed[1]
                        pname = ps.get("pipelineNickname", "?")
                        ptype = parsed[0]
                        vidx = ps.get("cameraVideoModeIndex", "?")
                        print(f"  Pipeline '{pname}' ({ptype}): video mode index = {vidx}")
                except (json.JSONDecodeError, TypeError, IndexError):
                    m = re.search(r'"cameraVideoModeIndex"\s*:\s*(\d+)', p)
                    if m:
                        print(f"  Pipeline video mode index: {m.group(1)}")

        print()
