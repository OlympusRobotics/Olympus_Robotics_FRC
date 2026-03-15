#!/usr/bin/env python3
"""fetch_photon_config.py — Download PhotonVision config and dump pretty JSON.

Usage:
    python3 photon_config/fetch_photon_config.py                # fetch from both coprocessors
    python3 photon_config/fetch_photon_config.py --host 13      # fetch from .13 only
    python3 photon_config/fetch_photon_config.py --host 14      # fetch from .14 only
    python3 photon_config/fetch_photon_config.py --dump-only     # skip fetch, just re-dump existing SQLite
"""

import argparse
import json
import os
import sqlite3
import subprocess
import sys
from pathlib import Path

ROBOT_SUBNET = "10.49.82"
REMOTE_USER = "photon"
REMOTE_PASS = "vision"
REMOTE_CONFIG_DIR = "/opt/photonvision/photonvision_config"

SCRIPT_DIR = Path(__file__).resolve().parent
LOCAL_BASE_DIR = SCRIPT_DIR

SSH_OPTS = ["-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no"]


def sshpass_cmd(base_cmd: list[str]) -> list[str]:
    return ["sshpass", "-p", REMOTE_PASS] + base_cmd


def host_reachable(host: str) -> bool:
    cmd = sshpass_cmd(["ssh"] + SSH_OPTS + [f"{REMOTE_USER}@{host}", "echo ok"])
    result = subprocess.run(cmd, capture_output=True, timeout=10)
    return result.returncode == 0


def fetch_config(host: str, local_dir: Path) -> bool:
    label = host.rsplit(".", 1)[-1]
    print(f"=== Fetching PhotonVision config from {host} ===")

    try:
        if not host_reachable(host):
            print(f"  WARNING: Cannot reach {host} — skipping\n")
            return False
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"  WARNING: {e} — skipping\n")
        return False

    local_dir.mkdir(parents=True, exist_ok=True)
    print(f"  Downloading {REMOTE_CONFIG_DIR}/ -> {local_dir}/")

    cmd = sshpass_cmd(["scp", "-r"] + SSH_OPTS + [
        f"{REMOTE_USER}@{host}:{REMOTE_CONFIG_DIR}/",
        str(local_dir) + "/",
    ])
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"  ERROR: scp failed with code {result.returncode}")
        return False

    print("  Done.\n")
    return True


def dump_pretty_json(local_dir: Path, label: str) -> None:
    db_path = local_dir / "photonvision_config" / "photon.sqlite"
    out_path = LOCAL_BASE_DIR / f"coprocessor-{label}_dump.json"

    if not db_path.exists():
        print(f"  No SQLite database at {db_path} — skipping dump")
        return

    if db_path.stat().st_size == 0:
        print(f"  SQLite database is empty (0 bytes) — skipping dump")
        return

    conn = sqlite3.connect(str(db_path))
    dump = {}

    # Check which tables exist
    tables = {row[0] for row in conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table'"
    )}

    if "global" in tables:
        dump["global"] = {}
        for filename, contents in conn.execute("SELECT filename, contents FROM global"):
            try:
                dump["global"][filename] = json.loads(contents)
            except json.JSONDecodeError:
                dump["global"][filename] = contents

    if "cameras" in tables:
        dump["cameras"] = {}
        for row in conn.execute(
            "SELECT unique_name, config_json, drivermode_json, "
            "pipeline_jsons, otherpaths_json FROM cameras"
        ):
            uid = row[0]
            config = json.loads(row[1])
            nickname = config.get("nickname", uid)
            dump["cameras"][nickname] = {
                "unique_name": uid,
                "config": config,
                "drivermode": json.loads(row[2]),
                "pipelines": json.loads(row[3]),
                "otherpaths": json.loads(row[4]),
            }

    conn.close()

    if not dump:
        print(f"  No data found in {db_path}")
        return

    with open(out_path, "w") as f:
        json.dump(dump, f, indent=2)

    size_kb = out_path.stat().st_size / 1024
    cameras = list(dump.get("cameras", {}).keys())
    global_keys = list(dump.get("global", {}).keys())
    print(f"  Dumped {db_path.name} -> {out_path.name} ({size_kb:.0f} KB)")
    print(f"    Cameras: {cameras or '(none)'}")
    print(f"    Global:  {global_keys or '(none)'}")


def main():
    parser = argparse.ArgumentParser(description="Fetch PhotonVision config from coprocessors")
    parser.add_argument("--host", type=int, help="Last octet of coprocessor IP (e.g. 13 or 14)")
    parser.add_argument("--dump-only", action="store_true",
                        help="Skip SCP fetch, just re-dump existing SQLite databases")
    args = parser.parse_args()

    # Check sshpass availability (only needed for fetch)
    if not args.dump_only:
        if subprocess.run(["which", "sshpass"], capture_output=True).returncode != 0:
            print("ERROR: sshpass is required. Install with: brew install sshpass")
            sys.exit(1)

    hosts = [f"{ROBOT_SUBNET}.{args.host}"] if args.host else [
        f"{ROBOT_SUBNET}.13",
        f"{ROBOT_SUBNET}.14",
    ]

    for host in hosts:
        label = host.rsplit(".", 1)[-1]
        local_dir = LOCAL_BASE_DIR / f"coprocessor-{label}"

        if not args.dump_only:
            fetch_config(host, local_dir)

        if local_dir.exists():
            dump_pretty_json(local_dir, label)
            print()

    print(f"Local config: {LOCAL_BASE_DIR}")


if __name__ == "__main__":
    main()
