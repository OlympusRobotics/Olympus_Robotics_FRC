#!/usr/bin/env python3
"""check_health.py — Diagnose PhotonVision coprocessor health.

Checks reachability, HTTP status, CPU/memory/temperature, network errors,
and PhotonVision service status for each coprocessor.

Usage:
    python3 photon_config/check_health.py              # check both coprocessors
    python3 photon_config/check_health.py --host 13    # check .13 only
    python3 photon_config/check_health.py --host 14    # check .14 only

Requires: sshpass (brew install sshpass)
"""

import argparse
import subprocess
import sys
import urllib.request
import urllib.error

ROBOT_SUBNET = "10.49.82"
REMOTE_USER = "photon"
REMOTE_PASS = "vision"
DEFAULT_HOSTS = [13, 14]

SSH_OPTS = ["-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no",
            "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR"]


def sshpass_cmd(base_cmd: list[str]) -> list[str]:
    return ["sshpass", "-p", REMOTE_PASS] + base_cmd


def banner(msg: str) -> None:
    print(f"\n{'='*60}")
    print(f"  {msg}")
    print(f"{'='*60}")


def section(msg: str) -> None:
    print(f"\n--- {msg} ---")


def run_ssh(host: str, cmd: str, timeout: int = 10) -> tuple[bool, str]:
    """Run a command over SSH. Returns (success, stdout)."""
    full_cmd = sshpass_cmd(
        ["ssh"] + SSH_OPTS + [f"{REMOTE_USER}@{host}", cmd]
    )
    try:
        result = subprocess.run(full_cmd, capture_output=True, text=True,
                                timeout=timeout)
        return result.returncode == 0, result.stdout.strip()
    except subprocess.TimeoutExpired:
        return False, "(timed out)"
    except FileNotFoundError:
        return False, "(sshpass not found — brew install sshpass)"


def check_ping(host: str) -> None:
    """Ping the host and report packet loss."""
    section(f"Ping ({host})")
    try:
        result = subprocess.run(
            ["ping", "-c", "10", "-W", "2", host],
            capture_output=True, text=True, timeout=30
        )
        # Extract the summary lines
        for line in result.stdout.splitlines():
            if "packet loss" in line or "round-trip" in line or "rtt" in line:
                print(f"  {line.strip()}")
        if result.returncode != 0:
            print("  WARN: Some or all pings failed")
        else:
            print("  OK")
    except subprocess.TimeoutExpired:
        print("  FAIL: ping timed out")


def check_http(host: str) -> None:
    """Check if the PhotonVision web UI is responding."""
    section(f"PhotonVision HTTP ({host}:5800)")
    url = f"http://{host}:5800"
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=5) as resp:
            print(f"  HTTP {resp.status} — PhotonVision UI is up")
    except urllib.error.HTTPError as e:
        print(f"  HTTP {e.code} — server responded but returned error")
    except urllib.error.URLError as e:
        print(f"  FAIL: {e.reason}")
    except Exception as e:
        print(f"  FAIL: {e}")


def check_cpu_mem(host: str) -> None:
    """Check CPU and memory usage."""
    section(f"CPU & Memory ({host})")

    # CPU load averages
    ok, out = run_ssh(host, "cat /proc/loadavg")
    if ok:
        parts = out.split()
        print(f"  Load average: {parts[0]} / {parts[1]} / {parts[2]}  "
              f"(1 / 5 / 15 min)")
        load1 = float(parts[0])
        if load1 > 3.5:
            print("  WARN: Very high CPU load — Pi is likely overloaded")
        elif load1 > 2.0:
            print("  WARN: Elevated CPU load")
    else:
        print(f"  Could not read load: {out}")

    # Memory
    ok, out = run_ssh(host, "free -m | grep Mem")
    if ok:
        parts = out.split()
        total = int(parts[1])
        used = int(parts[2])
        avail = int(parts[-1])
        pct = used * 100 // total
        print(f"  Memory: {used}M / {total}M used ({pct}%), {avail}M available")
        if avail < 100:
            print("  WARN: Very low available memory")
    else:
        print(f"  Could not read memory: {out}")


def check_temperature(host: str) -> None:
    """Check CPU temperature."""
    section(f"Temperature ({host})")

    # Try vcgencmd first (Raspberry Pi), fall back to thermal_zone
    ok, out = run_ssh(host, "vcgencmd measure_temp 2>/dev/null || "
                            "cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null")
    if ok and out:
        if "temp=" in out:
            # vcgencmd format: temp=52.0'C
            temp_str = out.split("=")[1].replace("'C", "").strip()
            temp = float(temp_str)
            print(f"  CPU temp: {temp}°C")
        else:
            # thermal_zone format: millidegrees
            try:
                temp = int(out) / 1000.0
                print(f"  CPU temp: {temp:.1f}°C")
            except ValueError:
                print(f"  Raw output: {out}")
                return

        if temp >= 80:
            print("  CRITICAL: Thermal throttling likely active!")
        elif temp >= 70:
            print("  WARN: Running hot — consider adding cooling")
        else:
            print("  OK")
    else:
        print(f"  Could not read temperature: {out}")


def check_network(host: str) -> None:
    """Check network interface stats for errors/drops."""
    section(f"Network interface stats ({host})")

    ok, out = run_ssh(host, "cat /proc/net/dev | grep -E 'eth0|end0'")
    if ok and out:
        # /proc/net/dev columns:
        # face |bytes packets errs drop fifo frame compressed multicast|
        #      |bytes packets errs drop fifo frame compressed multicast|
        #  (receive side)                                (transmit side)
        line = out.strip().split(":")[1].split()
        rx_bytes, rx_pkts, rx_errs, rx_drop = (
            int(line[0]), int(line[1]), int(line[2]), int(line[3]))
        tx_bytes, tx_pkts, tx_errs, tx_drop = (
            int(line[8]), int(line[9]), int(line[10]), int(line[11]))
        print(f"  RX: {rx_pkts} pkts, {rx_bytes // 1048576}MB, "
              f"{rx_errs} errors, {rx_drop} drops")
        print(f"  TX: {tx_pkts} pkts, {tx_bytes // 1048576}MB, "
              f"{tx_errs} errors, {tx_drop} drops")
        if rx_errs > 0 or tx_errs > 0:
            print("  WARN: Interface errors detected — check cable/connector")
        if rx_drop > 100 or tx_drop > 100:
            print("  WARN: High packet drops at interface level")
    else:
        print(f"  Could not read interface stats: {out}")


def check_service(host: str) -> None:
    """Check if the PhotonVision systemd service is running."""
    section(f"PhotonVision service ({host})")

    ok, out = run_ssh(host,
        "systemctl is-active photonvision 2>/dev/null && "
        "systemctl show photonvision --property=ActiveEnterTimestamp "
        "--no-pager 2>/dev/null")
    if ok:
        lines = out.splitlines()
        status = lines[0] if lines else "unknown"
        uptime_line = lines[1] if len(lines) > 1 else ""
        print(f"  Service status: {status}")
        if uptime_line:
            print(f"  {uptime_line}")
    else:
        print(f"  Could not query service: {out}")


def check_voltage(host: str) -> None:
    """Check for under-voltage warnings (Raspberry Pi)."""
    section(f"Power / under-voltage ({host})")

    ok, out = run_ssh(host,
        "vcgencmd get_throttled 2>/dev/null || echo 'N/A'")
    if ok and out != "N/A":
        # throttled=0x0 means no issues
        print(f"  {out}")
        if "0x0" in out:
            print("  OK — no throttling or under-voltage detected")
        else:
            # Decode common flags
            try:
                val = int(out.split("=")[1], 16)
                if val & 0x1:
                    print("  CRITICAL: Under-voltage detected RIGHT NOW")
                if val & 0x2:
                    print("  WARN: ARM frequency capped RIGHT NOW")
                if val & 0x4:
                    print("  WARN: Currently throttled")
                if val & 0x10000:
                    print("  INFO: Under-voltage has occurred since boot")
                if val & 0x20000:
                    print("  INFO: ARM frequency capping has occurred since boot")
                if val & 0x40000:
                    print("  INFO: Throttling has occurred since boot")
            except (ValueError, IndexError):
                pass
    else:
        print("  Could not read throttle status (may not be a Raspberry Pi)")

    # Also check dmesg for voltage warnings
    ok, out = run_ssh(host, "dmesg 2>/dev/null | grep -i -c voltage || echo 0")
    if ok:
        count = out.strip()
        if count != "0":
            print(f"  WARN: {count} voltage-related messages in dmesg")
        else:
            print("  No voltage warnings in dmesg")


def check_disk(host: str) -> None:
    """Check disk usage."""
    section(f"Disk usage ({host})")

    ok, out = run_ssh(host, "df -h / | tail -1")
    if ok:
        parts = out.split()
        print(f"  Root filesystem: {parts[2]} used / {parts[1]} total ({parts[4]})")
        pct = int(parts[4].replace("%", ""))
        if pct > 90:
            print("  WARN: Disk nearly full — logs filling up?")
    else:
        print(f"  Could not read disk: {out}")


def check_host(host_num: int) -> None:
    host = f"{ROBOT_SUBNET}.{host_num}"
    banner(f"Coprocessor {host_num}  ({host})")

    check_ping(host)
    check_http(host)

    # SSH-based checks — test SSH first
    section(f"SSH connectivity ({host})")
    ok, out = run_ssh(host, "echo ok")
    if not ok:
        print(f"  FAIL: Cannot SSH into {host} — skipping remote checks")
        print(f"  ({out})")
        return
    print(f"  OK — SSH working")

    check_cpu_mem(host)
    check_temperature(host)
    check_voltage(host)
    check_network(host)
    check_service(host)
    check_disk(host)


def main():
    parser = argparse.ArgumentParser(
        description="Check health of PhotonVision coprocessors")
    parser.add_argument("--host", type=int, choices=[13, 14],
                        help="Check only this coprocessor (13 or 14)")
    args = parser.parse_args()

    hosts = [args.host] if args.host else DEFAULT_HOSTS

    print("PhotonVision Coprocessor Health Check")
    print(f"Checking: {', '.join(f'{ROBOT_SUBNET}.{h}' for h in hosts)}")

    for h in hosts:
        check_host(h)

    print(f"\n{'='*60}")
    print("  Done.")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
