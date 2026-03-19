# PSA: DataLogWriter + AdvantageKit Dual Logging Will Lock Up Your roboRIO

**Team 4982 — Olympus Robotics**

## TL;DR

If you're using AdvantageKit (`LoggedRobot` + `WPILOGWriter`) and also open a second `DataLogWriter` via `NetworkTableInstance.startEntryDataLog()`, the dual write load on the roboRIO's slow eMMC flash will cause the robot to freeze within seconds of enabling. The JVM stalls waiting on synchronous disk I/O, which blocks the main robot thread, which kills NetworkTables, which drops your Driver Station connection.

## The Symptom

- Robot boots fine, connects to Driver Station, dashboards look normal
- The moment you **enable**, NetworkTables freezes within 5–15 seconds
- Elastic and AdvantageScope stop updating
- Driver Station shows "Lost communication with robot"
- SSH still works — the roboRIO kernel is alive, but the JVM is effectively hung
- `top` shows the Java process using 50%+ CPU but the loop is stuck

## What We Did Wrong

We wanted per-enable `.wpilog` files so each match/test session would be a separate log, independent of AdvantageKit's boot-time log. We added this to `Robot.java`:

```java
// In disabledExit() — called right before the robot enables
private void startEnableLog() {
    String path = "/home/lvuser/logs/enabled_" + timestamp + ".wpilog";
    enableLog = new DataLogWriter(path);
    enableNtLogger = NetworkTableInstance.getDefault()
        .startEntryDataLog(enableLog, "", "");  // ← logs ALL NT keys
}

// In disabledInit() — called when robot is disabled
private void stopEnableLog() {
    NetworkTableInstance.getDefault().stopEntryDataLog(enableNtLogger);
    enableLog.close();
}
```

This opens a **second** `.wpilog` writer alongside AdvantageKit's `WPILOGWriter`, both capturing every NT update to separate files on the same eMMC.

## Why It Kills the Robot

The roboRIO 2.0 uses eMMC flash (`mmcblk0`) with typical write latencies of **40–50ms per operation**. Under normal AdvantageKit logging, this is fine — writes are small and periodic.

But with two log writers:
1. **Double the disk write volume** — every NT value change gets written twice
2. **eMMC write contention** — flash has a single write queue; two writers serialize against each other
3. **Dirty page buildup** — the kernel's page cache fills with pending writes faster than the eMMC can flush
4. **JVM thread stall** — `DataLogWriter` eventually blocks on a synchronous `write()` call when page cache pressure is high
5. **Main thread blocked** — since FRC robot code runs on a single main thread with `UseSerialGC`, a blocked write stalls everything: command scheduler, NetworkTables publishing, Driver Station heartbeat
6. **DS timeout** — after ~1.5 seconds of no heartbeat, the DS declares "Lost communication"

## How We Diagnosed It

### git bisect
We had 4 commits between "known good" and "broken". `git bisect` narrowed it to the exact commit in 2 tests.

### On-roboRIO diagnostics
Since the roboRIO doesn't have `iostat`, `iotop`, or `strace`, we used:

**File descriptors** — the smoking gun:
```bash
# Shows two .wpilog files open simultaneously
for fd in /proc/$PID/fd/*; do
  link=$(readlink $fd 2>/dev/null)
  echo "$link" | grep -q wpilog && echo "$fd -> $link"
done
```

**Disk I/O delta** (sample `/proc/diskstats` twice, 2 seconds apart):
```bash
# mmcblk0p3 is the data partition
grep mmcblk0p3 /proc/diskstats
sleep 2
grep mmcblk0p3 /proc/diskstats
# Compare write counts (field 8) and write time ms (field 11)
```

**Memory pressure**:
```bash
grep -E "Dirty|Writeback" /proc/meminfo
# Healthy: Dirty < 500 KB, Writeback < 100 KB
# Sick:    Dirty in MB range, Writeback stuck high
```

**Healthy baseline** (single AdvantageKit log):
| Metric | Value |
|--------|-------|
| Open `.wpilog` FDs | 1 (AdvantageKit only) |
| Dirty pages | ~200 KB |
| eMMC writes/2s | ~3 ops, ~400 KB |
| FD count | 27 |

### Socket leak (secondary issue)

While investigating, we also found **55 TCP sockets stuck in CLOSE_WAIT** from PhotonVision coprocessors that kept disconnecting and reconnecting. The default `tcp_keepalive_time` on the roboRIO is **7200 seconds** (2 hours!) — dead connections pile up and never get cleaned. We fixed this:

```bash
# /etc/sysctl.conf on the roboRIO
net.ipv4.tcp_keepalive_time = 10    # check after 10s idle (was 7200)
net.ipv4.tcp_keepalive_intvl = 5    # probe every 5s (was 75)
net.ipv4.tcp_keepalive_probes = 3   # give up after 3 failures (was 9)
```

## The Fix

**Reverted the dual logging.** AdvantageKit's `WPILOGWriter` is sufficient — it already captures everything. If you want per-session log files, use AdvantageKit's built-in log splitting or process the single `.wpilog` offline with timestamps.

## Other Performance Wins (Same Session)

While debugging, we also fixed some loop overrun issues:

1. **Skip disconnected PhotonVision cameras** — added `cam.isConnected()` check before `getLatestResult()`. Disconnected cameras trigger expensive NT lookups + error stack traces every 20ms. `CameraUsing.periodic()` went from **17–31ms → 1.7ms**.

2. **Cache per-cycle computations in TurretAiming** — `targetpose()` was called 3× per cycle (in `targetAim()`, `vectorCalculations()`, and `maxFormula()`). CAN bus reads (`getPosition()`, `getVelocity()`) were called 3–5× per cycle instead of once. Cached everything. `TurretAiming.periodic()` went from **1–14ms → 0.5ms**.

3. **Merged duplicate Trigger** — two separate `new Trigger()` objects evaluating the same lambda for left trigger axis.

## Lessons Learned

1. **Never open a second `DataLogWriter` alongside AdvantageKit** on a roboRIO. The eMMC can't handle it.
2. **`/proc/diskstats` and `/proc/meminfo` are your friends** on the roboRIO since `iostat`/`iotop`/`strace` aren't available.
3. **Check `/proc/$PID/fd`** — seeing two `.wpilog` FDs would have immediately pointed to the problem.
4. **Set aggressive TCP keepalive** on the roboRIO if you have coprocessors — the 2-hour default is way too long for a robot that runs in 2.5-minute matches.
5. **`git bisect` is incredibly powerful** — found the exact bad commit out of 4 in 2 deploy-and-test cycles.
6. **PhotonVision `isConnected()` check is cheap** — always gate your camera processing with it.

Hope this saves someone else a late night at the shop.
