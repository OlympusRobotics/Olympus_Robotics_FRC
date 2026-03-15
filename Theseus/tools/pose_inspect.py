#!/usr/bin/env python3
"""Decode and print Pose2d values around a timestamp from a wpilog."""
import struct, math, sys
from wpiutil.log import DataLogReader

logfile = sys.argv[1]
start_ts = float(sys.argv[2]) if len(sys.argv) > 2 else 0
end_ts = float(sys.argv[3]) if len(sys.argv) > 3 else start_ts + 5

reader = DataLogReader(logfile)
entries = {}
for rec in reader:
    if rec.isStart():
        d = rec.getStartData()
        entries[d.entry] = (d.name, d.type)

def decode_pose2d(raw):
    if len(raw) == 24:
        x, y, rot = struct.unpack('<ddd', raw)
        return f'({x:.2f}, {y:.2f}, {math.degrees(rot):.1f} deg)'
    return f'<{len(raw)} bytes>'

poses = {'/RealOutputs/Turret/DesiredPose', '/RealOutputs/Turret/ActualPose', '/RealOutputs/Drivetrain/Pose'}
reader2 = DataLogReader(logfile)
for rec in reader2:
    if not rec.isStart() and not rec.isFinish() and not rec.isSetMetadata() and not rec.isControl():
        eid = rec.getEntry()
        ts = rec.getTimestamp() / 1e6
        if eid in entries and entries[eid][0] in poses and start_ts <= ts <= end_ts:
            name = entries[eid][0]
            if 'Desired' in name: label = 'DesiredPose'
            elif 'Actual' in name: label = 'ActualPose'
            else: label = 'DrivePose'
            print(f'{ts:.3f}s  {label:15s}  {decode_pose2d(rec.getRaw())}')
