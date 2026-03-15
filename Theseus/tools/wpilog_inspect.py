#!/usr/bin/env python3
"""Inspect and grep AdvantageKit .wpilog files.

Usage:
    python3 tools/wpilog_inspect.py <file.wpilog>                  # list all entries
    python3 tools/wpilog_inspect.py <file.wpilog> --keys           # list signal keys only
    python3 tools/wpilog_inspect.py <file.wpilog> --grep <pattern> # grep key names
    python3 tools/wpilog_inspect.py <file.wpilog> --dump <key>     # dump values for a key
"""
import argparse
import re
import sys

from wpiutil.log import DataLogReader


def main():
    parser = argparse.ArgumentParser(description="Inspect .wpilog files")
    parser.add_argument("file", help="Path to .wpilog file")
    parser.add_argument("--keys", action="store_true", help="List all signal keys")
    parser.add_argument("--grep", type=str, help="Filter keys by regex pattern")
    parser.add_argument("--dump", type=str, help="Dump all values for a specific key")
    parser.add_argument("--last", type=int, default=0, help="Show only last N values (with --dump)")
    args = parser.parse_args()

    reader = DataLogReader(args.file)
    entries = {}  # id -> (name, type, metadata)

    if args.keys or args.grep:
        for record in reader:
            if record.isStart():
                data = record.getStartData()
                name = data.name
                if args.grep and not re.search(args.grep, name, re.IGNORECASE):
                    continue
                print(f"{name}  [{data.type}]")
        return

    if args.dump:
        # First pass: find the entry ID for the requested key
        target_id = None
        target_type = None
        for record in reader:
            if record.isStart():
                data = record.getStartData()
                if data.name == args.dump:
                    target_id = data.entry
                    target_type = data.type
                    break

        if target_id is None:
            print(f"Key '{args.dump}' not found", file=sys.stderr)
            sys.exit(1)

        # Second pass: read values
        reader2 = DataLogReader(args.file)
        values = []
        for record in reader2:
            if not record.isStart() and not record.isFinish() and not record.isSetMetadata() and not record.isControl():
                if record.getEntry() == target_id:
                    ts = record.getTimestamp() / 1e6  # microseconds to seconds
                    try:
                        if target_type == "double":
                            val = record.getDouble()
                        elif target_type == "int64":
                            val = record.getInteger()
                        elif target_type == "boolean":
                            val = record.getBoolean()
                        elif target_type == "string":
                            val = record.getString()
                        elif target_type == "double[]":
                            val = list(record.getDoubleArray())
                        elif target_type == "float[]":
                            val = list(record.getFloatArray())
                        elif target_type == "int64[]":
                            val = list(record.getIntegerArray())
                        elif target_type == "string[]":
                            val = list(record.getStringArray())
                        else:
                            val = f"<{target_type}>"
                    except Exception:
                        val = "<decode error>"
                    values.append((ts, val))

        if args.last > 0:
            values = values[-args.last:]

        for ts, val in values:
            print(f"{ts:.3f}s  {val}")
        return

    # Default: summary
    record_count = 0
    key_count = 0
    for record in reader:
        if record.isStart():
            data = record.getStartData()
            key_count += 1
        record_count += 1

    print(f"File: {args.file}")
    print(f"Records: {record_count}")
    print(f"Signals: {key_count}")
    print(f"\nRun with --keys to list signals, --grep <pattern> to filter, --dump <key> to see values")


if __name__ == "__main__":
    main()
