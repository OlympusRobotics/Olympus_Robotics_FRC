#!/usr/bin/env bash
# fetch_logs.sh — Download and optionally purge logs from the roboRIO (macOS/Linux)
#
# Usage:
#   ./tools/fetch_logs.sh              # fetch new logs
#   ./tools/fetch_logs.sh --purge      # fetch then delete fetched logs from rio
#   ./tools/fetch_logs.sh --purge-all  # fetch then delete ALL logs from rio

set -euo pipefail

ROBOT_IP="10.49.82.2"
ROBOT_USER="lvuser"
REMOTE_LOG_DIR="/home/lvuser/logs"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOCAL_LOG_DIR="$SCRIPT_DIR/../logs"
SSH_OPTS="-o ConnectTimeout=5 -o StrictHostKeyChecking=no"

PURGE=false
PURGE_ALL=false

for arg in "$@"; do
    case "$arg" in
        --purge)     PURGE=true ;;
        --purge-all) PURGE_ALL=true ;;
        -h|--help)
            echo "Usage: $0 [--purge | --purge-all]"
            echo "  --purge      Delete fetched logs from the roboRIO after download"
            echo "  --purge-all  Delete ALL logs from the roboRIO after download"
            exit 0
            ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done

mkdir -p "$LOCAL_LOG_DIR"

echo "Connecting to roboRIO at $ROBOT_IP..."
if ! ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "echo ok" >/dev/null 2>&1; then
    echo "ERROR: Cannot reach roboRIO at $ROBOT_IP"
    exit 1
fi

# Build list of remote files
echo "Scanning remote logs..."
REMOTE_FILES=$(ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" \
    "find $REMOTE_LOG_DIR -maxdepth 2 -type f \( -name '*.wpilog' -o -name '*.hoot' \) 2>/dev/null" || true)

if [ -z "$REMOTE_FILES" ]; then
    echo "No log files found on roboRIO."
    exit 0
fi

# Download only files we don't already have
FETCHED=()
SKIPPED=0
while IFS= read -r remote_path; do
    filename=$(basename "$remote_path")
    # For files inside dated subdirs, flatten to local logs dir
    if [ -f "$LOCAL_LOG_DIR/$filename" ]; then
        SKIPPED=$((SKIPPED + 1))
        continue
    fi
    echo "  Downloading $filename..."
    scp $SSH_OPTS "$ROBOT_USER@$ROBOT_IP:$remote_path" "$LOCAL_LOG_DIR/" 2>/dev/null
    FETCHED+=("$remote_path")
done <<< "$REMOTE_FILES"

echo ""
echo "Downloaded: ${#FETCHED[@]} file(s), skipped $SKIPPED already-local file(s)"

# Purge if requested
if $PURGE_ALL; then
    echo ""
    echo "Purging ALL logs from roboRIO..."
    ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" \
        "find $REMOTE_LOG_DIR -maxdepth 2 -type f \( -name '*.wpilog' -o -name '*.hoot' -o -name '*.revlog' \) -exec rm -f {} \; && \
         find $REMOTE_LOG_DIR -mindepth 1 -maxdepth 1 -type d -exec rmdir {} \; 2>/dev/null"
    echo "Done — roboRIO logs purged."
elif $PURGE && [ ${#FETCHED[@]} -gt 0 ]; then
    echo ""
    echo "Purging ${#FETCHED[@]} fetched log(s) from roboRIO..."
    for remote_path in "${FETCHED[@]}"; do
        ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "rm -f '$remote_path'" 2>/dev/null
    done
    # Clean up empty directories
    ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" \
        "find $REMOTE_LOG_DIR -mindepth 1 -maxdepth 1 -type d -exec rmdir {} \; 2>/dev/null"
    echo "Done."
fi

echo ""
echo "Local logs: $LOCAL_LOG_DIR"
ls -lh "$LOCAL_LOG_DIR"/*.wpilog "$LOCAL_LOG_DIR"/*.hoot 2>/dev/null | tail -10
