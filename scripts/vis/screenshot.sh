#!/bin/bash
# scripts/vis/screenshot.sh — Quick screenshot helper for the GBP visualiser.
#
# Usage:
#   scripts/vis/screenshot.sh [delay_secs] [output_path]
#
# Defaults:
#   delay_secs  = 2
#   output_path = /tmp/vis-screenshot.png
#
# The visualiser will launch, wait <delay_secs> seconds, capture a screenshot,
# and then exit.
#
# Example:
#   scripts/vis/screenshot.sh 4 /tmp/my-run.png

set -euo pipefail

DELAY="${1:-2}"
OUTPUT="${2:-/tmp/vis-screenshot.png}"

# Validate that DELAY is a positive number (integer or decimal).
if ! [[ "${DELAY}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    echo "[vis/screenshot.sh] ERROR: delay must be a non-negative number, got '${DELAY}'" >&2
    exit 2
fi

echo "[vis/screenshot.sh] delay=${DELAY}s  output=${OUTPUT}"

# Ensure we run from the repo root so cargo can find the workspace.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

VIS_SCREENSHOT_DELAY="${DELAY}" \
VIS_SCREENSHOT_PATH="${OUTPUT}" \
VIS_SCREENSHOT_QUIT="1" \
WAYLAND_DISPLAY="" \
DISPLAY="${DISPLAY:-:0}" \
    cargo run --release -p visualiser

if [[ -f "${OUTPUT}" ]]; then
    echo "[vis/screenshot.sh] screenshot saved to ${OUTPUT}"
else
    echo "[vis/screenshot.sh] WARNING: visualiser exited but '${OUTPUT}' was not found" >&2
    exit 1
fi
