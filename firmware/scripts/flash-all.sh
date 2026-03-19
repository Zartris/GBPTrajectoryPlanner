#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FIRMWARE_DIR="$SCRIPT_DIR/.."
BINARY="$FIRMWARE_DIR/target/riscv32imac-unknown-none-elf/release/esp32c5_rust_template"

mapfile -t PROBES < <(probe-rs list 2>/dev/null | grep -oP '(?<=-- )[^\s]+(?= )' || true)

if [ ${#PROBES[@]} -eq 0 ]; then
    echo "No probes found. Is the device connected?"
    exit 1
fi

echo "Found ${#PROBES[@]} probe(s):"
for i in "${!PROBES[@]}"; do
    echo "  [$i] ${PROBES[$i]}"
done
echo ""

echo "Building..."
(cd "$FIRMWARE_DIR" && cargo build --release)
echo ""

PIDS=()
for i in "${!PROBES[@]}"; do
    probe-rs run --chip esp32c5 --preverify --no-location --catch-hardfault \
        --probe "${PROBES[$i]}" "$BINARY" \
        2>&1 | sed -u "s/^/[device-$i] /" &
    PIDS+=($!)
done

echo "Flashing ${#PIDS[@]} device(s) in parallel..."

trap 'kill "${PIDS[@]}" 2>/dev/null' INT TERM
wait "${PIDS[@]}"
