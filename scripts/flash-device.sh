#!/usr/bin/env bash
set -e

BINARY="target/riscv32imac-unknown-none-elf/release/esp32c5_rust_template"

mapfile -t PROBES < <(probe-rs list 2>/dev/null | grep -oP '(?<=-- )[^\s]+(?= )' || true)

if [ ${#PROBES[@]} -eq 0 ]; then
    echo "No probes found. Is the device connected?"
    exit 1
fi

if [ -n "$1" ]; then
    idx=$(( $1 - 1 ))
    if [ "$idx" -lt 0 ] || [ "$idx" -ge "${#PROBES[@]}" ]; then
        echo "Invalid selection '$1'. Available probes:"
        for i in "${!PROBES[@]}"; do echo "  $(( i + 1 )) ${PROBES[$i]}"; done
        exit 1
    fi
    PROBE="${PROBES[$idx]}"
else
    echo "Select a probe to flash:"
    select PROBE in "${PROBES[@]}"; do
        if [ -n "$PROBE" ]; then
            break
        fi
        echo "Invalid selection, try again."
    done
fi

echo ""
echo "Building..."
cargo build --release
echo ""

echo "Flashing $PROBE..."
probe-rs run --chip esp32c5 --preverify --always-print-stacktrace --no-location --catch-hardfault --probe "$PROBE" "$BINARY"
