#!/usr/bin/env bash
set -e

mapfile -t PORTS < <(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null)

if [ ${#PORTS[@]} -eq 0 ]; then
    echo "No serial ports found. Is the device connected?"
    exit 1
fi

echo "Select a port to monitor:"
select PORT in "${PORTS[@]}"; do
    if [ -n "$PORT" ]; then
        break
    fi
    echo "Invalid selection, try again."
done

echo ""
echo "Monitoring $PORT (Ctrl+R to reset, Ctrl+C to exit)..."
espflash monitor --port "$PORT" --log-format defmt --elf target/riscv32imac-unknown-none-elf/release/esp32c5_rust_template
