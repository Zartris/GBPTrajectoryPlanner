#!/usr/bin/env python3
"""Automated M6a visual test — runs the visualiser and tests each feature.

Usage:
    # Start simulator first:
    #   cargo run --release -p simulator -- --scenario config/scenarios/fleet_4.toml
    # Then run:
    python3 scripts/vis/test_m6a.py
"""
import subprocess
import time
import os
import sys

CMD_FILE = "/tmp/vis-commands.txt"
SCREENSHOT_DIR = "/tmp/vis-tests"

def send(cmd: str):
    with open(CMD_FILE, "a") as f:
        f.write(cmd + "\n")

def screenshot(name: str) -> str:
    path = f"{SCREENSHOT_DIR}/{name}.png"
    send(f"screenshot {path}")
    for _ in range(100):  # 10s max
        time.sleep(0.1)
        if os.path.exists(path) and os.path.getsize(path) > 100:
            print(f"  ✓ {name}: {os.path.getsize(path)} bytes")
            return path
    print(f"  ✗ {name}: screenshot timeout")
    return ""

def main():
    os.makedirs(SCREENSHOT_DIR, exist_ok=True)
    # Clear old command file
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)

    print("Waiting 3s for visualiser to initialize...")
    time.sleep(3)

    print("\n=== M6a Visual Test Suite ===\n")

    # Test 1: Default view
    print("1. Default view (overhead)")
    screenshot("01_default_view")

    # Test 2: F2 metrics
    print("2. Toggle F2 metrics")
    send("key F2")
    time.sleep(0.5)
    screenshot("02_metrics_on")

    # Test 3: F1 inspector
    print("3. Toggle F1 inspector")
    send("key F1")
    time.sleep(0.5)
    screenshot("03_inspector_on")

    # Test 4: Turn off F1/F2
    send("key F1")
    send("key F2")
    time.sleep(0.3)

    # Test 5: Camera orbit
    print("4. Camera orbit (yaw=0.5, pitch=0.5)")
    send("orbit 0.5 0.5")
    time.sleep(0.5)
    screenshot("04_orbit")

    # Test 6: Camera reset
    print("5. Camera reset (R)")
    send("key R")
    time.sleep(0.5)
    screenshot("05_reset")

    # Test 7: Zoom in
    print("6. Zoom in (distance=10)")
    send("zoom 10")
    time.sleep(0.5)
    screenshot("06_zoom_in")

    # Test 8: Reset and test draw toggles
    send("key R")
    time.sleep(0.3)

    print("7. Draw toggles — all off")
    send("draw_all off")
    time.sleep(0.5)
    screenshot("07_draw_all_off")

    print("8. Draw toggles — all on")
    send("draw_all on")
    time.sleep(0.5)
    screenshot("08_draw_all_on")

    # Test 9: Follow camera
    print("9. Follow camera (Tab)")
    send("key Tab")
    time.sleep(1.0)
    screenshot("09_follow")

    # Test 10: Escape back to orbit
    send("key Escape")
    send("key R")
    time.sleep(0.5)
    screenshot("10_back_to_orbit")

    print(f"\n=== Done. Screenshots in {SCREENSHOT_DIR}/ ===")
    print("Review with: ls -la /tmp/vis-tests/")

    # Optionally quit
    # send("quit")

if __name__ == "__main__":
    main()
