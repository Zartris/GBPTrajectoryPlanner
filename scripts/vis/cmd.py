#!/usr/bin/env python3
"""Send commands to the running visualiser via command file.

Usage:
    python3 scripts/vis/cmd.py screenshot              # Take screenshot
    python3 scripts/vis/cmd.py screenshot /path/to.png # Screenshot to specific path
    python3 scripts/vis/cmd.py key F1                  # Simulate F1 press
    python3 scripts/vis/cmd.py key F2                  # Simulate F2 press
    python3 scripts/vis/cmd.py key R                   # Reset camera
    python3 scripts/vis/cmd.py key C                   # Toggle orbit/pan
    python3 scripts/vis/cmd.py key Tab                 # Cycle follow camera
    python3 scripts/vis/cmd.py key Escape              # Exit follow mode
    python3 scripts/vis/cmd.py orbit <yaw> <pitch>     # Set orbit angles (radians)
    python3 scripts/vis/cmd.py zoom <distance>         # Set zoom distance
    python3 scripts/vis/cmd.py draw <field> <on|off>   # Toggle draw config field
    python3 scripts/vis/cmd.py draw_all on             # Enable all draw toggles
    python3 scripts/vis/cmd.py draw_all off            # Disable all draw toggles
    python3 scripts/vis/cmd.py wait <seconds>          # Sleep (for scripting)
    python3 scripts/vis/cmd.py quit                    # Close the visualiser
"""
import sys
import time
import os

CMD_FILE = os.environ.get("VIS_CMD_FILE", "/tmp/vis-commands.txt")

def send(cmd: str):
    with open(CMD_FILE, "a") as f:
        f.write(cmd + "\n")

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    action = sys.argv[1]

    if action == "screenshot":
        path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/vis-screenshot.png"
        send(f"screenshot {path}")
        # Wait for the screenshot to be written
        for _ in range(50):  # 5 seconds max
            time.sleep(0.1)
            if os.path.exists(path):
                sz = os.path.getsize(path)
                if sz > 0:
                    print(f"Screenshot saved: {path} ({sz} bytes)")
                    return
        print(f"Warning: screenshot may not have been saved to {path}")

    elif action == "key":
        if len(sys.argv) < 3:
            print("Usage: cmd.py key <KeyCode>")
            sys.exit(1)
        send(f"key {sys.argv[2]}")

    elif action == "orbit":
        if len(sys.argv) < 4:
            print("Usage: cmd.py orbit <yaw> <pitch>")
            sys.exit(1)
        send(f"orbit {sys.argv[2]} {sys.argv[3]}")

    elif action == "zoom":
        if len(sys.argv) < 3:
            print("Usage: cmd.py zoom <distance>")
            sys.exit(1)
        send(f"zoom {sys.argv[2]}")

    elif action == "draw":
        if len(sys.argv) < 4:
            print("Usage: cmd.py draw <field> <on|off>")
            sys.exit(1)
        send(f"draw {sys.argv[2]} {sys.argv[3]}")

    elif action == "draw_all":
        state = sys.argv[2] if len(sys.argv) > 2 else "on"
        send(f"draw_all {state}")

    elif action == "wait":
        secs = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
        time.sleep(secs)

    elif action == "quit":
        send("quit")

    else:
        print(f"Unknown action: {action}")
        print(__doc__)
        sys.exit(1)

if __name__ == "__main__":
    main()
