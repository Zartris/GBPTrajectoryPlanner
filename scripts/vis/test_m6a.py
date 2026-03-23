#!/usr/bin/env python3
"""
scripts/vis/test_m6a.py — Automated visualiser smoke-test.

Starts the visualiser with VIS_SCREENSHOT_DELAY, waits for the screenshot
file to appear, then reports success or failure.  Can be extended to run
multiple scenarios by passing different --scenario arguments.

Usage:
    python3 scripts/vis/test_m6a.py [options]

Options:
    --delay SECS      Seconds to wait for screenshot after launch (default: 5)
    --path PATH       Screenshot output path (default: /tmp/vis-test-screenshot.png)
    --timeout SECS    Maximum total seconds to wait for file (default: 30)
    --scenario FILE   Scenario TOML to pass to simulator (default: none)
    --no-quit         Do NOT set VIS_SCREENSHOT_QUIT (leave app running after screenshot)

Exit codes:
    0  — screenshot file exists and is non-empty (success)
    1  — timeout or file not found (failure)
    2  — usage / configuration error
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Smoke-test the GBP visualiser by checking a timed screenshot."
    )
    p.add_argument("--delay",    type=float, default=5.0,
                   help="Seconds after launch before screenshot is taken (default: 5)")
    p.add_argument("--path",     default="/tmp/vis-test-screenshot.png",
                   help="Screenshot output path (default: /tmp/vis-test-screenshot.png)")
    p.add_argument("--timeout",  type=float, default=30.0,
                   help="Max seconds to wait for the file (default: 30)")
    p.add_argument("--scenario", default=None,
                   help="Path to scenario TOML (passed to visualiser via SCENARIO_PATH env)")
    p.add_argument("--no-quit",  action="store_true",
                   help="Leave the visualiser running after the screenshot")
    return p.parse_args()


def wait_for_file(path: str, timeout: float, poll_interval: float = 0.5) -> bool:
    """Poll for a non-empty file at *path* for up to *timeout* seconds."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        p = Path(path)
        if p.exists() and p.stat().st_size > 0:
            return True
        time.sleep(poll_interval)
    return False


def run_scenario(
    delay: float,
    screenshot_path: str,
    timeout: float,
    scenario: str | None,
    quit_after: bool,
) -> bool:
    """Launch the visualiser for one scenario and check the screenshot appears."""

    # Remove stale screenshot so we don't mistake an old file for success.
    stale = Path(screenshot_path)
    if stale.exists():
        stale.unlink()
        print(f"[test_m6a] removed stale screenshot: {screenshot_path}")

    env = os.environ.copy()
    env["VIS_SCREENSHOT_DELAY"] = str(delay)
    env["VIS_SCREENSHOT_PATH"]  = screenshot_path
    if quit_after:
        env["VIS_SCREENSHOT_QUIT"] = "1"
    if scenario:
        env["SCENARIO_PATH"] = scenario
    # Prefer DISPLAY from env; fall back to :0 for headless CI.
    env.setdefault("DISPLAY", ":0")
    env.pop("WAYLAND_DISPLAY", None)

    # Resolve repo root (two levels up from this script).
    repo_root = Path(__file__).resolve().parent.parent.parent

    cmd = ["cargo", "run", "--release", "-p", "visualiser"]
    print(f"[test_m6a] launching: {' '.join(cmd)}")
    print(f"[test_m6a] screenshot delay={delay}s  path={screenshot_path}")

    proc = subprocess.Popen(cmd, cwd=str(repo_root), env=env)

    try:
        print(f"[test_m6a] waiting up to {timeout}s for screenshot…")
        success = wait_for_file(screenshot_path, timeout)
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    return success


def main() -> int:
    args = parse_args()

    scenarios = [args.scenario] if args.scenario else [None]

    all_passed = True
    for idx, scenario in enumerate(scenarios, start=1):
        label = scenario or "(default)"
        print(f"\n[test_m6a] === scenario {idx}/{len(scenarios)}: {label} ===")

        passed = run_scenario(
            delay=args.delay,
            screenshot_path=args.path,
            timeout=args.timeout,
            scenario=scenario,
            quit_after=not args.no_quit,
        )

        if passed:
            size = Path(args.path).stat().st_size
            print(f"[test_m6a] PASS — screenshot written ({size} bytes): {args.path}")
        else:
            print(f"[test_m6a] FAIL — screenshot not found within {args.timeout}s")
            all_passed = False

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
