---
name: flash-firmware
description: Build and flash ESP32-C5 firmware to connected probes via probe-rs
disable-model-invocation: true
---

Run the following to flash all connected probes in parallel:

```bash
cd /repo && bash scripts/flash-all.sh
```

If the user specifies a device index (e.g. "flash device 0"), use:
```bash
cd /repo && bash scripts/flash-device.sh <index>
```

Check that probe-rs can see devices first: `probe-rs list`
