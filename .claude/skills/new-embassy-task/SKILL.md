---
name: new-embassy-task
description: Pattern and checklist for adding a new embassy async task to ESP32-C5 firmware
user-invocable: false
---

When adding a new embassy async task to /repo/firmware/src/bins/esp32c5/src/bin/main.rs:

1. Define the task function with `#[embassy_executor::task]`
2. Parameters that must live for 'static need `&'static mut T` — use the `mk_static!` macro
3. Spawn in `main()` with `spawner.spawn(task_name(args)).unwrap()`
4. Keep stack allocations small — the firmware denies `clippy::large_stack_frames`
5. Never use `mem::forget` on esp_hal types (denied in firmware)
6. Use `defmt::info!/warn!/error!` for logging, NOT `println!`
