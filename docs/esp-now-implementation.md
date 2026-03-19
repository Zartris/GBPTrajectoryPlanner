# ESP-NOW Implementation Notes

## The Problem: Task Starvation

The original code used a single loop with `select`:

```rust
loop {
    let res = select(ticker.next(), esp_now.receive_async()).await;
    match res {
        Either::First(_) => { /* SEND - blocks here until send completes */ }
        Either::Second(_) => { /* receive */ }
    }
}
```

`select` waits for **whichever future finishes first**, then runs that branch. The issue:

1. Ticker fires → we enter the **send** branch
2. `send_async` takes time (waiting on the radio hardware)
3. While stuck sending, incoming packets arrive — but nobody is watching
4. When send finishes, we loop back and the ticker is **already ready again**
5. So we send again, and again — the receive branch never wins

This is called **starvation** — one task starves the other of CPU time.

---

## The Fix: Two Independent Tasks

Embassy lets you spawn **separate tasks** that run concurrently:

```rust
#[embassy_executor::task]
async fn sender_task(mut sender: EspNowSender<'static>) {
    loop {
        ticker.next().await;  // only this task waits on the ticker
        sender.send_async(...).await;
    }
}

#[embassy_executor::task]
async fn receiver_task(mut receiver: EspNowReceiver<'static>) {
    loop {
        let msg = receiver.receive_async().await;  // always watching
        // process...
    }
}
```

The scheduler handles both tasks independently. When `sender_task` is blocked waiting for the radio, `receiver_task` is still polled and can receive packets.

---

## Why `split()`?

Rust won't allow two owners of the same value, so you can't give `esp_now` to two tasks. `split()` breaks it into separate pieces each task can own:

```
esp_now.split()
    ├── EspNowManager  → peer management (add/remove peers)
    ├── EspNowSender   → send_async()
    └── EspNowReceiver → receive_async()
```

---

## What the Official Duplex Example Does Differently

Comparing against [the official duplex example](https://github.com/esp-rs/esp-hal/tree/main/examples/esp-now/embassy_esp_now_duplex):

### `mk_static!` — We May Need This

The example promotes split handles to `'static` using a locally defined macro:

```rust
let manager = mk_static!(EspNowManager<'static>, manager);
let sender = mk_static!(
    Mutex::<NoopRawMutex, EspNowSender<'static>>,
    Mutex::<NoopRawMutex, _>::new(sender)
);
```

Our code passes sender and receiver directly into tasks. This *may* cause a lifetime compile error. If it does, add this macro and wrap accordingly — it is just a one-time static cell initialiser, not in any crate.

### Peer Discovery

The example's listener registers any unknown device it hears a broadcast from:

```rust
if !manager.peer_exists(&r.info.src_address) {
    manager.add_peer(PeerInfo {
        peer_address: r.info.src_address,
        encrypt: false,
        ...
    }).unwrap();
}
```

Once registered, the main loop sends **unicast** directly to that peer with ACK. Our code drops `_manager` and stays broadcast-only. See `architecture.md` for when and why to use unicast.

### Sender Mutex

The example wraps the sender in `Mutex<NoopRawMutex, EspNowSender>` because it is shared between two tasks. We only send from one task so we don't need this — but it is the pattern to follow if a second task ever needs to send.

| | Official Example | Our Code |
|---|---|---|
| `mk_static!` for tasks | Yes | No — may need it |
| Peer discovery | Yes, via `EspNowManager` | No — manager dropped |
| Delivery guarantee | Unicast with ACK after discovery | Broadcast only |
| Sender sharing | Mutex-wrapped | Not needed (1 sender) |

---

## Multi-Robot Implementation Requirements

### Per-Robot State Tracking

The current `QoS` struct tracks one sequence stream. With multiple robots broadcasting, all their packets arrive interleaved and the sequence tracking becomes meaningless — it will report constant drop storms.

You need a **per-robot state map** keyed on robot ID:

```rust
use heapless::FnvIndexMap;

struct RobotState {
    qos: QoS,
    last_payload: Payload,
    last_rssi: i8,
    last_seen: esp_hal::time::Instant,
}

// supports up to 16 robots in the swarm
type RobotMap = FnvIndexMap<u8, RobotState, 16>;
```

In `receiver_task`, look up or insert by `payload.robot_id` before processing.

### Stale Robot Detection

When a robot moves out of range it vanishes silently. Track `last_seen` per robot. If no packet arrives for ~200 ms (20 missed packets at 100 Hz), consider it gone and remove it from the map.

### Robot ID from MAC Address

Every robot currently sends `robot: 0`. Derive the ID from the WiFi MAC address at boot — guaranteed unique per device, no manual configuration:

```rust
// pseudocode — exact API depends on esp-hal version
let mac = esp_hal::peripherals::WIFI::mac_address();
let robot_id = mac[5]; // last byte of MAC
```

### Payload Needs Velocity and Heading

Position alone cannot predict a merge. Add velocity and heading so each robot can project the other's path forward:

```rust
struct Payload {
    seq: u16,
    robot_id: u8,
    pose: Pose,      // x, y, z
    velocity: f32,   // m/s
    heading: f32,    // radians
}
```

This adds 8 bytes. ESP-NOW allows up to 250 bytes per packet.

### RSSI as a Proximity Sensor

Every received packet carries signal strength in `data.info.rx_control.rssi` (more negative = weaker = further away). Store it in `RobotState` — a rising RSSI means a robot is closing, which is exactly the merge trigger condition.

### Transmission Jitter

All robots tick at exactly the same frequency from boot. Robots powered on together will synchronise their transmit cycles and collide on every tick. Add a small random delay before starting the sender ticker:

```rust
Timer::after(Duration::from_micros(some_random_seed % 10_000)).await;
let mut ticker = Ticker::every(Duration::from_hz(100));
```

---

## Further Reading

- [Embassy book](https://embassy.dev/book/) — the async embedded runtime
- [Embassy task docs](https://docs.embassy.dev/embassy-executor/git/cortex-m/attr.task.html) — how `#[embassy_executor::task]` works
- [esp-hal duplex example](https://github.com/esp-rs/esp-hal/tree/main/examples/esp-now/embassy_esp_now_duplex) — the official example this pattern is based on
