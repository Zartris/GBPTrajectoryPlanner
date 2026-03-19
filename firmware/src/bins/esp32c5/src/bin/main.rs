#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    efuse::{self, InterfaceMacAddress},
    timer::timg::TimerGroup,
};
use esp_radio::{
    esp_now::{BROADCAST_ADDRESS, EspNowReceiver, EspNowSender, WifiPhyRate},
    wifi::Protocols,
};
use heapless::{HistoryBuf, index_map::FnvIndexMap};
use serde::{Deserialize, Serialize};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[derive(Serialize, Deserialize, defmt::Format)]
struct Pose {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Serialize, Deserialize, defmt::Format)]
struct Payload {
    seq: u16,
    robot: u8,
    pose: Pose,
}

struct QoS {
    dropped_packets: u32,
    total_packets: u32,
    drop_events: HistoryBuf<(u16, u16), 5>,
    average_freq: f32,
    package_loss: f32,
    last_seq: Option<u16>,
    last_timestamp: esp_hal::time::Instant,
}

impl Default for QoS {
    fn default() -> Self {
        Self {
            dropped_packets: 0,
            total_packets: 0,
            drop_events: Default::default(),
            average_freq: Default::default(),
            package_loss: Default::default(),
            last_seq: None,
            last_timestamp: esp_hal::time::Instant::now(),
        }
    }
}

impl QoS {
    const EMA_ALPHA: f32 = 0.1;

    pub fn process_message(&mut self, payload: &Payload, timestamp: esp_hal::time::Instant) {
        let Some(last_seq) = self.last_seq else {
            self.last_seq = Some(payload.seq);
            self.last_timestamp = timestamp;
            return;
        };

        if payload.seq != last_seq.wrapping_add(1) {
            self.drop_events.write((last_seq, payload.seq));
            let gap = payload.seq.wrapping_sub(last_seq.wrapping_add(1)) as u32;
            self.dropped_packets += gap;
        }
        self.total_packets += 1;
        self.package_loss = 100.0 * self.dropped_packets as f32
            / (self.total_packets + self.dropped_packets) as f32;

        let freq = 1.0E6 / (timestamp - self.last_timestamp).as_micros() as f32;
        self.average_freq = Self::EMA_ALPHA * freq + (1.0 - Self::EMA_ALPHA) * self.average_freq;

        self.last_seq = Some(payload.seq);
        self.last_timestamp = timestamp;
    }

    pub fn report(&self, robot_id: u8) {
        info!(
            "Robot {}: dropped {} packets, loss: {}%, freq: {} Hz",
            robot_id, self.dropped_packets, self.package_loss, self.average_freq
        );
        info!("Last 5 drop events:");
        self.drop_events
            .iter()
            .for_each(|x| info!("\t{}\t->\t{}", x.0, x.1));
    }
}

struct RobotState {
    qos: QoS,
    last_seen: Instant,
}

impl RobotState {
    fn new() -> Self {
        Self {
            qos: QoS::default(),
            last_seen: Instant::now(),
        }
    }
}

// How long without a packet before a robot is considered out of range.
// Tune this to match your broadcast rate (500 ms = 5 missed packets at 100 Hz).
const STALE_TIMEOUT: Duration = Duration::from_millis(500);

// Maximum number of robots tracked simultaneously.
type RobotMap = FnvIndexMap<u8, RobotState, 16>;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn sender_task(mut sender: EspNowSender<'static>, robot_id: u8) {
    let mut ticker = Ticker::every(Duration::from_hz(100));
    let mut seq: u16 = 0;
    loop {
        ticker.next().await;
        let payload = Payload {
            seq,
            robot: robot_id,
            pose: Pose {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        };
        let mut buf = [0u8; 20];
        match postcard::to_slice(&payload, &mut buf) {
            Ok(_) => match sender.send_async(&BROADCAST_ADDRESS, &buf).await {
                Ok(_) => seq = seq.wrapping_add(1),
                Err(e) => defmt::warn!("Failed to send: {:?}", e),
            },
            Err(_) => defmt::warn!("Serialization failed"),
        }
    }
}

#[embassy_executor::task]
async fn receiver_task(mut receiver: EspNowReceiver<'static>, robots: &'static mut RobotMap) {
    let mut last_report = Instant::now();
    loop {
        let received = receiver.receive_async().await;
        let timestamp = received.info.rx_control.timestamp;

        // Deserialize the payload. If it fails, skip this packet but keep listening for more.
        let payload = match postcard::from_bytes::<Payload>(received.data()) {
            Ok(p) => p,
            Err(_) => {
                defmt::warn!("Failed to deserialize payload");
                continue;
            }
        };

        // Update QoS for this robot, or add it to the map if it's new.
        match robots.entry(payload.robot).or_insert_with(|| {
            info!("New robot discovered: {}", payload.robot);
            RobotState::new()
        }) {
            Ok(state) => {
                state.last_seen = Instant::now();
                state.qos.process_message(&payload, timestamp);
            }
            Err(_) => defmt::warn!("Robot map full, ignoring robot {}", payload.robot),
        }

        // Purge robots we haven't heard from within the stale timeout.
        let now = Instant::now();
        let mut stale: heapless::Vec<u8, 16> = heapless::Vec::new();
        for (id, state) in robots.iter() {
            if now
                .checked_duration_since(state.last_seen)
                .is_some_and(|d| d >= STALE_TIMEOUT)
            {
                let _ = stale.push(*id);
            }
        }
        for id in stale.iter() {
            robots.remove(id);
            info!("Robot {} out of range, removed", id);
        }

        // Write QoS report every 10 seconds for all robots.
        if last_report.elapsed() >= Duration::from_secs(10) {
            last_report = Instant::now();
            for (id, state) in robots.iter() {
                state.qos.report(*id);
            }
        }
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The example suggets 72kB but it doesn't fit in the dram
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let wifi = peripherals.WIFI;
    let (mut controller, interfaces) = esp_radio::wifi::new(wifi, Default::default()).unwrap();

    controller.set_protocols(Protocols::default()).unwrap();

    let esp_now = interfaces.esp_now;
    esp_now.set_channel(40).expect("Failed to set channel");
    esp_now
        .set_rate(WifiPhyRate::RateMcs7Lgi)
        .expect("Failed to set rate");

    info!("esp-now version {}", esp_now.version().unwrap());

    let mac = efuse::interface_mac_address(InterfaceMacAddress::Station);
    let robot_id = mac.as_bytes()[5];
    info!("Robot ID: {} (MAC: {:x})", robot_id, mac.as_bytes());

    let (_manager, sender, receiver) = esp_now.split();
    spawner.spawn(sender_task(sender, robot_id)).unwrap();
    spawner
        .spawn(receiver_task(
            receiver,
            mk_static!(RobotMap, FnvIndexMap::new()),
        ))
        .unwrap();

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(3600)).await;
    }
}
