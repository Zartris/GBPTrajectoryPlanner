#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    efuse::{self, InterfaceMacAddress},
    timer::timg::TimerGroup,
};
use esp_radio::{esp_now::WifiPhyRate, wifi::Protocols};
use esp32c5_rust_template::comms::{RobotMap, receiver_task, sender_task};
use heapless::index_map::FnvIndexMap;
use static_cell::StaticCell;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The example suggests 72kB but it doesn't fit in the dram
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
