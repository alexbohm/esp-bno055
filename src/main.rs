#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_time::{Duration, Instant, Timer};
use esp32c3_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    spi::{Spi, SpiMode},
    system::SystemParts,
    timer::TimerGroup,
    Rtc, IO,
};
use esp_backtrace as _;
use esp_println::println;
use heapless::String;
struct EspTimeSource;

impl embedded_sdmmc::TimeSource for EspTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 30,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    // ////////////////////////////////////////
    // Initialize embassy.
    // ////////////////////////////////////////
    println!("Initializing Embassy...");
    let peripherals = Peripherals::take();

    let mut system: SystemParts = peripherals.SYSTEM.split();

    let clocks = ClockControl::configure(
        system.clock_control,
        esp32c3_hal::clock::CpuClock::Clock160MHz,
    )
    .freeze();

    esp32c3_hal::embassy::init(
        &clocks,
        esp32c3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );
    // ////////////////////////////////////////
    // "Normal Main"
    // ////////////////////////////////////////
    println!("Initializing Watchdogs...");

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // // GPIO1
    // let mut gpio1 = io.pins.gpio1.into_push_pull_output();
    // gpio1.set_low().unwrap();

    // // Onboard button.
    // let onboard_button = io.pins.gpio9.into_pull_up_input();

    // ////////////////////////////////////////
    // Initialize non-volatile storage.
    // ////////////////////////////////////////
    println!("Initializing SD Card...");

    // Initialize the SPI lines.
    let sclk = io.pins.gpio6;
    let miso = io.pins.gpio7;
    let mosi = io.pins.gpio5;
    let cs = io.pins.gpio4.into_push_pull_output();

    let spi = Spi::new_no_cs(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        400u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let timesource = EspTimeSource {};

    let mut sd_card =
        embedded_sdmmc::Controller::new(embedded_sdmmc::SdMmcSpi::new(spi, cs), timesource);

    // Initialize the sd card.
    sd_card.device().init().unwrap();
    // After initialization, we can boost the frequency of the bus.
    sd_card
        .device()
        .spi()
        .change_bus_frequency(80u32.MHz(), &clocks);

    let size = sd_card.device().card_size_bytes().unwrap();
    println!("Found SD Card:\n    size: {}", size);

    let mut volume = sd_card.get_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();

    let root_dir = sd_card.open_root_dir(&volume).unwrap();

    let mut file = sd_card
        .open_file_in_dir(
            &mut volume,
            &root_dir,
            "data.csv",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        )
        .unwrap();

    // ////////////////////////////////////////
    // Take measurements.
    // ////////////////////////////////////////

    // Buffer of data to write.
    let mut buffer: String<2048> = String::new();
    for _ in 0..buffer.capacity() / 16 {
        buffer.push_str("0123456789ABCDEF").unwrap();
    }
    println!("Using buffer of size: {}", buffer.len());

    // Capture benchmark start time.
    println!("Taking measurements...");
    let before = Instant::now();

    // Write the buffer to disk, handling busy errors and panicking otherwise.
    for chunk_index in 0..1_000_000 {
        let chunk_before = Instant::now();
        let mut busy_errors: u32 = 0;
        let mut written = 0;
        while written < buffer.len() {
            match sd_card.write(&mut volume, &mut file, buffer[written..].as_bytes()) {
                Ok(bytes_written) => {
                    written += bytes_written;
                }
                Err(embedded_sdmmc::Error::DeviceError(
                    embedded_sdmmc::SdMmcError::TimeoutWaitNotBusy,
                )) => {
                    busy_errors += 1;
                }
                Err(e) => {
                    panic!("{:?}", e);
                }
            }
        }

        let bytespersec = buffer.len() as u64 * 1_000_000 / chunk_before.elapsed().as_micros();
        println!("Chunk {chunk_index} Data Rate: {bytespersec} Busy Errors: {busy_errors}");
    }
    sd_card.close_file(&volume, file).unwrap();

    let elapsed = before.elapsed().as_millis();

    let bytespersec = buffer.len() as u64 * 1_000_000 * 1_000_000 / before.elapsed().as_micros();

    println!("Done writing file.");
    println!("Milliseconds Elapsed: {elapsed}");
    println!("Data Rate: {bytespersec}");

    // // Stop recording data if the button is pressed.
    // if onboard_button.is_low().unwrap() {
    //     break;
    // }

    loop {
        println!("Done.");
        Timer::after(Duration::from_millis(1000u64)).await;
    }
}
