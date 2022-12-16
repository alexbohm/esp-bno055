#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c3_hal::{
    adc::{AdcConfig, ADC, ADC1},
    analog::SarAdcExt,
    clock::ClockControl,
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    pulse_control::{ClockSource, PulseControl},
    spi::{Spi, SpiMode},
    timer::TimerGroup,
    utils::{smartLedAdapter, SmartLedsAdapter},
    Delay, Rtc, UsbSerialJtag, IO,
};
use esp_backtrace as _;
use esp_println::println;
use heapless::String;
use smart_leds::{SmartLedsWrite, RGB8};

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

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    // let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let clocks = ClockControl::configure(
        system.clock_control,
        esp32c3_hal::clock::CpuClock::Clock160MHz,
    )
    .freeze();

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

    let mut delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // GPIO1
    let mut gpio1 = io.pins.gpio1.into_push_pull_output();
    gpio1.set_low().unwrap();

    // Onboard button.
    let onboard_button = io.pins.gpio9.into_pull_up_input();

    // Onboard RGB LED.
    let pulse = PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();
    let mut led = <smartLedAdapter!(1)>::new(pulse.channel0, io.pins.gpio8);

    const INTENSITY: u8 = 1;
    const BLUE: [RGB8; 1] = [RGB8 {
        r: 0,
        g: 0,
        b: INTENSITY,
    }];
    const PURPLE: [RGB8; 1] = [RGB8 {
        r: INTENSITY,
        g: 0,
        b: INTENSITY,
    }];

    const GREEN: [RGB8; 1] = [RGB8 {
        r: 0,
        g: INTENSITY,
        b: 0,
    }];
    const TEAL: [RGB8; 1] = [RGB8 {
        r: 0,
        g: INTENSITY,
        b: INTENSITY,
    }];
    const OFF: [RGB8; 1] = [RGB8 { r: 0, g: 0, b: 0 }];
    let mut color_toggle = true;

    // ////////////////////////////////////////
    // ADC Thermistor.
    // ////////////////////////////////////////

    // let analog = peripherals.APB_SARADC.split();
    // let mut adc1_config = AdcConfig::new();

    // let mut thermistor_pin = adc1_config.enable_pin(
    //     io.pins.gpio0.into_analog(),
    //     esp32c3_hal::adc::Attenuation::Attenuation11dB,
    // );

    // let mut adc1 = ADC::<ADC1>::adc(
    //     &mut system.peripheral_clock_control,
    //     analog.adc1,
    //     adc1_config,
    // )
    // .unwrap();

    // let mut read_thermistor = || -> u32 { nb::block!(adc1.read(&mut thermistor_pin)).unwrap() };

    // let mut value = read_thermistor();
    // let mut read_thermistor = || -> u32 {
    //     let current_value = read_thermistor();

    //     value = ((value * 90) + (current_value * 10)) / 100;

    //     value
    // };

    // let mut _read_thermistor_c = || -> f32 {
    //     let value: f32 = read_thermistor() as f32;

    //     let mv = (value * 3300.0) / 4096.0;

    //     // (mv - 500.0) / 10.0
    //     mv
    // };

    // loop {
    //     let value = read_thermistor_c();
    //     println!("Thermistor: {}C", value);
    // }
    // ////////////////////////////////////////
    // Initialize non-volatile storage.
    // ////////////////////////////////////////
    println!("Initializing SD Card...");
    led.write(BLUE.iter().cloned()).unwrap();

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
        5u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let timesource = EspTimeSource {};

    let mut sd_card =
        embedded_sdmmc::Controller::new(embedded_sdmmc::SdMmcSpi::new(spi, cs), timesource);

    sd_card.device().init().unwrap();

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
    // Initialize the IMU.
    // ////////////////////////////////////////
    println!("Initializing IMU...");

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio2,
        io.pins.gpio3,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut imu = bno055::Bno055::new(i2c).with_alternative_address();
    // delay.delay_ms(600u32);

    gpio1.set_high().unwrap();
    let result = imu.init(&mut delay);
    gpio1.set_low().unwrap();
    result.unwrap();

    // let mut try_init = || -> Result<(), bno055::Error<esp32c3_hal::i2c::Error>> {
    //     for _ in 0..5 {
    //         if let Ok(v) = imu.init(&mut delay) {
    //             return Ok(v);
    //         }
    //         delay.delay_ms(600u32);
    //     }
    //     imu.init(&mut delay)
    // };

    // try_init().unwrap();

    // Use the external crystal on the BNO055. According to the datasheet,
    // "It takes minimum ~600ms to configure the external crystal and startup the BNO055."
    imu.set_external_crystal(true, &mut delay).unwrap();
    delay.delay_ms(600u32);

    imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
        .unwrap();

    // Wait for the imu to become calibrated.
    {
        println!("Waiting for calibration...");

        while !imu.is_fully_calibrated().unwrap() {
            if color_toggle {
                led.write(PURPLE.iter().cloned()).unwrap();
            } else {
                led.write(OFF.iter().cloned()).unwrap();
            }
            color_toggle = !color_toggle;

            delay.delay_ms(100u32);
        }
    }

    // ////////////////////////////////////////
    // Take measurements.
    // ////////////////////////////////////////
    println!("Taking measurements...");

    let mut buffer: String<44> = String::new();
    loop {
        if let Ok(quat) = imu.quaternion() {
            let x = &quat.v.x;
            let y = &quat.v.y;
            let z = &quat.v.z;
            let s = &quat.s;

            println!("{{\"v\":{{\"x\":{x:.7},\"y\":{y:.7},\"z\":{z:.7}}},\"s\":{s:.7}}}");

            buffer.clear();
            writeln!(buffer, "{x:.7},{y:.7},{z:.7},{s:.7}").unwrap();

            // Write the buffer to disk, handling busy errors and panicking otherwise.
            {
                let mut written = 0;
                while written < buffer.len() {
                    match sd_card.write(&mut volume, &mut file, buffer[written..].as_bytes()) {
                        Ok(bytes_written) => {
                            written += bytes_written;
                        }
                        Err(embedded_sdmmc::Error::DeviceError(
                            embedded_sdmmc::SdMmcError::TimeoutWaitNotBusy,
                        )) => {}
                        Err(e) => {
                            panic!("{:?}", e);
                        }
                    }
                }
            }

            // Stop recording data if the button is pressed.
            if onboard_button.is_low().unwrap() {
                break;
            }

            // Activity LED.
            if color_toggle {
                led.write(TEAL.iter().cloned()).unwrap();
            } else {
                led.write(GREEN.iter().cloned()).unwrap();
            }
            color_toggle = !color_toggle;

            // Sampling period delay.
            delay.delay_ms(10u32);
        }
    }

    println!("Closing out file.");
    sd_card.close_file(&volume, file).unwrap();
    println!("Done writing file.");

    color_toggle = true;
    loop {
        if color_toggle {
            led.write(GREEN.iter().cloned()).unwrap();
        } else {
            led.write(OFF.iter().cloned()).unwrap();
        }
        color_toggle = !color_toggle;

        delay.delay_ms(500u32);
    }
}
