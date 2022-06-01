#![no_main]
#![no_std]
#![feature(str_split_whitespace_as_str)]

//use panic_semihosting as _;
//use embedded_hal as hal;

pub mod hardware;

//use core::time::Duration;

use defmt::{info, Format};
use defmt_rtt as _;
use hal::pac::sdio::resp;
// global logger
use panic_probe as _; // gloibal panic handler
                      //use defmt_semihosting as _; // global logger
                      //use panic_probe as _; // gloibal panic handler

use hardware::{
    adc_internal::AdcInternal,
    gpio::{AfeType, GainSetting, Gpio},
    hal,
};

use arrform::*;
use core::str;
use cortex_m::asm::delay;
use rtic;
use stm32f1xx_hal::adc::SampleTime;
use stm32f1xx_hal::pac::TIM3;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::timer::{Counter, CounterUs, Event, Timer};
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use systick_monotonic::{fugit::Hertz, fugit::Rate, *};
use usb_device::prelude::*;

#[derive(Clone, Copy, Debug)]
pub struct DeviceSettings {
    pub sampling_time: SampleTime,
    pub sampling_freq: Hertz<u32>,
    //pub data_interval: systick_monotonic::fugit::Duration<u64, 1, 1000>,
    pub gain: GainSetting,
    pub led_off: bool,
}

impl Default for DeviceSettings {
    fn default() -> Self {
        Self {
            sampling_time: SampleTime::T_239,
            sampling_freq: 10.kHz(),
            //data_interval: (1000 as u64).millis(),
            gain: GainSetting::Low,
            led_off: true,
        }
    }
}

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [CAN_SCE, CAN_RX1])]
mod app {

    use super::*;

    // use cortex_m::asm::delay;
    // use stm32f1xx_hal::prelude::*;
    // use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
    // use systick_monotonic::*;
    // use usb_device::prelude::*;

    #[monotonic(binds=SysTick, default=true)]
    type Mono = Systick<1_000>; // 1 kHz / 1ms granularity

    #[shared]
    struct Shared {
        adc_internal: AdcInternal,
        adc_counter: CounterUs<TIM3>,
        device_settings: DeviceSettings,
        gpio: Gpio,
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial_config: usbd_serial::SerialPort<'static, UsbBusType>,
        serial_data: usbd_serial::SerialPort<'static, UsbBusType>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let systick = cx.core.SYST;
        let mut mono = Systick::new(systick, 48_000_000);
        //let clock = SystemTimer::;

        let dipho = hardware::setup::setup(cx.device);
        update_settings_task::spawn(mono.now()).unwrap();

        let local = Local {};
        let shared = Shared {
            adc_internal: dipho.adc_internal,
            adc_counter: dipho.adc_counter,
            device_settings: DeviceSettings::default(),
            gpio: dipho.gpio,
            usb_dev: dipho.usb_dev,
            serial_config: dipho.serial_config,
            serial_data: dipho.serial_data,
        };

        (shared, local, init::Monotonics(mono))
    }

    #[task(binds = TIM3, shared=[adc_internal, adc_counter, serial_data])]
    fn transmit_adc_data(cx: transmit_adc_data::Context) {
        let mut adc_internal = cx.shared.adc_internal;
        let mut adc_counter = cx.shared.adc_counter;
        let mut serial_data = cx.shared.serial_data;
        let mut data: f32 = 0.00;
        (&mut adc_counter).lock(|adc_counter| {
            adc_counter.clear_interrupt(Event::Update);
        });
        (&mut adc_internal).lock(|adc_internal| {
            data = adc_internal.read_afe_output_voltage();
        });

        let dtbs = arrform!(32, "{:.6} mA \r\n", data);

        (&mut serial_data).lock(|serial_data| {
            serial_data.write(dtbs.as_bytes()).ok();
        });
    }

    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, serial_config, serial_data, device_settings, adc_counter])]
    fn usb_tx(cx: usb_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial_config = cx.shared.serial_config;
        let mut serial_data = cx.shared.serial_data;
        let mut device_settings = cx.shared.device_settings;
        let mut adc_counter = cx.shared.adc_counter;

        (
            &mut usb_dev,
            &mut serial_config,
            &mut serial_data,
            &mut device_settings,
            &mut adc_counter,
        )
            .lock(
                |usb_dev, serial_config, serial_data, device_settings, adc_counter| {
                    super::usb_poll(
                        usb_dev,
                        serial_config,
                        serial_data,
                        device_settings,
                        adc_counter,
                    );
                },
            );
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, serial_config, serial_data, device_settings, adc_counter])]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial_config = cx.shared.serial_config;
        let mut serial_data = cx.shared.serial_data;
        let mut device_settings = cx.shared.device_settings;
        let mut adc_counter = cx.shared.adc_counter;

        (
            &mut usb_dev,
            &mut serial_config,
            &mut serial_data,
            &mut device_settings,
            &mut adc_counter,
        )
            .lock(
                |usb_dev, serial_config, serial_data, device_settings, adc_counter| {
                    super::usb_poll(
                        usb_dev,
                        serial_config,
                        serial_data,
                        device_settings,
                        adc_counter,
                    );
                },
            );
    }

    #[task(priority = 1, shared=[gpio, adc_internal, device_settings])]
    fn update_settings_task(
        mut cx: update_settings_task::Context,
        instant: <Mono as rtic::Monotonic>::Instant,
    ) {
        let mut gpio = cx.shared.gpio;
        let mut adc_internal = cx.shared.adc_internal;
        let mut device_settings = cx.shared.device_settings;

        (&mut gpio, &mut adc_internal, &mut device_settings).lock(
            |gpio, adc_internal, device_settings| {
                gpio.set_gain(device_settings.gain);
                adc_internal.set_gain(device_settings.gain);
                adc_internal.set_sample_time(device_settings.sampling_time);
            },
        );
        let next_instant = instant + (500 as u64).millis();
        update_settings_task::spawn_at(next_instant, next_instant).unwrap();
    }
}

fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
    serial_config: &mut usbd_serial::SerialPort<'static, B>,
    serial_data: &mut usbd_serial::SerialPort<'static, B>,
    device_settings: &mut DeviceSettings,
    adc_counter: &mut CounterUs<TIM3>,
) {
    if !usb_dev.poll(&mut [serial_config, serial_data]) {
        return;
    }

    let mut buf: [u8; 64] = [32; 64];
    //let buf: &mut [u8];

    match serial_config.read(&mut buf) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            let mut cfg_str = str::from_utf8(&buf).unwrap();
            let mut cfg_str_split = cfg_str.split_ascii_whitespace();

            let af: ArrForm<64>;
            let response: &str;
            match cfg_str_split.next().unwrap_or_else(|| "") {
                "help" => response = "<help text>\r\n",
                "get" => {
                    match cfg_str_split.next().unwrap_or_else(|| "") {
                        "afe_type" => response = "<afe type>\r\n",
                        "gain" => {
                            af = arrform!(64, "gain setting: {:?} \r\n", device_settings.gain);
                            response = af.as_str();
                        }
                        "sampling_time" => {
                            af = arrform!(
                                64,
                                "gain setting: {:?} \r\n",
                                device_settings.sampling_time
                            );
                            response = af.as_str();
                        }
                        "sampling_freq" => {
                            af = arrform!(
                                64,
                                "gain setting: {:?} \r\n",
                                device_settings.sampling_freq
                            );
                            adc_counter
                                .start(device_settings.sampling_freq.into_duration())
                                .unwrap();
                            response = af.as_str();
                        }
                        _ => response = "get command error: unknown field!\r\n",
                    };
                }
                "set" => {
                    match cfg_str_split.next().unwrap_or_else(|| "") {
                        "afe_type" => response = "<afe type\r\n",
                        "gain" => match cfg_str_split.next().unwrap_or_else(|| "") {
                            "Low" | "low" | "LOW" | "lo" | "LO" => {
                                response = "set gain: \"Low\" ok!\r\n";
                                device_settings.gain = GainSetting::Low;
                            }
                            "Medium" | "medium" | "MEDIUM" | "Med" | "med" | "MED" => {
                                response = "set gain: \"Medium\" ok!\r\n";
                                device_settings.gain = GainSetting::Medium;
                            }
                            "High" | "high" | "HIGH" | "hi" | "HI" => {
                                response = "set gain: \"High\" ok!\r\n";
                                device_settings.gain = GainSetting::High;
                            }
                            "Max" | "max" | "MAX" => {
                                response = "set gain: \"Max\" ok!\r\n";
                                device_settings.gain = GainSetting::Max;
                            }
                            _ => response = "set gain error: invalid value!\r\n",
                        },
                        "sampling_time" => match cfg_str_split.next().unwrap_or_else(|| "") {
                            "1" => {
                                response = "set sampling time: 1.5 cycles ok!";
                                device_settings.sampling_time = SampleTime::T_1;
                            }
                            "7" => {
                                response = "set sampling time: 7.5 cycles ok!";
                                device_settings.sampling_time = SampleTime::T_7;
                            }
                            "13" => {
                                response = "set sampling time: 13.5 cycles ok!";
                                device_settings.sampling_time = SampleTime::T_13;
                            }
                            "28" => {
                                response = "set sampling time: 28.5 cycles ok!";
                                device_settings.sampling_time = SampleTime::T_28;
                            }
                            "41" => {
                                response = "set sampling time: 41.5 cycles ok!";
                                device_settings.sampling_time = SampleTime::T_41;
                            }
                            "55" => {
                                response = "set sampling time: 55.5 cycles ok!";
                                device_settings.sampling_time = SampleTime::T_55;
                            }
                            "71" => {
                                response = "set sampling time: 71.5 cycles ok!";
                                device_settings.sampling_time = SampleTime::T_71;
                            }
                            "239" => {
                                response = "set sampling time: 239.5 cycles ok!";
                                device_settings.sampling_time = SampleTime::T_239;
                            }

                            _ => response = "set sampling time error: invalid value!\r\n",
                        },
                        "sampling_freq" => {
                            let freq = cfg_str_split
                                .next()
                                .unwrap_or_else(|| "")
                                .parse::<u32>()
                                .unwrap_or_else(|_| 0 as u32);
                            if freq <= 15 {
                                response = "set sampling frequency error: sampling frequency must be higher than 15Hz!\r\n";
                            } else if freq > 50_000 {
                                response = "set sampling frequency error: sampling frequency must not be higher than 50kHz!\r\n";
                            } else {
                                let rate: Rate<u32, 1, 1> = freq.Hz();
                                device_settings.sampling_freq = rate;
                                af = arrform!(
                                    64,
                                    "set sampling frequency: {:?} ok!\r\n",
                                    device_settings.sampling_freq
                                );
                                response = af.as_str();
                                adc_counter
                                    .start(device_settings.sampling_freq.into_duration())
                                    .unwrap();
                            }
                        }
                        _ => response = "set command error: unknown field!\r\n",
                    };
                }
                _ => response = "unknown command",
            }

            serial_config.write(response.as_bytes()).ok();
            //serial_config.write(&buf).ok();
        }
        _ => {}
    }
}
