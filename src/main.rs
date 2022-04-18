#![no_main]
#![no_std]

use panic_semihosting as _;
//use embedded_hal as hal;

pub mod hardware;

use defmt::{info, Format};
//use defmt_rtt as _; // global logger
//use panic_probe as _; // gloibal panic handler

use hardware::{
    adc_internal::AdcInternal,
    gpio::{AfeType, GainSetting, Gpio},
    hal,
};

use cortex_m::asm::delay;
use rtic;
use stm32f1xx_hal::adc::SampleTime;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use systick_monotonic::*;
use usb_device::prelude::*;

#[derive(Clone, Copy, Debug)]
pub struct DeviceSettings {
    pub sampling_time: SampleTime,
    pub data_interval: u32,
    pub gain: GainSetting,
    pub led_off: bool,
}

impl Default for DeviceSettings {
    fn default() -> Self {
        Self {
            sampling_time: SampleTime::T_239,
            data_interval: 100, //miliseconds
            gain: GainSetting::Low,
            led_off: true,
        }
    }
}

#[rtic::app(device = stm32f1xx_hal::pac)]
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
        device_settings: DeviceSettings,
        gpio: Gpio,
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial_config: usbd_serial::SerialPort<'static, UsbBusType>,
        serial_data: usbd_serial::SerialPort<'static, UsbBusType>,
    }

    #[local]
    struct Local {
        adc_internal: AdcInternal,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let systick = cx.core.SYST;
        let mono = Systick::new(systick, 48_000_000);
        //let clock = SystemTimer::;

        let dipho = hardware::setup::setup(cx.device);

        let local = Local {
            adc_internal: dipho.adc_internal,
        };

        let shared = Shared {
            device_settings: DeviceSettings::default(),
            gpio: dipho.gpio,
            usb_dev: dipho.usb_dev,
            serial_config: dipho.serial_config,
            serial_data: dipho.serial_data,
        };

        (shared, local, init::Monotonics(mono))
    }

    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, serial_config, serial_data])]
    fn usb_tx(cx: usb_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial_config = cx.shared.serial_config;
        let mut serial_data = cx.shared.serial_data;

        (&mut usb_dev, &mut serial_config, &mut serial_data).lock(
            |usb_dev, serial_config, serial_data| {
                super::usb_poll(usb_dev, serial_config, serial_data);
            },
        );
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, serial_config, serial_data])]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial_config = cx.shared.serial_config;
        let mut serial_data = cx.shared.serial_data;

        (&mut usb_dev, &mut serial_config, &mut serial_data).lock(
            |usb_dev, serial_config, serial_data| {
                super::usb_poll(usb_dev, serial_config, serial_data);
            },
        );
    }
}

fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
    serial: &mut usbd_serial::SerialPort<'static, B>,
    serial2: &mut usbd_serial::SerialPort<'static, B>,
) {
    if !usb_dev.poll(&mut [serial, serial2]) {
        return;
    }

    let mut buf = [0u8; 8];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            for c in buf[0..count].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }

            serial2.write(&buf[0..count]).ok();
        }
        _ => {}
    }
}
