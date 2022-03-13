#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m::asm::delay;
use systick_monotonic::*;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::bus;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use rtic::app;

#[app(device = stm32f1xx_hal::stm32, peripherals = true)]
mod app {
    use super::*;

    #[monotonic(binds=SysTick, default=true)]
    type SysMono = Systick<1_000>; // 1 kHz / 1ms granularity

    #[shared]
    struct SharedResources {   
    }

    #[local]
    struct LocalResources {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,     
    }

    #[init(local = [USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None])]
    fn init(cx: init::Context) -> (SharedResources, LocalResources,  init::Monotonics) {

        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        // Initialize the monotonic
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().0);

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);

        //for development only
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().unwrap();
        delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        *cx.local.USB_BUS = Some(UsbBus::new(usb));

        let serial = SerialPort::new(cx.local.USB_BUS.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(cx.local.USB_BUS.as_ref().unwrap(), UsbVidPid(0x1600, 0x2137))
            .manufacturer("Sinara")
            .product("DiPho")
            .serial_number("DiPho")
            .device_class(USB_CLASS_CDC)
            .build();

            (
                SharedResources {
                },
                LocalResources {
                    usb_dev,
                    serial
                },
                init::Monotonics(mono),
            )
    }

    #[task(binds = USB_LP_CAN_RX0, local = [usb_dev, serial])]
    fn usb_rx0(mut cx: usb_rx0::Context) {
        usb_poll(&mut cx.local.usb_dev, &mut cx.local.serial);
    }

    fn usb_poll<B: bus::UsbBus>(
        usb_dev: &mut UsbDevice<'static, B>,
        serial: &mut SerialPort<'static, B>,
    ) {
        if !usb_dev.poll(&mut [serial]) {
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
    
                serial.write(&buf[0..count]).ok();
            }
            _ => {}
        }
    }
}

