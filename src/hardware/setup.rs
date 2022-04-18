
//use stm32f1xx_hal::hal::digital::v2::OutputPin;
//use stm32f1xx_hal::


use super::hal::{
    self as hal,
    gpio::GpioExt,
    prelude::*,
    usb::{Peripheral, UsbBus, UsbBusType},
    
};

use super::{
    adc_internal::{AdcInternal, AdcInternalPins},
    gpio::{Gpio, GpioPins},
    
};

//use stm32f1xx_hal::afio::MAPR as afio;
use usb_device::prelude::*;
//use systick_monotonic::*;
use cortex_m::asm::delay;
use defmt::info;

pub struct DiPhoDevices {
    pub gpio: Gpio,
    pub adc_internal: AdcInternal,
    pub usb_dev: UsbDevice<'static, UsbBusType>,
    pub serial_config: usbd_serial::SerialPort<'static, UsbBusType>,
    pub serial_data: usbd_serial::SerialPort<'static, UsbBusType>,
}

pub fn setup(
    device: stm32f1xx_hal::stm32::Peripherals,
) -> DiPhoDevices {

    let mut flash = device.FLASH.constrain();
    let mut rcc = device.RCC.constrain();
    let ccdr = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .freeze(&mut flash.acr);

    assert!(ccdr.usbclk_valid());

    



    info!("--- Starting hardware setup ---");
    


    //let mut delay = delay::AsmDelay::new(ccdr.clocks.c_ck().0);

    let mut gpioa = device.GPIOA.split();
    let mut gpiob = device.GPIOB.split();

    info!("Disable JTAG");
    let mut afio = device.AFIO.constrain();
    let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    


    info!("Setup GPIO");

    let pa15 = pa15.into_push_pull_output(&mut gpioa.crh);
    let pb3 = pb3.into_push_pull_output(&mut gpiob.crl);
    //let pb4 = pb4.into_push_pull_output(&mut gpiob.crl);

    let gpio_pins = GpioPins {
        led: 
            gpioa.pa8.into_push_pull_output(&mut gpioa.crh),
        gain_sel: (
            pb3,
            pa15,
        ),
        afe_type_ind:
            gpiob.pb2.into_floating_input(&mut gpiob.crl),

    };

    let mut gpio = Gpio::new(gpio_pins);

    info!("AFE type: {}", gpio.get_afe_type());

    info!("Setup USB");

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low();
    delay(ccdr.sysclk().raw() / 100);

    let usb_dm = gpioa.pa11;
    let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

    let usb = Peripheral {
        usb: device.USB,
        pin_dm: usb_dm,
        pin_dp: usb_dp,
    };

    unsafe {
        USB_BUS.replace(UsbBus::new(usb));
    }

    let serial_config = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
    let serial_data = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

    let usb_dev = UsbDeviceBuilder::new(
        unsafe { USB_BUS.as_ref().unwrap() },
        UsbVidPid(0x16c0, 0x27dd),
    )
    .manufacturer("Sinara")
    .product("DiPho")
    .serial_number("DiPho")
    .device_class(usbd_serial::USB_CLASS_CDC)
    .build();

    info!("Setup internal ADC");

    info!("Setup CPU ADCs");
    let adc_internal_pins = AdcInternalPins {
        afe_output: 
            gpioa.pa0.into_analog(&mut gpioa.crl),
        cal_output: 
            gpioa.pa1.into_analog(&mut gpioa.crl),
        bias_output:
            gpioa.pa2.into_analog(&mut gpioa.crl),
    };

    let mut adc_internal = AdcInternal::new(
        &ccdr,
        device.ADC1,
        adc_internal_pins,
    );

    info!("AFE: {} V", adc_internal.read_afe_raw());
    info!("CAL: {} V", adc_internal.read_cal_raw());
    info!("BIAS: {} V", adc_internal.read_bias_raw());
    
    info!("--- Hardware setup done! ---");

    DiPhoDevices {
        gpio,
        adc_internal,
        usb_dev,
        serial_config,
        serial_data,

    }


}