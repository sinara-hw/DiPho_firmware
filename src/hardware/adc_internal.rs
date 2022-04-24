//use embedded_hal::adc::Channel;

use super::hal::{
    adc,
    gpio::{gpioa::*, Analog},
    prelude::*,
    rcc::Clocks,
    stm32::ADC1,
};

use systick_monotonic::{fugit::Hertz, *};

pub enum AdcChannel {
    AfeOutput,
    CalOutput,
    BiasOutput,
}

pub struct AdcInternalPins {
    pub afe_output: PA0<Analog>,
    pub cal_output: PA1<Analog>,
    pub bias_output: PA2<Analog>,
}

pub struct AdcInternal {
    adc1: adc::Adc<ADC1>,
    pins: AdcInternalPins,
}

impl AdcInternal {
    pub fn new(clocks: &Clocks, adc: ADC1, pins: AdcInternalPins) -> Self {
        let adc1 = adc::Adc::adc1(adc, *clocks);

        AdcInternal { adc1, pins }
    }

    pub fn read(&mut self, ch: AdcChannel) -> f32 {
        match ch {
            AdcChannel::AfeOutput => self.read_afe_output_voltage(),
            AdcChannel::CalOutput => self.read_cal_resistance(),
            AdcChannel::BiasOutput => self.read_bias_output_voltage(),
        }
    }

    pub fn read_afe_output_voltage(&mut self) -> f32 {
        let p = &mut self.pins.afe_output;
        let code: u16 = self.adc1.read(p).unwrap();

        //TODO: processing here
        code as f32
    }

    pub fn read_cal_resistance(&mut self) -> f32 {
        let p = &mut self.pins.afe_output;
        let code: u16 = self.adc1.read(p).unwrap();

        //TODO: processing here
        code as f32
    }

    pub fn read_bias_output_voltage(&mut self) -> f32 {
        let p = &mut self.pins.afe_output;
        let code: u16 = self.adc1.read(p).unwrap();

        //TODO: processing here
        code as f32
    }

    pub fn read_afe_raw(&mut self) -> u16 {
        let p = &mut self.pins.afe_output;
        let readout: u16 = self.adc1.read(p).unwrap();
        readout
    }

    pub fn read_cal_raw(&mut self) -> u16 {
        let p = &mut self.pins.afe_output;
        let readout: u16 = self.adc1.read(p).unwrap();
        readout
    }

    pub fn read_bias_raw(&mut self) -> u16 {
        let p = &mut self.pins.afe_output;
        let readout: u16 = self.adc1.read(p).unwrap();
        readout
    }

    pub fn read_vref(&mut self) -> u16 {
        self.adc1.read_vref()
    }
}
