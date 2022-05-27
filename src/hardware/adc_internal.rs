//use embedded_hal::adc::Channel;
use super::gpio::GainSetting;

use super::hal::{
    adc,
    adc::SampleTime,
    gpio::{gpioa::*, Analog},
    prelude::*,
    rcc::Clocks,
    stm32::ADC1,
};

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

pub struct AdcInternalSettings {
    pub sampling_time: SampleTime,
    pub gain: GainSetting,
    pub switch_resistance: f32,
}

pub struct AdcInternal {
    adc1: adc::Adc<ADC1>,
    pins: AdcInternalPins,
    settings: AdcInternalSettings,
}

impl AdcInternal {
    pub fn new(clocks: &Clocks, adc: ADC1, pins: AdcInternalPins, settings: AdcInternalSettings) -> Self {
        let mut adc1 = adc::Adc::adc1(adc, *clocks);
        adc1.set_sample_time(settings.sampling_time);
        AdcInternal { adc1, pins , settings}
    }

    pub fn calibrate_tia(&mut self){
        self.settings.switch_resistance = self.read_cal_resistance();
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
        let voltage: f32 = (code as f32) / (4096.0 * 3.3);
        let gain_resistance: f32;
        //TODO: add support for logarithmic amp output
        match self.settings.gain {
            GainSetting::Low => gain_resistance = 294.0,
            GainSetting::Medium => gain_resistance = 3090.0,
            GainSetting::High => gain_resistance = 32_400.0,
            GainSetting::Max => gain_resistance = 324_000.0,
        }
        let photocurrent: f32 = voltage / (gain_resistance + self.settings.switch_resistance);
        photocurrent
    }

    pub fn set_gain(&mut self, gain: GainSetting) {
        self.settings.gain = gain;
    }

    pub fn set_sample_time(&mut self, sample_time: SampleTime) {
        self.adc1.set_sample_time(sample_time);
    }


    pub fn read_cal_resistance(&mut self) -> f32 {
        let p = &mut self.pins.afe_output;
        let code: u16 = self.adc1.read(p).unwrap();
        let voltage: f32 = (code as f32) / (4096.0 * 3.3);
        let switch_resistance: f32 = 6650.0 * 3.3 / voltage;
        switch_resistance
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
