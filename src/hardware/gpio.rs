use super::hal::gpio::{gpioa::*, gpiob::*, Floating, Input, Output, PinState, PushPull};
use defmt::Format;
//use embedded_hal as hal;
//use hal::digital::v2::{InputPin, OutputPin, PinState};

#[derive(Copy, Clone, Debug, Format)]
pub enum AfeType {
    /// Default (transimpedance) amplifier is main part of analog front-end fitted on daughterboard
    Transimpedance,
    /// Alternative (logarithmic) amplifier is main part of analog front-end fitted on daughterboard
    Logarithmic,
}

#[derive(Copy, Clone, Debug, Format)]
pub enum GainSetting {
    /// Low gain (around 200/300) was selected in config (all gain settings are applicable only for TIA configuration)
    Low,
    /// Medium gain (around 3100/3300)
    Medium,
    /// High gain (around 32.4k/32.5k)
    High,
    /// Maximum gain (around 324k)
    Max,
}

impl Default for GainSetting {
    fn default() -> Self {
        Self::Low
    }
}
#[allow(clippy::type_complexity)]
pub struct GpioPins {
    pub led: PA8<Output<PushPull>>,
    pub gain_sel: (PB3<Output<PushPull>>, PA15<Output<PushPull>>),
    pub afe_type_ind: PB2<Input<Floating>>,
}

#[derive(Copy, Clone, Debug)]
pub enum State {
    Assert,
    Deassert,
}

impl From<bool> for State {
    fn from(other: bool) -> State {
        match other {
            true => State::Assert,
            false => State::Deassert,
        }
    }
}

impl From<State> for PinState {
    fn from(other: State) -> PinState {
        match other {
            State::Assert => PinState::High,
            State::Deassert => PinState::Low,
        }
    }
}

pub struct Gpio {
    pins: GpioPins,
}

impl Gpio {
    pub fn new(pins: GpioPins) -> Self {
        let mut gpio = Gpio { pins };
        gpio.set_gain(GainSetting::Low);
        gpio.set_led(PinState::Low);
        gpio
    }

    pub fn set_gain(&mut self, gain: GainSetting) {
        match gain {
            GainSetting::Low => {
                self.pins.gain_sel.0.set_state(PinState::High);
                self.pins.gain_sel.1.set_state(PinState::High);
            }
            GainSetting::Medium => {
                self.pins.gain_sel.0.set_state(PinState::High);
                self.pins.gain_sel.1.set_state(PinState::Low);
            }
            GainSetting::High => {
                self.pins.gain_sel.0.set_state(PinState::Low);
                self.pins.gain_sel.1.set_state(PinState::High);
            }
            GainSetting::Max => {
                self.pins.gain_sel.0.set_state(PinState::Low);
                self.pins.gain_sel.1.set_state(PinState::Low);
            }
        }
    }

    pub fn get_gain(&mut self) -> GainSetting {
        match (
            self.pins.gain_sel.0.get_state(),
            self.pins.gain_sel.1.get_state(),
        ) {
            (PinState::High, PinState::High) => GainSetting::Low,
            (PinState::High, PinState::Low) => GainSetting::Medium,
            (PinState::Low, PinState::High) => GainSetting::High,
            (PinState::Low, PinState::Low) => GainSetting::Max,
        }
    }

    pub fn get_afe_type(&mut self) -> AfeType {
        match self.pins.afe_type_ind.is_high() {
            true => AfeType::Transimpedance,
            false => AfeType::Logarithmic,
        }
    }

    pub fn set_led(&mut self, state: PinState) {
        //let s = PinState::from(state);
        self.pins.led.set_state(state);
    }
}
