//! ADC access for AVR (ATmega328P), via direct register access.
//!
//! The Ariel OS HAL does not expose an ADC API on AVR yet, and the `avr-hal`
//! [`Adc`](crate::mcu::Adc) driver needs ownership of the `ADC` peripheral
//! plus an arduino-hal `Pins` struct. Both conflict with the pins handed out
//! by `ariel-os-avr::init()`.
//!
//! This module instead drives the ADC registers directly, so a thermistor on
//! `A0` (`PC0` / ADC0) can be sampled with nothing but the Ariel OS pin
//! marker. It deliberately covers only what the heater loop needs: AVcc
//! reference, channel 0, 10-bit blocking reads.
//!
//! ## Registers (ATmega328P datasheet, chapter 23)
//!
//! * `ADMUX` (`0x7C`): reference selection (`REFS1:0 = 01` for AVcc) and
//!   channel selection (`MUX3:0 = 0000` for ADC0).
//! * `ADCSRA` (`0x7A`): `ADEN` enables the ADC, `ADPS2:0 = 111` selects the
//!   `/128` prescaler (16 MHz / 128 = 125 kHz, inside the 50–200 kHz band),
//!   `ADSC` starts a conversion, `ADIF` signals completion.
//! * `ADCL` (`0x78`) / `ADCH` (`0x79`): result registers; `ADCL` must be read
//!   first, which locks both until `ADCH` is read.
//! * `DIDR0` (`0x7E`): `ADC0D` disables the digital input buffer of `ADC0`,
//!   which only adds noise and power draw on an analog pin. It does not affect
//!   the pin's pull-up.

#![allow(clippy::missing_panics_doc, reason = "single supported MCU")]

/// I/O-space addresses of the ADC registers used here (datasheet §23.9).
const ADCL: u16 = 0x78;
const ADCH: u16 = 0x79;
const ADCSRA: u16 = 0x7A;
const ADMUX: u16 = 0x7C;
const DIDR0: u16 = 0x7E;

/// Bit positions used below.
const ADEN: u8 = 1 << 7;
const ADSC: u8 = 1 << 6;
const ADIF: u8 = 1 << 4;
const ADPS_128: u8 = 0b111;
const REFS_AVCC: u8 = 0b01 << 6;
const ADC0D: u8 = 1 << 0;

/// ADC driver bound to channel 0 (`A0`).
///
/// Created once via [`Adc::init`]; afterwards [`Adc::read_blocking`] samples
/// the pin. `Send` so it can live in a task.
pub struct Adc {
    _private: (),
}

// SAFETY: all state lives in the ADC hardware registers; every method runs
// to completion on one core with interrupts only masking the shared
// `embassy-time` epoch, never ADC state.
unsafe impl Send for Adc {}

impl Adc {
    /// Enables the ADC with AVcc reference, `/128` prescaler, channel 0.
    ///
    /// The caller keeps ownership of the `PC0` marker, and configures it (for
    /// example with a pull-up); this only touches the ADC registers, never the
    /// port registers.
    #[must_use]
    pub fn init() -> Self {
        unsafe {
            // AVcc reference, right-adjusted result, channel ADC0.
            write8(ADMUX, REFS_AVCC);
            write8(DIDR0, read8(DIDR0) | ADC0D);
            // Enable + /128 prescaler, no conversion running yet.
            write8(ADCSRA, ADEN | ADPS_128);
        }
        Self { _private: () }
    }

    /// Takes one 10-bit sample from channel 0.
    ///
    /// Starts the conversion and spins on `ADIF`. At 125 kHz ADC clock one
    /// conversion takes 13 cycles (~104 µs), so the busy-wait is short.
    pub fn read_blocking(&mut self) -> u16 {
        unsafe {
            // Start conversion, keeping enable + prescaler bits.
            write8(ADCSRA, read8(ADCSRA) | ADSC);
            // Wait for completion.
            while read8(ADCSRA) & ADIF == 0 {
                core::hint::spin_loop();
            }
            // Clear the flag (write-one-to-clear) without stopping the ADC.
            write8(ADCSRA, read8(ADCSRA) | ADIF);
            // ADCL first: locks both registers until ADCH is read.
            let low = u16::from(read8(ADCL));
            let high = u16::from(read8(ADCH));
            (high << 8) | low
        }
    }
}

/// Write one ADC register byte.
///
/// # Safety
///
/// `addr` must be a writable 8-bit ADC register.
unsafe fn write8(addr: u16, val: u8) {
    unsafe { core::ptr::write_volatile(addr as *mut u8, val) }
}

/// Read one ADC register byte.
///
/// # Safety
///
/// `addr` must be a readable 8-bit ADC register.
unsafe fn read8(addr: u16) -> u8 {
    unsafe { core::ptr::read_volatile(addr as *const u8) }
}
