//! Storage (EEPROM) driver for AVR.
//!
//! This module re-exports the `avr-hal` EEPROM driver, which reads, writes and erases single
//! bytes of the EEPROM, synchronously: the hardware takes 1.8 ms to write or erase a byte, or
//! 3.4 ms to do both at once, and the driver polls for it.
//!
//! The adapter that presents the EEPROM as the async NOR flash of
//! [`embedded-storage-async`](https://docs.rs/embedded-storage-async), which is what Ariel OS
//! stores key-value pairs on, lives in `ariel-os-avr` instead, next to the rest of the Ariel
//! OS glue.

#[cfg(any(feature = "device-atmega328p", feature = "arduino-uno"))]
pub use atmega_hal::eeprom::Eeprom;
