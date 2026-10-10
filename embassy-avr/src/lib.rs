//! Embassy Hardware Abstraction Layer (HAL) for AVR microcontrollers.
//!
//! `embassy-avr` wraps the [`avr-hal`](https://github.com/ariel-os/avr-hal) family
//! of crates (avr-hal-generic, atmega-hal, arduino-hal) to provide Embassy-style
//! async APIs for AVR microcontrollers.
//!
//! ## Supported devices
//!
//! - ATmega328P (Arduino Uno, Nano, Pro)
//!
//! ## Features
//!
//! - `device-atmega328p` — enable ATmega328P support via `atmega-hal`
//! - `arduino-uno` — enable Arduino Uno board crate (`arduino-hal`)
//! - `time-driver-tc0` — enable the [`time_driver::EmbassyAvrTimeDriver`]
//!   backed by Timer0 in CTC mode at ~1 kHz, registered as the global
//!   `embassy-time` driver.
//! - `rt` — enable `avr-device`'s runtime, and define the USART0 interrupt
//!   handlers that `uart` needs.
//!
//! ## Limitations
//!
//! The ATmega328P has only 2 KB of SRAM and 32 KB of flash. As a result:
//!
//! - No `executor-interrupt` (only supported on Cortex-M); use `executor-thread`.
//! - No USB, BLE, networking, or hardware RNG.
//! - I2C/SPI drivers are blocking wrappers around the `nb` (non-blocking)
//!   `avr-hal` drivers — they spin until the hardware completes the transaction.
//!   This is acceptable for slow peripherals but means the executor does not
//!   yield during I2C/SPI transactions.
//! - `uart` drives USART0 only, with interrupts in both directions; it needs
//!   the `rt` feature.
//! - [`storage`] only re-exports the byte-wise, blocking EEPROM driver of
//!   `avr-hal`; the async NOR flash adapter over it lives in `ariel-os-avr`.
//! - [`adc`] exposes the ATmega328P ADC through direct register access so a
//!   thermistor can be sampled without taking `arduino-hal` peripherals.

#![no_std]
#![cfg_attr(target_arch = "avr", feature(asm_experimental_arch))]
#![cfg_attr(target_arch = "avr", feature(abi_avr_interrupt))]
#![warn(missing_docs)]

pub mod adc;
pub mod gpio;
pub mod i2c;
#[cfg(all(feature = "rt", feature = "device-atmega328p"))]
mod ring_buffer;
pub mod spi;
pub mod storage;
#[cfg(feature = "time-driver-tc0")]
pub mod time_driver;
#[cfg(all(feature = "rt", feature = "device-atmega328p"))]
pub mod uart;

// Re-export at chip-level PAC and HAL for downstream `ariel-os-avr` consumers.
#[cfg(feature = "arduino-uno")]
pub use arduino_hal as board;
#[cfg(feature = "device-atmega328p")]
pub use atmega_hal as mcu;
pub use avr_device;
pub use avr_hal_generic as hal;
