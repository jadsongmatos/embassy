//! Minimal USART0 driver for AVR (ATmega328P), via direct register access.
//!
//! On the Arduino Uno, USB serial is plain USART0, bridged to USB by the board's ATmega16U2; no
//! USB stack is involved. Transmission polls the data-register-empty flag, so it never depends on
//! `embassy-time`.
//!
//! With the `rt` feature, reception is interrupt-driven: the `USART_RX` handler moves every
//! received byte into a small ring buffer, so bytes are not lost while the application busy-waits
//! elsewhere (ADC conversions, transmissions). Without it, [`Serial::try_read`] polls the
//! receiver, which only holds two bytes.
//!
//! ## Registers (ATmega328P datasheet, chapter 20)
//!
//! * `UBRR0` (`0xC4`/`0xC5`): baud-rate divider. 57600 baud at 16 MHz uses `U2X0 = 0` and
//!   `UBRR = 16` (58824 baud, +2.1 %), like the Arduino core: the Uno's USB-serial firmware
//!   special-cases 57600 baud the same way, so both ends match.
//! * `UCSR0A` (`0xC0`): `U2X0` doubles the rate; `RXC0`/`UDRE0` are the
//!   receive-complete / data-register-empty flags.
//! * `UCSR0B` (`0xC1`): `RXCIE0` enables the receive interrupt, `RXEN0`/`TXEN0` enable the
//!   receiver/transmitter.
//! * `UCSR0C` (`0xC2`): `UCSZ01:0 = 11` selects 8 data bits, 1 stop, no parity.
//! * `UDR0` (`0xC6`): shared TX/RX data register.

#[cfg(feature = "rt")]
use portable_atomic::{AtomicU8, Ordering};

/// I/O-space addresses of the USART0 registers used here.
const UCSR0A: u16 = 0xC0;
const UCSR0B: u16 = 0xC1;
const UCSR0C: u16 = 0xC2;
const UBRR0L: u16 = 0xC4;
const UBRR0H: u16 = 0xC5;
const UDR0: u16 = 0xC6;

/// Bit positions used below.
#[cfg(not(feature = "rt"))]
const RXC0: u8 = 1 << 7;
const UDRE0: u8 = 1 << 5;
#[cfg(feature = "rt")]
const RXCIE0: u8 = 1 << 7;
const RXEN0: u8 = 1 << 4;
const TXEN0: u8 = 1 << 3;
/// `UCSZ01:0 = 11` (8 data bits) in `UCSR0C`.
const UCSZ0_8BIT: u8 = 0b11 << 1;
/// `UBRR` for 57600 baud at 16 MHz with `U2X0 = 0` (Arduino core and Uno USB-serial firmware).
const UBRR0_57600: u16 = 16;

/// `UCSR0B` value: receiver and transmitter on, plus the receive interrupt when it has a handler.
#[cfg(feature = "rt")]
const UCSR0B_ENABLE: u8 = RXCIE0 | RXEN0 | TXEN0;
#[cfg(not(feature = "rt"))]
const UCSR0B_ENABLE: u8 = RXEN0 | TXEN0;

/// Capacity of the receive ring buffer, a power of two. One slot always stays empty, to tell a
/// full buffer from an empty one.
#[cfg(feature = "rt")]
const RX_BUF_LEN: u8 = 64;

/// Bytes stored by the `USART_RX` handler, waiting for [`Serial::try_read`].
#[cfg(feature = "rt")]
static RX_BUF: [AtomicU8; RX_BUF_LEN as usize] = [const { AtomicU8::new(0) }; RX_BUF_LEN as usize];
/// Index of the next slot to write. Only the `USART_RX` handler modifies it.
#[cfg(feature = "rt")]
static RX_HEAD: AtomicU8 = AtomicU8::new(0);
/// Index of the next slot to read. Only [`Serial::try_read`] modifies it.
#[cfg(feature = "rt")]
static RX_TAIL: AtomicU8 = AtomicU8::new(0);

/// Returns the ring buffer index that follows `index`.
#[cfg(feature = "rt")]
fn next_index(index: u8) -> u8 {
    (index + 1) & (RX_BUF_LEN - 1)
}

/// Holds the `USART_RX` handler, which moves each received byte into the ring buffer.
#[cfg(feature = "rt")]
#[allow(missing_docs, reason = "`avr_device::interrupt` generates an undocumented handler")]
mod rx_interrupt {
    use portable_atomic::Ordering;

    use super::{next_index, read8, RX_BUF, RX_HEAD, RX_TAIL, UDR0};

    #[avr_device::interrupt(atmega328p)]
    fn USART_RX() {
        // Reading `UDR0` clears `RXC0`. The byte must be read even when the buffer is full,
        // otherwise the interrupt fires again right away.
        let byte = unsafe { read8(UDR0) };
        let head = RX_HEAD.load(Ordering::Relaxed);
        let next = next_index(head);
        // When the buffer is full, the byte is dropped.
        if next != RX_TAIL.load(Ordering::Relaxed) {
            RX_BUF[usize::from(head)].store(byte, Ordering::Relaxed);
            RX_HEAD.store(next, Ordering::Relaxed);
        }
    }
}

/// USART0 driver (57600 baud, 8N1).
///
/// Created once via [`Serial::init`]; afterwards [`Serial::try_read`] returns received bytes
/// without blocking and [`Serial::write_byte`] busy-waits on TX. `Send` so it can live in a task.
pub struct Serial {
    _private: (),
}

// SAFETY: all state lives in the USART0 hardware registers and in the receive buffer, which only
// the `USART_RX` handler (writer) and `try_read` (reader) access, through atomics.
unsafe impl Send for Serial {}

impl Serial {
    /// Configures USART0 for 57600 baud 8N1 and enables RX + TX.
    ///
    /// With the `rt` feature, this also enables the receive interrupt; received bytes are only
    /// buffered once global interrupts are enabled.
    #[must_use]
    pub fn init() -> Self {
        unsafe {
            write8(UBRR0H, (UBRR0_57600 >> 8) as u8);
            write8(UBRR0L, UBRR0_57600 as u8);
            // Clears U2X0: the Arduino core does the same for 57600 baud at 16 MHz, to match the
            // bootloader and the USB-serial firmware.
            write8(UCSR0A, 0);
            write8(UCSR0C, UCSZ0_8BIT);
            write8(UCSR0B, UCSR0B_ENABLE);
        }
        Self { _private: () }
    }

    /// Returns the next received byte, or [`None`] when none is waiting.
    #[cfg(feature = "rt")]
    pub fn try_read(&mut self) -> Option<u8> {
        let tail = RX_TAIL.load(Ordering::Relaxed);
        if tail == RX_HEAD.load(Ordering::Relaxed) {
            return None;
        }
        let byte = RX_BUF[usize::from(tail)].load(Ordering::Relaxed);
        RX_TAIL.store(next_index(tail), Ordering::Relaxed);
        Some(byte)
    }

    /// Returns the next received byte, or [`None`] when none is waiting.
    #[cfg(not(feature = "rt"))]
    pub fn try_read(&mut self) -> Option<u8> {
        unsafe {
            if read8(UCSR0A) & RXC0 == 0 {
                return None;
            }
            Some(read8(UDR0))
        }
    }

    /// Transmits one byte, spinning until the data register is empty.
    pub fn write_byte(&mut self, byte: u8) {
        unsafe {
            while read8(UCSR0A) & UDRE0 == 0 {
                core::hint::spin_loop();
            }
            write8(UDR0, byte);
        }
    }

    /// Transmits `buf` byte by byte.
    pub fn write_all(&mut self, buf: &[u8]) {
        for &b in buf {
            self.write_byte(b);
        }
    }
}

/// Write one USART0 register byte.
///
/// # Safety
///
/// `addr` must be a writable 8-bit USART0 register.
unsafe fn write8(addr: u16, val: u8) {
    unsafe { core::ptr::write_volatile(addr as *mut u8, val) }
}

/// Read one USART0 register byte.
///
/// # Safety
///
/// `addr` must be a readable 8-bit USART0 register.
unsafe fn read8(addr: u16) -> u8 {
    unsafe { core::ptr::read_volatile(addr as *const u8) }
}
