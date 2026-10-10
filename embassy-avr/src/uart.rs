//! Interrupt-driven, buffered USART0 driver for the ATmega328P.
//!
//! [`BufferedUart`] implements the [`embedded_io_async`] traits on top of two buffers that the
//! caller provides. Interrupt handlers move the data, so neither direction busy-waits:
//!
//! * `USART_RX` moves each received byte into the receive buffer;
//! * `USART_UDRE` feeds the transmit buffer to the transmitter;
//! * `USART_TX` reports that the transmitter sent everything, for
//!   [`flush`](embedded_io_async::Write::flush).
//!
//! The crate defines these handlers with the `rt` feature, which this module requires.
//!
//! On the Arduino Uno, USART0 is the USB serial port: the board's ATmega16U2 bridges `PD0` (RX)
//! and `PD1` (TX) to USB.
//!
//! ## Baud rate
//!
//! The divider follows the Arduino core, for the 16 MHz clock of the supported boards: `U2X0`
//! doubles the rate, except at 57600 baud and at rates too slow for it. The Uno's bootloader and
//! USB-serial firmware make the same exception, so both ends run at 58824 baud (+2.1 %) when
//! 57600 baud is requested.

use core::cell::Cell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::{Poll, Waker};

use avr_device::atmega328p::usart0::RegisterBlock;
use avr_device::atmega328p::USART0;
use portable_atomic::{AtomicBool, AtomicU8, Ordering};

use crate::mcu::port::{PD0, PD1};
use crate::ring_buffer::RingBuffer;

/// Clock frequency of the supported boards, in hertz.
const F_CPU: u32 = 16_000_000;
/// Largest divider, since `UBRR0` has 12 bits.
const UBRR_MAX: u32 = 4095;

/// `UCSR0A` flag: the received byte has no valid stop bit.
const FE0: u8 = 1 << 4;
/// `UCSR0A` flag: a received byte was lost.
const DOR0: u8 = 1 << 3;
/// `UCSR0A` flag: the received byte has a parity error.
const UPE0: u8 = 1 << 2;

/// Bytes received by `USART_RX`, waiting for a read.
static RX_BUF: RingBuffer = RingBuffer::new();
/// Bytes waiting for `USART_UDRE` to send them.
static TX_BUF: RingBuffer = RingBuffer::new();
/// Task waiting for received bytes.
static RX_WAKER: WakerSlot = WakerSlot::new();
/// Task waiting for room in the transmit buffer, or for the end of the transmission.
static TX_WAKER: WakerSlot = WakerSlot::new();
/// Reception errors since the last read, as `UCSR0A` flags.
static RX_ERRORS: AtomicU8 = AtomicU8::new(0);
/// Whether the transmitter has bytes to send. Writes set it, and `USART_TX` clears it.
static TX_BUSY: AtomicBool = AtomicBool::new(false);

/// UART configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct Config {
    /// Baud rate.
    pub baudrate: u32,
    /// Number of data bits.
    pub data_bits: DataBits,
    /// Parity bit.
    pub parity: Parity,
    /// Number of stop bits.
    pub stop_bits: StopBits,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115_200,
            data_bits: DataBits::DataBits8,
            parity: Parity::ParityNone,
            stop_bits: StopBits::STOP1,
        }
    }
}

/// Number of data bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataBits {
    /// 5 data bits.
    DataBits5,
    /// 6 data bits.
    DataBits6,
    /// 7 data bits.
    DataBits7,
    /// 8 data bits.
    DataBits8,
}

/// Parity bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    /// No parity bit.
    ParityNone,
    /// Even parity bit.
    ParityEven,
    /// Odd parity bit.
    ParityOdd,
}

/// Number of stop bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    /// 1 stop bit.
    STOP1,
    /// 2 stop bits.
    STOP2,
}

/// Configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConfigError {
    /// The clock cannot generate the baud rate.
    BaudrateNotSupported,
    /// A buffer is shorter than 2 bytes.
    BufferTooSmall,
}

/// Reception error.
///
/// A read returns the errors that occurred since the previous read before any more data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// Received bytes were lost: the receive buffer was full, or the handler ran too late.
    Overrun,
    /// A received byte had a parity error, and was dropped.
    Parity,
    /// A received byte had no valid stop bit, and was dropped.
    Framing,
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// Interrupt-driven, buffered USART0 driver.
///
/// Only one instance may exist at a time.
pub struct BufferedUart<'d> {
    usart: USART0,
    _buffers: PhantomData<&'d mut [u8]>,
}

impl<'d> BufferedUart<'d> {
    /// Configures USART0, and enables its receiver and its transmitter.
    ///
    /// The driver buffers received bytes in `rx_buf` and bytes to send in `tx_buf`. Each buffer
    /// holds one byte less than its length. The pins only prove ownership: the USART takes them
    /// over.
    ///
    /// # Errors
    ///
    /// Returns an error if the clock cannot generate the baud rate, or if a buffer is shorter
    /// than 2 bytes.
    pub fn new(
        usart: USART0,
        _rx: PD0,
        _tx: PD1,
        rx_buf: &'d mut [u8],
        tx_buf: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        let (u2x, ubrr) = baud_divider(config.baudrate).ok_or(ConfigError::BaudrateNotSupported)?;
        if rx_buf.len() < 2 || tx_buf.len() < 2 {
            return Err(ConfigError::BufferTooSmall);
        }

        // Turns the USART off, so that no handler runs while the buffers change.
        usart.ucsr0b().reset();
        // SAFETY: the handlers are off, and the buffers outlive `Self`, whose drop detaches them.
        unsafe {
            RX_BUF.init(rx_buf);
            TX_BUF.init(tx_buf);
        }
        RX_ERRORS.store(0, Ordering::Relaxed);
        TX_BUSY.store(false, Ordering::Relaxed);

        usart.ubrr0().write(|w| w.set(ubrr));
        // Writing 1 to `TXC0` clears a transmit-complete flag left from earlier.
        usart.ucsr0a().write(|w| w.u2x0().bit(u2x).txc0().set_bit());
        usart.ucsr0c().write(|w| {
            let w = w.umsel0().usart_async();
            let w = match config.data_bits {
                DataBits::DataBits5 => w.ucsz0().chr5(),
                DataBits::DataBits6 => w.ucsz0().chr6(),
                DataBits::DataBits7 => w.ucsz0().chr7(),
                DataBits::DataBits8 => w.ucsz0().chr8(),
            };
            let w = match config.parity {
                Parity::ParityNone => w.upm0().disabled(),
                Parity::ParityEven => w.upm0().parity_even(),
                Parity::ParityOdd => w.upm0().parity_odd(),
            };
            match config.stop_bits {
                StopBits::STOP1 => w.usbs0().stop1(),
                StopBits::STOP2 => w.usbs0().stop2(),
            }
        });
        // Writes enable the transmit interrupts.
        usart
            .ucsr0b()
            .write(|w| w.rxen0().set_bit().txen0().set_bit().rxcie0().set_bit());

        Ok(Self {
            usart,
            _buffers: PhantomData,
        })
    }
}

impl Drop for BufferedUart<'_> {
    fn drop(&mut self) {
        // Turns the USART off before the buffers go away. Bytes not sent yet are lost.
        self.usart.ucsr0b().reset();
        RX_BUF.deinit();
        TX_BUF.deinit();
        TX_BUSY.store(false, Ordering::Relaxed);
    }
}

impl embedded_io::ErrorType for BufferedUart<'_> {
    type Error = Error;
}

impl embedded_io_async::Read for BufferedUart<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        poll_fn(|cx| {
            // Registers first, so that a byte received during the checks wakes the task.
            RX_WAKER.register(cx.waker());
            if let Some(error) = take_error() {
                return Poll::Ready(Err(error));
            }
            let mut len = 0;
            for slot in buf.iter_mut() {
                // SAFETY: reads, which need the only `BufferedUart`, are the only readers of the
                // receive buffer.
                let Some(byte) = (unsafe { RX_BUF.pop() }) else {
                    break;
                };
                *slot = byte;
                len += 1;
            }
            if len == 0 {
                Poll::Pending
            } else {
                Poll::Ready(Ok(len))
            }
        })
        .await
    }
}

impl embedded_io::ReadReady for BufferedUart<'_> {
    fn read_ready(&mut self) -> Result<bool, Error> {
        Ok(!RX_BUF.is_empty() || RX_ERRORS.load(Ordering::Relaxed) != 0)
    }
}

impl embedded_io_async::Write for BufferedUart<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        poll_fn(|cx| {
            TX_WAKER.register(cx.waker());
            let mut len = 0;
            for &byte in buf {
                // SAFETY: writes, which need the only `BufferedUart`, are the only writers of the
                // transmit buffer.
                if !unsafe { TX_BUF.push(byte) } {
                    break;
                }
                len += 1;
            }
            if len == 0 {
                // `USART_UDRE` wakes the task once it has sent the whole buffer.
                return Poll::Pending;
            }
            critical_section::with(|_| {
                TX_BUSY.store(true, Ordering::Relaxed);
                regs().ucsr0b().modify(|_, w| w.udrie0().set_bit().txcie0().set_bit());
            });
            Poll::Ready(Ok(len))
        })
        .await
    }

    async fn flush(&mut self) -> Result<(), Error> {
        poll_fn(|cx| {
            TX_WAKER.register(cx.waker());
            if TX_BUSY.load(Ordering::Relaxed) {
                Poll::Pending
            } else {
                Poll::Ready(Ok(()))
            }
        })
        .await
    }
}

/// Returns `U2X0` and `UBRR0` for `baudrate`, or [`None`] if the clock cannot generate it.
fn baud_divider(baudrate: u32) -> Option<(bool, u16)> {
    if baudrate == 0 {
        return None;
    }
    let doubled = (F_CPU / 4 / baudrate).checked_sub(1)? / 2;
    let (u2x, ubrr) = if baudrate != 57_600 && doubled <= UBRR_MAX {
        (true, doubled)
    } else {
        (false, (F_CPU / 8 / baudrate).checked_sub(1)? / 2)
    };
    // The divider only approximates most rates, within 5 % at the usual ones.
    let samples_per_bit = if u2x { 8 } else { 16 };
    let actual = F_CPU / samples_per_bit / (ubrr + 1);
    if ubrr > UBRR_MAX || actual.abs_diff(baudrate) > baudrate / 20 {
        return None;
    }
    Some((u2x, u16::try_from(ubrr).ok()?))
}

/// Returns and clears the oldest kind of reception error.
fn take_error() -> Option<Error> {
    let errors = RX_ERRORS.swap(0, Ordering::Relaxed);
    if errors & DOR0 != 0 {
        Some(Error::Overrun)
    } else if errors & UPE0 != 0 {
        Some(Error::Parity)
    } else if errors & FE0 != 0 {
        Some(Error::Framing)
    } else {
        None
    }
}

/// Returns the USART0 registers.
fn regs() -> &'static RegisterBlock {
    // SAFETY: the registers are always mapped at this address. Only a `BufferedUart`, which
    // owns USART0, and its interrupt handlers use them.
    unsafe { &*USART0::ptr() }
}

/// Handles `USART_RX`: moves the received byte into the receive buffer.
fn on_receive() {
    let regs = regs();
    // The flags describe the byte in `UDR0`, and reading `UDR0` clears them and `RXC0`.
    let mut errors = regs.ucsr0a().read().bits() & (FE0 | DOR0 | UPE0);
    let byte = regs.udr0().read().bits();
    // SAFETY: this handler is the only writer of the receive buffer.
    if errors & (FE0 | UPE0) == 0 && !unsafe { RX_BUF.push(byte) } {
        errors |= DOR0;
    }
    if errors != 0 {
        RX_ERRORS.fetch_or(errors, Ordering::Relaxed);
    }
    RX_WAKER.wake();
}

/// Handles `USART_UDRE`: sends the next byte of the transmit buffer.
fn on_data_register_empty() {
    let regs = regs();
    // SAFETY: this handler is the only reader of the transmit buffer.
    if let Some(byte) = unsafe { TX_BUF.pop() } {
        regs.udr0().write(|w| w.set(byte));
    }
    if TX_BUF.is_empty() {
        // Stops this interrupt, and lets a blocked write fill the buffer again.
        regs.ucsr0b().modify(|_, w| w.udrie0().clear_bit());
        TX_WAKER.wake();
    }
}

/// Handles `USART_TX`: the transmitter sent its last byte, and entering the handler cleared
/// `TXC0`.
fn on_transmit_complete() {
    // A write may have added bytes that `USART_UDRE` has not taken yet; their transmission will
    // complete later.
    if TX_BUF.is_empty() {
        TX_BUSY.store(false, Ordering::Relaxed);
        regs().ucsr0b().modify(|_, w| w.txcie0().clear_bit());
        TX_WAKER.wake();
    }
}

/// Holds the USART0 interrupt handlers.
#[allow(missing_docs, reason = "`avr_device::interrupt` generates undocumented handlers")]
mod handlers {
    use super::{on_data_register_empty, on_receive, on_transmit_complete};

    #[avr_device::interrupt(atmega328p)]
    fn USART_RX() {
        on_receive();
    }

    #[avr_device::interrupt(atmega328p)]
    fn USART_UDRE() {
        on_data_register_empty();
    }

    #[avr_device::interrupt(atmega328p)]
    fn USART_TX() {
        on_transmit_complete();
    }
}

/// Waker of a task waiting for a USART0 event, shared with an interrupt handler.
struct WakerSlot(critical_section::Mutex<Cell<Option<Waker>>>);

impl WakerSlot {
    const fn new() -> Self {
        Self(critical_section::Mutex::new(Cell::new(None)))
    }

    /// Makes the next [`WakerSlot::wake`] wake `waker`.
    fn register(&self, waker: &Waker) {
        critical_section::with(|cs| {
            let slot = self.0.borrow(cs);
            let registered = slot.take();
            slot.set(match registered {
                Some(registered) if registered.will_wake(waker) => Some(registered),
                _ => Some(waker.clone()),
            });
        });
    }

    /// Wakes the registered waker, if any, and forgets it.
    fn wake(&self) {
        if let Some(waker) = critical_section::with(|cs| self.0.borrow(cs).take()) {
            waker.wake();
        }
    }
}
