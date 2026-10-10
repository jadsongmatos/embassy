#![allow(missing_docs)]
//! Time driver for AVR (Timer0 CTC at 1 kHz).
//!
//! Implements [`embassy_time_driver::Driver`] on the ATmega328P, so that
//! `embassy_time::Timer` and friends work.
//!
//! ## Operation
//!
//! Timer0 CTC mode with `OCR0A = 249` and prescaler `/64`. At 16 MHz this
//! generates a 1 kHz (1 ms) tick. The `TIMER0_COMPA` ISR counts the ticks,
//! and when the earliest deadline of the timer queue has passed, it lets the
//! queue wake the expired wakers and records the next deadline.
//!
//! The timer queue is [`embassy_time_queue_utils::Queue`]. Its kind comes from
//! the features of `embassy-time-queue-utils`, which the application selects,
//! for example `generic-queue-8` through `embassy-time`.
//!
//! ## Critical section
//!
//! Shared state is protected by `critical_section::with(...)`. The impl
//! comes from `avr-device` defaults (`restore-state-u8`) — the application
//! MUST NOT declare `critical-section` with `features = ["restore-state-*"]`.

use core::cell::{Cell, RefCell};
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

/// Embassy time driver for AVR (Timer0 CTC 1 kHz).
pub struct EmbassyAvrTimeDriver {
    /// Milliseconds since boot, counted by the Timer0 ISR.
    ticks: Mutex<Cell<u64>>,
    /// Earliest deadline in the queue, or `u64::MAX` when the queue is empty.
    next_expiration: Mutex<Cell<u64>>,
    /// Wakers waiting for a deadline.
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: EmbassyAvrTimeDriver = EmbassyAvrTimeDriver {
    ticks: Mutex::new(Cell::new(0)),
    next_expiration: Mutex::new(Cell::new(u64::MAX)),
    queue: Mutex::new(RefCell::new(Queue::new())),
});

impl Driver for EmbassyAvrTimeDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| self.ticks.borrow(cs).get())
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            // `true` when the new deadline comes before every other one.
            if queue.schedule_wake(at, waker) {
                self.update_next_expiration(cs, &mut queue);
            }
        });
    }
}

impl EmbassyAvrTimeDriver {
    /// Wakes the expired wakers, and records the next deadline for the ISR.
    fn update_next_expiration(&self, cs: CriticalSection<'_>, queue: &mut Queue) {
        let now = self.ticks.borrow(cs).get();
        self.next_expiration.borrow(cs).set(queue.next_expiration(now));
    }

    /// Counts one tick, and handles the deadlines it reaches.
    fn tick(&self) {
        critical_section::with(|cs| {
            let ticks = self.ticks.borrow(cs).get() + 1;
            self.ticks.borrow(cs).set(ticks);
            if ticks >= self.next_expiration.borrow(cs).get() {
                let mut queue = self.queue.borrow(cs).borrow_mut();
                self.update_next_expiration(cs, &mut queue);
            }
        });
    }
}

// ---------------------------------------------------------------------------
// ISR: TIMER0_COMPA (vector 14)
// ---------------------------------------------------------------------------

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    // Clear OCF0A (write-one-to-clear).
    unsafe { write8(TIFR0, 1 << 1) };
    DRIVER.tick();
}

// ---------------------------------------------------------------------------
// Timer0 hardware init
// ---------------------------------------------------------------------------

/// ATmega328P I/O register addresses used by Timer0.
const TCCR0A: u16 = 0x44;
const TCCR0B: u16 = 0x45;
const TCNT0: u16 = 0x46;
const OCR0A: u16 = 0x47;
const TIMSK0: u16 = 0x6E;
const TIFR0: u16 = 0x35;

/// Start Timer0 in CTC mode at 1 kHz and enable the COMPA interrupt.
///
/// Call this once during hardware init, before enabling global interrupts.
/// Also executes `sei` so the tick (and therefore `embassy-time`) runs.
///
/// # Safety
///
/// Touches the Timer0 registers; must run once, before the executor starts.
pub fn init() {
    unsafe {
        // WGM01 = 1, WGM00 = 0 -> CTC mode (clear on compare match A).
        write8(TCCR0A, 1 << 1);
        // CS02:0 = 000 -> timer stopped while configuring.
        write8(TCCR0B, 0);
        write8(TCNT0, 0);
        write8(OCR0A, 249);
        // Clear any pending OCF0A flag, then enable OCIE0A.
        write8(TIFR0, 1 << 1);
        write8(TIMSK0, 1 << 1);
        // CS02:0 = 011 (prescaler /64) -> start timer. 16 MHz / 64 / 250 = 1 kHz.
        write8(TCCR0B, (1 << 1) | (1 << 0));
        // Enable global interrupts so the COMPA ISR fires. Without `sei` the
        // epoch never advances and `embassy_time::Timer` never wakes.
        avr_device::interrupt::enable();
    }
}

/// Write one I/O register byte.
///
/// # Safety
///
/// `addr` must be a writable 8-bit I/O register (here: a Timer0 register).
unsafe fn write8(addr: u16, val: u8) {
    unsafe { core::ptr::write_volatile(addr as *mut u8, val) }
}
