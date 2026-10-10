#![allow(missing_docs)]

//! Time driver for AVR (Timer0 CTC at 1 kHz).
//!
//! Implements [`embassy_time_driver::Driver`] on the ATmega328P so that
//! [`embassy_time::Timer`] and friends work alongside the Embassy executor's
//! `integrated-timers` feature.
//!
//! ## Operation
//!
//! Timer0 CTC mode with `OCR0A = 249` and prescaler `/64`. At 16 MHz this
//! generates a 1 kHz (1 ms) tick. The `__vector_14` (TIMER0_COMPA) ISR
//! increments the epoch counter and checks the alarm deadline.
//!
//! The alarm is one-shot: the ISR calls the stored callback (the timer queue,
//! which wakes the expired timers), then sets the deadline to `u64::MAX` so it
//! won't fire again until `set_alarm` is called with a new deadline. The
//! callback itself stays registered.
//!
//! ## Critical section
//!
//! Shared state is protected by `critical_section::with(...)`. The impl
//! comes from `avr-device` defaults (`restore-state-u8`) — the application
//! MUST NOT declare `critical-section` with `features = ["restore-state-*"]`.

use core::cell::{Cell, UnsafeCell};

use embassy_time_driver::{AlarmHandle, Driver};
use portable_atomic::{AtomicU8, Ordering};

/// The single alarm ID (the executor needs only one alarm).
const ALARM_ID: u8 = 0;

/// Milliseconds since boot, incremented by the Timer0 ISR (1 kHz tick).
static EPOCH_MS: critical_section::Mutex<Cell<u64>> = critical_section::Mutex::new(Cell::new(0));

/// Alarm state, accessed under critical_section only (ISR + thread mode).
struct AlarmState {
    timestamp: Cell<u64>,
    callback: Cell<Option<(fn(*mut ()), *mut ())>>,
}

// Safety: function pointers and raw pointers are Send + Sync
unsafe impl Send for AlarmState {}
unsafe impl Sync for AlarmState {}

static ALARM: critical_section::Mutex<UnsafeCell<AlarmState>> =
    critical_section::Mutex::new(UnsafeCell::new(AlarmState {
        timestamp: Cell::new(u64::MAX),
        callback: Cell::new(None),
    }));

static NEXT_ALARM: AtomicU8 = AtomicU8::new(0);

/// Embassy time driver for AVR (Timer0 CTC 1 kHz) — a zero-sized type.
pub struct EmbassyAvrTimeDriver;

embassy_time_driver::time_driver_impl!(static DRIVER: EmbassyAvrTimeDriver = EmbassyAvrTimeDriver);

impl Driver for EmbassyAvrTimeDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| EPOCH_MS.borrow(cs).get())
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        // We only support one alarm
        let mut x = NEXT_ALARM.load(Ordering::Acquire);
        loop {
            if x == 0 {
                match NEXT_ALARM.compare_exchange(x, 1, Ordering::AcqRel, Ordering::Acquire) {
                    Ok(_) => return Some(AlarmHandle::new(ALARM_ID + 1)),
                    Err(actual) => x = actual,
                }
            } else {
                return None;
            }
        }
    }

    fn set_alarm_callback(&self, _alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        critical_section::with(|cs| {
            let alarm = unsafe { &mut *ALARM.borrow(cs).get() };
            alarm.callback.set(Some((callback, ctx)));
        })
    }

    fn set_alarm(&self, _alarm: AlarmHandle, timestamp: u64) -> bool {
        let now = self.now();
        if timestamp <= now {
            return false;
        }

        critical_section::with(|cs| {
            let alarm = unsafe { &mut *ALARM.borrow(cs).get() };
            alarm.timestamp.set(timestamp);
        });
        true
    }
}

// ---------------------------------------------------------------------------
// ISR: TIMER0_COMPA (vector 14)
// ---------------------------------------------------------------------------

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    handle_timer0_compa()
}

fn handle_timer0_compa() {
    // Clear OCF0A (write-one-to-clear).
    unsafe { write8(TIFR0, 1 << 1) };

    let callback = critical_section::with(|cs| {
        let epoch = EPOCH_MS.borrow(cs).get() + 1;
        EPOCH_MS.borrow(cs).set(epoch);

        let alarm = unsafe { &mut *ALARM.borrow(cs).get() };
        if alarm.timestamp.get() <= epoch {
            // One-shot: disarm the alarm, but copy the callback instead of taking it. The driver
            // contract keeps the callback registered after the alarm fires, and the generic timer
            // queue only sets it once.
            alarm.timestamp.set(u64::MAX);
            alarm.callback.get()
        } else {
            None
        }
    });

    if let Some((f, ctx)) = callback {
        f(ctx);
    }
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
