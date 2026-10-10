//! Byte ring buffer shared between a task and an interrupt handler.

use portable_atomic::{AtomicPtr, AtomicUsize, Ordering};

/// Byte ring buffer over a caller-provided slice, for one writer and one reader.
///
/// The writer only moves `end` and the reader only moves `start`, so each side sees a consistent
/// state when the other one interrupts it. One slot always stays free, to tell a full buffer from
/// an empty one: a buffer of `len` bytes holds `len - 1` bytes.
///
/// Until [`RingBuffer::init`], and after [`RingBuffer::deinit`], the buffer is both empty and
/// full.
pub(crate) struct RingBuffer {
    buf: AtomicPtr<u8>,
    len: AtomicUsize,
    /// Index of the next byte to pop.
    start: AtomicUsize,
    /// Index of the next byte to push.
    end: AtomicUsize,
}

impl RingBuffer {
    /// Returns a buffer without storage.
    pub(crate) const fn new() -> Self {
        Self {
            buf: AtomicPtr::new(core::ptr::null_mut()),
            len: AtomicUsize::new(0),
            start: AtomicUsize::new(0),
            end: AtomicUsize::new(0),
        }
    }

    /// Makes the buffer store its bytes in `buf`, and empties it.
    ///
    /// # Safety
    ///
    /// `buf` must stay valid, and unused elsewhere, until [`RingBuffer::deinit`]. Neither the
    /// writer nor the reader may run meanwhile.
    pub(crate) unsafe fn init(&self, buf: &mut [u8]) {
        self.start.store(0, Ordering::Relaxed);
        self.end.store(0, Ordering::Relaxed);
        self.buf.store(buf.as_mut_ptr(), Ordering::Relaxed);
        self.len.store(buf.len(), Ordering::Relaxed);
    }

    /// Detaches the storage given to [`RingBuffer::init`].
    pub(crate) fn deinit(&self) {
        self.len.store(0, Ordering::Relaxed);
        self.buf.store(core::ptr::null_mut(), Ordering::Relaxed);
        self.start.store(0, Ordering::Relaxed);
        self.end.store(0, Ordering::Relaxed);
    }

    /// Returns whether the buffer holds no bytes.
    pub(crate) fn is_empty(&self) -> bool {
        self.start.load(Ordering::Relaxed) == self.end.load(Ordering::Relaxed)
    }

    /// Appends `byte`, and returns `false` if the buffer is full.
    ///
    /// # Safety
    ///
    /// Only the writer may call this.
    pub(crate) unsafe fn push(&self, byte: u8) -> bool {
        let end = self.end.load(Ordering::Relaxed);
        let next = self.wrap(end + 1);
        if next == self.start.load(Ordering::Relaxed) {
            return false;
        }
        // SAFETY: `end` is below `len`, since `wrap` keeps it there, and `init`'s caller keeps
        // `buf` valid for `len` bytes.
        unsafe { self.buf.load(Ordering::Relaxed).add(end).write(byte) };
        self.end.store(next, Ordering::Relaxed);
        true
    }

    /// Removes and returns the oldest byte.
    ///
    /// # Safety
    ///
    /// Only the reader may call this.
    pub(crate) unsafe fn pop(&self) -> Option<u8> {
        let start = self.start.load(Ordering::Relaxed);
        if start == self.end.load(Ordering::Relaxed) {
            return None;
        }
        // SAFETY: as in `push`, and the writer has stored this byte.
        let byte = unsafe { self.buf.load(Ordering::Relaxed).add(start).read() };
        self.start.store(self.wrap(start + 1), Ordering::Relaxed);
        Some(byte)
    }

    /// Returns `index`, or 0 once `index` reaches the end of the storage.
    fn wrap(&self, index: usize) -> usize {
        if index >= self.len.load(Ordering::Relaxed) {
            0
        } else {
            index
        }
    }
}
