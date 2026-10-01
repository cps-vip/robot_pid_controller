/// Encoder reader with delta sampling for Zynq-7000.
///
/// On Zynq-7000, quadrature encoders typically run in FPGA (PL) logic
/// and are read via a 32-bit memory-mapped AXI register (MMIO), or via
/// hardware pulse counters.
pub struct Encoder {
    counter_addr: *const u32,
    last: u32,
}

// Safety: The counter address is a hardware register in the PL/PS address space.
unsafe impl Send for Encoder {}

impl Encoder {
    /// Create a new encoder reader from an MMIO counter register address.
    ///
    /// # Safety
    /// The caller must ensure `counter_addr` is a valid memory-mapped register address.
    pub unsafe fn new(counter_addr: *const u32) -> Self {
        let initial = unsafe { core::ptr::read_volatile(counter_addr) };
        Self {
            counter_addr,
            last: initial,
        }
    }

    /// Read the raw count directly.
    pub fn count(&self) -> u32 {
        unsafe { core::ptr::read_volatile(self.counter_addr) }
    }

    /// Returns delta counts since last call.
    pub fn delta_counts(&mut self) -> i16 {
        let now = self.count();
        let delta = now.wrapping_sub(self.last) as i16;
        self.last = now;
        delta
    }
}
