use zynq7000_hal::uart::{Rx, Tx};

/// Minimal bidirectional serial interface for Zynq-7000 UART
pub struct SerialInterface {
    tx: Tx,
    rx: Rx,
}

impl SerialInterface {
    pub fn new(tx: Tx, rx: Rx) -> Self {
        Self { tx, rx }
    }

    pub fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            // Spin until TX FIFO has space
            while self.tx.write_fifo(b).is_err() {
                core::hint::spin_loop();
            }
        }
    }

    /// Read one byte if available.
    pub fn try_read(&mut self) -> Option<u8> {
        match self.rx.read_fifo() {
            Ok(b) => Some(b),
            Err(nb::Error::WouldBlock) => None,
            Err(nb::Error::Other(_)) => None,
        }
    }
}
