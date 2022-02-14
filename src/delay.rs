//! Delay abstractions for delay loops.
//!
use msp430::asm;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};

/// Delay struct for MSP430 delay loops
pub struct Delay {}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new() -> Self {
        Delay {}
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) {}
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(ms as u32);
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(ms as u32);
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        // Tricky to get this to not overflow
        let mut i = us;
        asm::nop();
        loop {
            asm::nop();
            i -= 1;
            if i == 0 {
                break;
            }
        }
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(us as u32)
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(us as u32)
    }
}
