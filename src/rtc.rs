//! Real time counter
//!
//! Can be used as a periodic 16-bit timer
use crate::clock::Smclk;
use core::marker::PhantomData;
use embedded_hal::timer::{Cancel, CountDown, Periodic};
use msp430fr5949 as pac;
use pac::RTC_B_REAL_TIME_CLOCK as RTC;
use void::Void;

mod sealed {
    use super::*;
}
/// Typestate representing the SMCLK clock source for RTC
pub struct Rtc {
/// Typestate representing the SMCLK clock source for RTC
    periph: RTC,
    
}

impl Rtc {
    /// Convert into RTC object with VLOCLK as clock source
    pub fn new(rtc: RTC) -> Self {
        Rtc {
            periph: rtc
        }
    }
}

impl Rtc {
    // Configure the RTC to use SMCLK as clock source. Setting comes in effect the next time RTC
    // is started.

    /// Enable RTC timer interrupts
    #[inline]
    pub fn enable_interrupts(&mut self) {
        self.periph.rtcctl01.
            modify(|r,w| unsafe {w.bits(r.bits())}
            .rtcaie()
            .set_bit());
    }

    /// Disable RTC timer interrupts
    #[inline]
    pub fn disable_interrupts(&mut self) {
        self.periph.rtcctl01.
            modify(|r,w| unsafe {w.bits(r.bits())}
        .rtcaie()
        .clear_bit());
    }

    /// Clear interrupt flag
    #[inline]
    pub fn clear_interrupt(&mut self) {
        self.periph.rtciv.read();
    }

    /// Read current timer count, which goes up from 0 to 2^16-1
    #[inline]
    pub fn get_count_8(&self) -> u8 {
        self.periph.rtcsec.read().bits() as u8 
    }
    /// Read current timer count, which goes up from 0 to 2^16-1
    #[inline]
    pub fn get_count_16(&self) -> u16 {
        (self.periph.rtcsec.read().bits() as u16) +
        ((self.periph.rtcmin.read().bits() as u16) << 8)
    }

    /// Read current timer count, which goes up from 0 to 2^16-1
    #[inline]
    pub fn get_count(&self) -> u32 {
        (self.periph.rtcsec.read().bits() as u32) + 
        ((self.periph.rtcmin.read().bits() as u32) << 8) +
        ((self.periph.rtchour.read().bits() as u32) << 16) +
        ((self.periph.rtcdow.read().bits() as u32) << 24)
    }
}

impl CountDown for Rtc {
    type Time = u16;

    #[inline]
    fn start<T: Into<Self::Time>>(&mut self, count: T) {
        // self.periph
        //     .rtcmod
        //     .write(|w| unsafe { w.bits(count.into()) });
        let min : u8 = count.into() as u8;
        self.periph.rtcctl01.modify(|r, w| {
            unsafe { w.bits(r.bits()) }
                .rtchold()
                .set_bit()
        });
        // self.periph.rtcsec.write(|w| { unsafe{ w.bits(sec)}}); 
        self.periph.rtcmin.write(|w| { unsafe{ w.bits(min)}}); 
        // //  Need to clear interrupt flag from last timer run
        self.periph.rtciv.read();
        self.periph.rtcctl01.modify(|r, w| {
            unsafe { w.bits(r.bits()) }
                .rtchold()
                .clear_bit()
        });
    }

    #[inline]
    fn wait(&mut self) -> nb::Result<(), Void> {
        // if self.periph.rtcctl.read().rtcifg().bit() {
        //     self.periph.rtciv.read();
        //     Ok(())
        // } else {
        //     Err(nb::Error::WouldBlock)
        // }
        if self.periph.rtcctl01.read().rtctevifg().bit() { // rtcifg().bit() {
            self.periph.rtciv.read();
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Cancel for Rtc {
    type Error = Void;

    #[inline]
    fn cancel(&mut self) -> Result<(), Self::Error> {
        // unsafe {
        //     self.periph
        //         .rtcctl
        //         //      Bit pattern is all 0s, so we can use clear instead of modify
        //         .clear_bits(|w| w.rtcss().variant(RTCSS_A::DISABLED))
        // };
        self.periph.rtcctl01.modify(|r, w| {
            unsafe { w.bits(r.bits()) }
                .rtchold()
                .set_bit()
        });
        Ok(())
    }
}

impl Periodic for Rtc {}
