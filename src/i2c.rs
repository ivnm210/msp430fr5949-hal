//! I2c bus

use crate::gpio::{Alternate2, Pin, Pin2, Pin6, Pin7}; //, P1, P4};
use crate::gpio::{P1, P2};
use embedded_hal::blocking::i2c::Write;
use msp430fr5949::USCI_B0_I2C_MODE;
use nb;

/// I2C error
#[derive(Debug)]
pub enum Error {
    #[doc(hidden)]
    _Extensible,
}

// FIXME these should be "closed" traits
/// SCK pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SckPin<I2C> {}

/// SDA pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SdaPin<I2C> {}

// I2C0
unsafe impl<DIR> SckPin<USCI_B0_I2C_MODE> for Pin<P2, Pin2, Alternate2<DIR>> {}
unsafe impl<DIR> SdaPin<USCI_B0_I2C_MODE> for Pin<P1, Pin6, Alternate2<DIR>> {}

pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident, $op:ident) => {
        loop {
            let sr = $i2c.ucb0ifg_i2c.read();
            if sr.$flag().$op() {
                break;
            }
        }
    };
}

macro_rules! hal2 {
    ($($I2CX:ident: ($i2cX:ident),)+) => {
        $(
            impl<SCK, SDA> I2c<$I2CX, (SCK, SDA)> {
                pub fn $i2cX (
                    i2c: $I2CX,
                    pins: (SCK, SDA),
                ) -> Self
                where
                    // F: Into<Hertz>,
                    SCK: SckPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    i2c.ucb0ctl0_i2c.write(|w| unsafe { w.bits(1) } );
                    i2c.ucb0ctl0_i2c.modify(|r,w| unsafe { w.bits(r.bits() | 0x80) } );
                    i2c.ucb0ctl1_i2c.modify(|_,w| unsafe { w.bits(0xA9) } );
                    i2c.ucb0br0_i2c.write(|w| unsafe { w.bits(2) } );
                    i2c.ucb0ctl0_i2c.modify(|r,w| unsafe { w.bits(r.bits() & 0xFE) } );

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCK, SDA)) {
                    (self.i2c, self.pins)
                }

                // / Change the clock frequency of the SPI device.
                // pub fn reclock<F>(&mut self, freq: F, clocks: &Clocks) where F: Into<Hertz> {
                    // Disable peripheral
                    // self.spi.cr1.modify(|_, w| w.sse().clear_bit());

                    // let scr: u8;
                    // let mut cpsr = 2u32;
                    // let target_bitrate : u32 = clocks.sysclk.0 / freq.into().0;

                    // Find solution for
                    // SSInClk = SysClk / (CPSDVSR * (1 + SCR))
                    // with:
                    //   CPSDVSR in [2,254]
                    //   SCR in [0,255]

                    // loop {
                    //     let scr32 = (target_bitrate / cpsr) - 1;
                    //     if scr32 < 255 {
                    //         scr = scr32 as u8;
                    //         break;
                    //     }
                    //     cpsr += 2;
                    //     assert!(cpsr <= 254);
                    // }

                    // let cpsr = cpsr as u8;

                    // self.spi.cpsr.write(|w| unsafe { w.cpsdvsr().bits(cpsr) });
                    // self.spi.cr0.modify(|_,w| unsafe { w.scr().bits(scr) });

                    // Enable peripheral again
                    // self.spi.cr1.modify(|_, w| w.sse().set_bit());
                // }
            }

            impl<PINS> Write<A: AddressMode = SevenBitAddress> for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, address: A, bytes: &[u8]) -> nb::Result<(), Error> {
                    // Transmit FIFO Not Full
                    if self.i2c.ucb0ifg_i2c.read().uctxifg().bit_is_clear() {
                         Err(nb::Error::WouldBlock)
                    } else {
                        self.i2c.ucb0txbuf_i2c.write(|w| unsafe {
                             w.bits(byte.into())
                        });
                        busy_wait!(self.i2c, uctxifg, bit_is_set);
                        Ok(())
                    }
                }
            }

        )+
    }
}

hal2! {
    USCI_B0_I2C_MODE: (i2c0),
}
