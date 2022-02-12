//! Serial Peripheral Interface (SPI) bus

use crate::gpio::{Alternate2, Pin, Pin2, Pin6, Pin7}; //, P1, P4};
                                                      // use crate::hw_traits::gpio::{p1::P1, p2::P2, p3::P3, p4::P4};
use crate::gpio::{P1, P2};
use embedded_hal::blocking::i2c::{Write};
use msp430fr5949::USCI_B0_I2C_MODE;
use nb;

/// SPI error
#[derive(Debug)]
pub enum Error {
    #[doc(hidden)]
    _Extensible,
}

// FIXME these should be "closed" traits
/// SCK pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SckPin<I2C> {}

/// MISO pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MisoPin<I2C> {}

/// MOSI pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MosiPin<I2C> {}

// I2C0
unsafe impl<DIR> SckPin<USCI_B0_I2C_MODE> for Pin<P2, Pin2, Alternate2<DIR>> {}
unsafe impl<DIR> MisoPin<USCI_B0_I2C_MODE> for Pin<P1, Pin7, Alternate2<DIR>> {}
unsafe impl<DIR> MosiPin<USCI_B0_I2C_MODE> for Pin<P1, Pin6, Alternate2<DIR>> {}

/// SPI peripheral operating in full duplex master mode
// pub struct Spi<SPI, PINS, USCI:SerialUsci> {
//     spi: SPI,
//     pins: PINS,
//     tx: Tx<USCI>,
// }
pub struct I2c<I2C, PINS> {
    spi: I2C,
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

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident),)+) => {
        $(
            //impl<SCK, MISO, MOSI, USCI: SerialUsci> Spi<$SPIX, (SCK, MISO, MOSI), USCI> {
            impl<SCK, MISO, MOSI> I2c<$I2CX, (SCK, MISO, MOSI)> {
                /// Configures the SPI peripheral to operate in full duplex master mode
                // pub fn $spiX<F>(
                pub fn $i2cX (
                    i2c: $I2CX,
                    pins: (SCK, MISO, MOSI),
                ) -> Self
                where
                    // F: Into<Hertz>,
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    i2c.ucb0ctl0_i2c.write(|w| unsafe { w.bits(1) } );
                    i2c.ucb0ctl0_i2c.modify(|r,w| unsafe { w.bits(r.bits() | 0x80) } );
                    i2c.ucb0ctl1_i2c.modify(|_,w| unsafe { w.bits(0xA9) } );
                    i2c.ucb0br0_i2c.write(|w| unsafe { w.bits(2) } );
                    i2c.ucb0ctl0_i2c.modify(|r,w| unsafe { w.bits(r.bits() & 0xFE) } );

                    //Spi { spi, pins, tx }
                    I2c { i2c, pins }
                }

                /// Releases the SPI peripheral and associated pins
                // pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI), Tx<USCI>) {
                //     (self.spi, self.pins, self.tx)
                // }
                pub fn free(self) -> ($I2CX, (SCK, MISO, MOSI)) {
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

            //impl<PINS, USCI: SerialUsci> FullDuplex<u8> for Spi<$SPIX, PINS, USCI> {
            impl<PINS> Write<A: AddressMode = SevenBitAddress> for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, address: A, bytes: &[u8]) -> nb::Result<(), Error> {
                    // Transmit FIFO Not Full
                    //self.tx.bwrite_all(b"spi send\n\r");
                    if self.i2c.ucb0ifg_i2c.read().uctxifg().bit_is_clear() {
                         //self.tx.bwrite_all(b"err\n\r");
                         Err(nb::Error::WouldBlock)
                    } else {
                        self.i2c.ucb0txbuf_i2c.write(|w| unsafe {
                             w.bits(byte.into())
                        });
                        //self.tx.bwrite_all(b"wait\n\r");
                        busy_wait!(self.i2c, uctxifg, bit_is_set);
                        Ok(())
                    }
                }
            }

            //impl<PINS, USCI: SerialUsci> embedded_hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS, USCI> {}
            impl<PINS> embedded_hal::blocking::spi::transfer::Default<u8> for I2c<$I2CX, PINS> {}

            //impl<PINS, USCI: SerialUsci> embedded_hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS, USCI> {}
            impl<PINS> embedded_hal::blocking::spi::write::Default<u8> for I2c<$I2CX, PINS> {}
        )+
    }
}

hal! {
    //USCI_B0_SPI_MODE: (spi0, USCI_A1_UART_MODE),
    USCI_B0_I2C_MODE: (i2c0),
}
