//! Serial Peripheral Interface (SPI) bus

//use crate::clock::{Aclk, Clock, Smclk};
use crate::gpio::{Alternate2, Pin,  Pin2, Pin6, Pin7}; //, P1, P4};
// use crate::hw_traits::gpio::{p1::P1, p2::P2, p3::P3, p4::P4};
use crate::gpio::{P1, P2};
//use embedded_hal::serial::{Read, Write};
use embedded_hal::spi::{Mode, FullDuplex};
use msp430fr5949::{USCI_B0_SPI_MODE};
//use msp430fr5949 as pac;
use nb;
//use nb::block;
// use embedded_hal::prelude::_embedded_hal_blocking_serial_Write;
// use tm4c123x::{SSI0, SSI1, SSI2, SSI3};

/// SPI error
#[derive(Debug)]
pub enum Error {
    #[doc(hidden)]
    _Extensible,
}

// FIXME these should be "closed" traits
/// SCK pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SckPin<SPI> {}

/// MISO pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MisoPin<SPI> {}

/// MOSI pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MosiPin<SPI> {}

// SSI0
unsafe impl<DIR> SckPin<USCI_B0_SPI_MODE> for Pin<P2, Pin2, Alternate2<DIR>> {}
unsafe impl<DIR> MisoPin<USCI_B0_SPI_MODE> for Pin<P1, Pin7, Alternate2<DIR>> {}
unsafe impl<DIR> MosiPin<USCI_B0_SPI_MODE> for Pin<P1, Pin6, Alternate2<DIR>> {}


/// SPI peripheral operating in full duplex master mode
// pub struct Spi<SPI, PINS, USCI:SerialUsci> {
//     spi: SPI,
//     pins: PINS,
//     tx: Tx<USCI>,
// }
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

macro_rules! busy_wait {
    ($spi:expr, $flag:ident, $op:ident) => {
        loop {
            let sr = $spi.ucb0ifg_spi.read();
            if sr.$flag().$op() {
                break;
            }
        }
    };
}

macro_rules! hal {
    //($($SPIX:ident: ($spiX:ident, $Usci:ident),)+) => {
    ($($SPIX:ident: ($spiX:ident),)+) => {
        $(
            //impl<SCK, MISO, MOSI, USCI: SerialUsci> Spi<$SPIX, (SCK, MISO, MOSI), USCI> {
            impl<SCK, MISO, MOSI> Spi<$SPIX, (SCK, MISO, MOSI)> {
                /// Configures the SPI peripheral to operate in full duplex master mode
                // pub fn $spiX<F>(
                pub fn $spiX (
                    spi: $SPIX,
                    pins: (SCK, MISO, MOSI),
                    _mode: Mode,
                    //tx: Tx<USCI>,
                    // freq: F,
                    // clocks: &Clocks,
                    // pc: &sysctl::PowerControl,
                ) -> Self
                where
                    // F: Into<Hertz>,
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    spi.ucb0ctl0_spi.write(|w| unsafe { w.bits(1) } );
                    spi.ucb0ctl0_spi.modify(|r,w| unsafe { w.bits(r.bits() | 0x80) } );
                    spi.ucb0ctl1_spi.modify(|_,w| unsafe { w.bits(0xA9) } );
                    spi.ucb0br0_spi.write(|w| unsafe { w.bits(2) } );
                    spi.ucb0ctl0_spi.modify(|r,w| unsafe { w.bits(r.bits() & 0xFE) } );

                    //Spi { spi, pins, tx }
                    Spi { spi, pins }
                }

                /// Releases the SPI peripheral and associated pins
                // pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI), Tx<USCI>) {
                //     (self.spi, self.pins, self.tx)
                // }
                pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI)) {
                    (self.spi, self.pins)
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
            impl<PINS> FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    //self.tx.bwrite_all(b"spi read\n\r");
                    // Receive FIFO Not Empty
                    if self.spi.ucb0ifg_spi.read().ucrxifg().bit_is_clear() {
                         //self.tx.bwrite_all(b"err\n\r");
                         Err(nb::Error::WouldBlock)
                    } else {
                        let r = self.spi.ucb0rxbuf_spi.read().bits() as u8;
                         Ok(r)
                    }
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    // Transmit FIFO Not Full
                    //self.tx.bwrite_all(b"spi send\n\r");
                    if self.spi.ucb0ifg_spi.read().uctxifg().bit_is_clear() {
                         //self.tx.bwrite_all(b"err\n\r");
                         Err(nb::Error::WouldBlock)
                    } else {
                        self.spi.ucb0txbuf_spi.write(|w| unsafe {
                             w.bits(byte.into())
                        });
                        //self.tx.bwrite_all(b"wait\n\r");
                        busy_wait!(self.spi, uctxifg, bit_is_set);
                        Ok(())
                    }
                }
            }

            //impl<PINS, USCI: SerialUsci> embedded_hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS, USCI> {}
            impl<PINS> embedded_hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

            //impl<PINS, USCI: SerialUsci> embedded_hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS, USCI> {}
            impl<PINS> embedded_hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
        )+
    }
}

hal! {
    //USCI_B0_SPI_MODE: (spi0, USCI_A1_UART_MODE),
    USCI_B0_SPI_MODE: (spi0),
}
