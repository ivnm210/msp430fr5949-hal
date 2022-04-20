//! Serial Peripheral Interface (SPI) bus

use crate::gpio::{Alternate2, Pin, Pin0, Pin1, Pin2, Pin5, Pin6, Pin7};
use crate::gpio::{P1, P2};
use embedded_hal::spi::{FullDuplex, Mode};
use msp430fr5949::{USCI_A0_SPI_MODE, USCI_B0_SPI_MODE};
use nb;

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
unsafe impl<DIR> SckPin<USCI_A0_SPI_MODE> for Pin<P1, Pin5, Alternate2<DIR>> {}
unsafe impl<DIR> MisoPin<USCI_A0_SPI_MODE> for Pin<P2, Pin1, Alternate2<DIR>> {}
unsafe impl<DIR> MosiPin<USCI_A0_SPI_MODE> for Pin<P2, Pin0, Alternate2<DIR>> {}

/// SPI peripheral operating in full duplex master mode
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

macro_rules! busy_waita {
    ($spi:expr, $flag:ident, $op:ident) => {
        loop {
            let sr = $spi.uca0ifg_spi.read();
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

                    Spi { spi, pins }
                }

                /// Releases the SPI peripheral and associated pins
                pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI)) {
                    (self.spi, self.pins)
                }

            }

            impl<PINS> FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // Receive FIFO Not Empty
                    if self.spi.ucb0ifg_spi.read().ucrxifg().bit_is_clear() {
                         Err(nb::Error::WouldBlock)
                    } else {
                        let r = self.spi.ucb0rxbuf_spi.read().bits() as u8;
                         Ok(r)
                    }
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    // Transmit FIFO Not Full
                    if self.spi.ucb0ifg_spi.read().uctxifg().bit_is_clear() {
                         Err(nb::Error::WouldBlock)
                    } else {
                        self.spi.ucb0txbuf_spi.write(|w| unsafe {
                             w.bits(byte.into())
                        });
                        busy_wait!(self.spi, uctxifg, bit_is_set);
                        Ok(())
                    }
                }
            }

            impl<PINS> embedded_hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

            impl<PINS> embedded_hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
        )+
    }
}

macro_rules! hala {
    ($($SPIX:ident: ($spiX:ident),)+) => {
        $(
            impl<SCK, MISO, MOSI> Spi<$SPIX, (SCK, MISO, MOSI)> {
                /// Configures the SPI peripheral to operate in full duplex master mode
                pub fn $spiX (
                    spi: $SPIX,
                    pins: (SCK, MISO, MOSI),
                    _mode: Mode,
                ) -> Self
                where
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    spi.uca0ctl0_spi.write(|w| unsafe { w.bits(1) } );
                    spi.uca0ctl0_spi.modify(|r,w| unsafe { w.bits(r.bits() | 0x80) } );
                    spi.uca0ctl1_spi.modify(|_,w| unsafe { w.bits(0xA9) } );
                    spi.uca0br0_spi.write(|w| unsafe { w.bits(2) } );
                    spi.uca0ctl0_spi.modify(|r,w| unsafe { w.bits(r.bits() & 0xFE) } );

                    Spi { spi, pins }
                }

                /// Releases the SPI peripheral and associated pins
                pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI)) {
                    (self.spi, self.pins)
                }
            }

            impl<PINS> FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // Receive FIFO Not Empty
                    if self.spi.uca0ifg_spi.read().ucrxifg().bit_is_clear() {
                         Err(nb::Error::WouldBlock)
                    } else {
                        let r = self.spi.uca0rxbuf_spi.read().bits() as u8;
                         Ok(r)
                    }
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    // Transmit FIFO Not Full
                    if self.spi.uca0ifg_spi.read().uctxifg().bit_is_clear() {
                         Err(nb::Error::WouldBlock)
                    } else {
                        self.spi.uca0txbuf_spi.write(|w| unsafe {
                             w.bits(byte.into())
                        });
                        busy_waita!(self.spi, uctxifg, bit_is_set);
                        Ok(())
                    }
                }
            }

            impl<PINS> embedded_hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

            impl<PINS> embedded_hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
        )+
    }
}

hal! {
    USCI_B0_SPI_MODE: (spi0),
}

hala! {
    USCI_A0_SPI_MODE: (spi1),
}
