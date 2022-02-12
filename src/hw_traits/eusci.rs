use super::Steal;
use core::convert::TryInto;
use msp430fr5949 as pac;

pub enum Ucssel {
    Uclk,
    Aclk,
    Smclk,
}

pub struct UcxCtl0 {
    pub ucpen: bool,
    pub ucpar: bool,
    pub ucmsb: bool,
    pub uc7bit: bool,
    pub ucspb: bool,
    pub ucssel: Ucssel,
    pub ucrxeie: bool,
}

pub trait EUsci: Steal {
    fn ctl0_reset(&self);

    // only call while in reset state
    fn brw_settings(&self, ucbr: u16);

    // only call while in reset state
    fn loopback(&self, loopback: bool);

    fn rx_rd(&self) -> u8;

    fn tx_wr(&self, val: u8);

    fn txie_set(&self);
    fn txie_clear(&self);
    fn rxie_set(&self);
    fn rxie_clear(&self);

    fn txifg_rd(&self) -> bool;
    fn rxifg_rd(&self) -> bool;

    fn iv_rd(&self) -> u16;
}

pub trait EUsciUart: EUsci {
    type Statw: UcaxStatw;

    // only call while in reset state
    fn ctl0_settings(&self, reg: UcxCtl0);

    fn mctlw_settings(&self, ucos16: bool, ucbrs: u8, ucbrf: u8);

    fn statw_rd(&self) -> Self::Statw;
}

pub trait UcaxStatw {
    fn ucfe(&self) -> bool;
    fn ucoe(&self) -> bool;
    fn ucpe(&self) -> bool;
    fn ucbrk(&self) -> bool;
    fn ucbusy(&self) -> bool;
}

fn get_bit(val: u8, bit: u8) -> bool {
    let bit = (val >> bit) & 1;
    if bit > 0 {
        true
    } else {
        false
    }
}

fn get_ucssel(isel: Ucssel) -> u8 {
    match isel {
        Ucssel::Uclk => 0,
        Ucssel::Aclk => 1,
        Ucssel::Smclk => 2,
    }
}

macro_rules! eusci_a_impl {
    ($EUsci:ident, $eusci:ident, $ucaxctl0:ident, $ucaxctl1:ident, $ucaxbrw0:ident, $ucaxbrw1:ident, $ucaxmctlw:ident,
     $ucaxstatw:ident, $ucaxrxbuf:ident, $ucaxtxbuf:ident, $ucaxie:ident, $ucaxifg:ident,
     $ucaxiv:ident, $Statw:ty) => {
        impl Steal for pac::$EUsci {
            #[inline(always)]
            unsafe fn steal() -> Self {
                pac::Peripherals::steal().$EUsci
            }
        }

        impl EUsci for pac::$EUsci {
            #[inline(always)]
            fn ctl0_reset(&self) {
                self.$ucaxctl0.write(|w| w.ucswrst().set_bit());
            }

            #[inline(always)]
            fn brw_settings(&self, ucbr: u16) {
                self.$ucaxbrw0
                    .write(|w| unsafe { w.bits(((ucbr & 0xff) & 0xff).try_into().unwrap()) });
                self.$ucaxbrw1
                    .write(|w| unsafe { w.bits(((ucbr >> 8) & 0xff).try_into().unwrap()) });
            }

            #[inline(always)]
            fn loopback(&self, loopback: bool) {
                self.$ucaxstatw.write(|w| w.uclisten().bit(loopback));
            }

            #[inline(always)]
            fn rx_rd(&self) -> u8 {
                let ret: u8 = self.$ucaxrxbuf.read().bits().try_into().unwrap();
                ret
            }

            #[inline(always)]
            fn tx_wr(&self, bits: u8) {
                self.$ucaxtxbuf
                    .write(|w| unsafe { w.bits(bits.try_into().unwrap()) });
            }

            #[inline(always)]
            fn txie_set(&self) {
                self.$ucaxie.write(|w| w.uctxie().set_bit());
            }

            #[inline(always)]
            fn txie_clear(&self) {
                self.$ucaxie.write(|w| w.uctxie().clear_bit());
            }

            #[inline(always)]
            fn rxie_set(&self) {
                self.$ucaxie.write(|w| w.ucrxie().set_bit());
            }

            #[inline(always)]
            fn rxie_clear(&self) {
                self.$ucaxie.write(|w| w.ucrxie().clear_bit());
            }

            #[inline(always)]
            fn txifg_rd(&self) -> bool {
                self.$ucaxifg.read().uctxifg().bit()
            }

            #[inline(always)]
            fn rxifg_rd(&self) -> bool {
                self.$ucaxifg.read().ucrxifg().bit()
            }

            #[inline(always)]
            fn iv_rd(&self) -> u16 {
                self.$ucaxiv.read().bits()
            }
        }

        impl EUsciUart for pac::$EUsci {
            type Statw = $Statw;

            #[inline(always)]
            fn ctl0_settings(&self, reg: UcxCtl0) {
                self.$ucaxctl1.write(|w| {
                    w.ucpen()
                        .bit(reg.ucpen)
                        .ucpar()
                        .bit(reg.ucpar)
                        .ucmsb()
                        .bit(reg.ucmsb)
                        .uc7bit()
                        .bit(reg.uc7bit)
                        .ucspb()
                        .bit(reg.ucspb)
                });
                self.$ucaxctl0.write(|w| {
                    unsafe {
                        // w.ucrxeie()
                        //  .bit(reg.ucrxeie)
                        w.ucsselx()
                            .bits(get_ucssel(reg.ucssel))
                            .ucswrst()
                            .clear_bit()
                    }
                });
                //.ucrxeie()
                //.bit(reg.ucrxeie)
            }

            #[inline(always)]
            fn mctlw_settings(&self, ucos16: bool, ucbrs: u8, ucbrf: u8) {
                self.$ucaxmctlw.write(|w| {
                    w.ucos16()
                        .bit(ucos16)
                        .ucbrs0()
                        .bit(get_bit(ucbrs, 0))
                        .ucbrs1()
                        .bit(get_bit(ucbrs, 1))
                        .ucbrs2()
                        .bit(get_bit(ucbrs, 2))
                        .ucbrs3()
                        .bit(get_bit(ucbrs, 3))
                        .ucbrs4()
                        .bit(get_bit(ucbrs, 4))
                        .ucbrs5()
                        .bit(get_bit(ucbrs, 5))
                        .ucbrs6()
                        .bit(get_bit(ucbrs, 6))
                        .ucbrs7()
                        .bit(get_bit(ucbrs, 7))
                        .ucbrf()
                        .bits(ucbrf)
                });
            }

            #[inline(always)]
            fn statw_rd(&self) -> Self::Statw {
                self.$ucaxstatw.read()
            }
        }

        impl UcaxStatw for $Statw {
            #[inline(always)]
            fn ucfe(&self) -> bool {
                self.ucfe().bit()
            }

            #[inline(always)]
            fn ucoe(&self) -> bool {
                self.ucoe().bit()
            }

            #[inline(always)]
            fn ucpe(&self) -> bool {
                self.ucpe().bit()
            }

            #[inline(always)]
            fn ucbrk(&self) -> bool {
                self.ucbrk().bit()
            }

            #[inline(always)]
            fn ucbusy(&self) -> bool {
                self.ucbusy().bit()
            }
        }
    };
}

eusci_a_impl!(
    USCI_A0_UART_MODE,
    usci_a0_uart_mode,
    uca0ctl0,
    uca0ctl1,
    uca0br0,
    uca0br1,
    uca0mctlw,
    uca0statw,
    uca0rxbuf,
    uca0txbuf,
    uca0ie,
    uca0ifg,
    uca0iv,
    pac::usci_a0_uart_mode::uca0statw::R
);

eusci_a_impl!(
    USCI_A1_UART_MODE,
    usci_a1_uart_mode,
    uca1ctl0,
    uca1ctl1,
    uca1br0,
    uca1br1,
    uca1mctlw,
    uca1statw,
    uca1rxbuf,
    uca1txbuf,
    uca1ie,
    uca1ifg,
    uca1iv,
    pac::usci_a1_uart_mode::uca1statw::R
);
