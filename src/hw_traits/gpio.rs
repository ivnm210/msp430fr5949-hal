use super::Steal;
use crate::gpio::{P1, P2, P3, P4};
use msp430fr5949 as pac;

pub trait GpioPeriph: Steal {
    fn pxin_rd(&self) -> u8;

    fn pxout_rd(&self) -> u8;
    fn pxout_wr(&self, bits: u8);
    fn pxout_set(&self, bits: u8);
    fn pxout_clear(&self, bits: u8);
    fn pxout_toggle(&self, bits: u8);

    fn pxdir_rd(&self) -> u8;
    fn pxdir_wr(&self, bits: u8);
    fn pxdir_set(&self, bits: u8);
    fn pxdir_clear(&self, bits: u8);

    fn pxren_rd(&self) -> u8;
    fn pxren_wr(&self, bits: u8);
    fn pxren_set(&self, bits: u8);
    fn pxren_clear(&self, bits: u8);

    fn pxselc_wr(&self, bits: u8);

    fn pxsel0_rd(&self) -> u8;
    fn pxsel0_wr(&self, bits: u8);
    fn pxsel0_set(&self, bits: u8);
    fn pxsel0_clear(&self, bits: u8);

    fn pxsel1_rd(&self) -> u8;
    fn pxsel1_wr(&self, bits: u8);
    fn pxsel1_set(&self, bits: u8);
    fn pxsel1_clear(&self, bits: u8);
}

pub trait GpioPeriph2 {
    fn pxin_rd(&self) -> u8;

    fn pxout_rd(&self) -> u8;
    fn pxout_wr(&self, bits: u8);
    fn pxout_set(&self, bits: u8);
    fn pxout_clear(&self, bits: u8);
    fn pxout_toggle(&self, bits: u8);

    fn pxdir_rd(&self) -> u8;
    fn pxdir_wr(&self, bits: u8);
    fn pxdir_set(&self, bits: u8);
    fn pxdir_clear(&self, bits: u8);

    fn pxren_rd(&self) -> u8;
    fn pxren_wr(&self, bits: u8);
    fn pxren_set(&self, bits: u8);
    fn pxren_clear(&self, bits: u8);

    fn pxselc_wr(&self, bits: u8);

    fn pxsel0_rd(&self) -> u8;
    fn pxsel0_wr(&self, bits: u8);
    fn pxsel0_set(&self, bits: u8);
    fn pxsel0_clear(&self, bits: u8);

    fn pxsel1_rd(&self) -> u8;
    fn pxsel1_wr(&self, bits: u8);
    fn pxsel1_set(&self, bits: u8);
    fn pxsel1_clear(&self, bits: u8);
}

pub trait GpioPeriph16: Steal {
    fn pxin_rd(&self) -> u16;

    fn pxout_rd(&self) -> u16;
    fn pxout_wr(&self, bits: u16);
    fn pxout_set(&self, bits: u16);
    fn pxout_clear(&self, bits: u16);
    fn pxout_toggle(&self, bits: u16);

    fn pxdir_rd(&self) -> u16;
    fn pxdir_wr(&self, bits: u16);
    fn pxdir_set(&self, bits: u16);
    fn pxdir_clear(&self, bits: u16);

    fn pxren_rd(&self) -> u16;
    fn pxren_wr(&self, bits: u16);
    fn pxren_set(&self, bits: u16);
    fn pxren_clear(&self, bits: u16);

    fn pxselc_wr(&self, bits: u16);

    fn pxsel0_rd(&self) -> u16;
    fn pxsel0_wr(&self, bits: u16);
    fn pxsel0_set(&self, bits: u16);
    fn pxsel0_clear(&self, bits: u16);

    fn pxsel1_rd(&self) -> u16;
    fn pxsel1_wr(&self, bits: u16);
    fn pxsel1_set(&self, bits: u16);
    fn pxsel1_clear(&self, bits: u16);
}

pub trait IntrPeriph: GpioPeriph {
    fn pxies_rd(&self) -> u8;
    fn pxies_wr(&self, bits: u8);
    fn pxies_set(&self, bits: u8);
    fn pxies_clear(&self, bits: u8);

    fn pxie_rd(&self) -> u8;
    fn pxie_wr(&self, bits: u8);
    fn pxie_set(&self, bits: u8);
    fn pxie_clear(&self, bits: u8);

    fn pxifg_rd(&self) -> u8;
    fn pxifg_wr(&self, bits: u8);
    fn pxifg_set(&self, bits: u8);
    fn pxifg_clear(&self, bits: u8);

    fn pxiv_rd(&self) -> u16;
}

pub trait IntrPeriph2: GpioPeriph2 {
    fn pxies_rd(&self) -> u8;
    fn pxies_wr(&self, bits: u8);
    fn pxies_set(&self, bits: u8);
    fn pxies_clear(&self, bits: u8);

    fn pxie_rd(&self) -> u8;
    fn pxie_wr(&self, bits: u8);
    fn pxie_set(&self, bits: u8);
    fn pxie_clear(&self, bits: u8);

    fn pxifg_rd(&self) -> u8;
    fn pxifg_wr(&self, bits: u8);
    fn pxifg_set(&self, bits: u8);
    fn pxifg_clear(&self, bits: u8);

    fn pxiv_rd(&self) -> u16;
}

pub trait IntrPeriph16: GpioPeriph16 {
    fn pxies_rd(&self) -> u16;
    fn pxies_wr(&self, bits: u16);
    fn pxies_set(&self, bits: u16);
    fn pxies_clear(&self, bits: u16);

    fn pxie_rd(&self) -> u16;
    fn pxie_wr(&self, bits: u16);
    fn pxie_set(&self, bits: u16);
    fn pxie_clear(&self, bits: u16);

    fn pxifg_rd(&self) -> u16;
    fn pxifg_wr(&self, bits: u16);
    fn pxifg_set(&self, bits: u16);
    fn pxifg_clear(&self, bits: u16);

    fn pxiv_rd(&self) -> u16;
}

macro_rules! reg_methods {
    ($reg:ident, $rd:ident, $wr:ident, $set:ident, $clear:ident) => {
        #[inline(always)]
        fn $rd(&self) -> u8 {
            self.port.$reg.read().bits()
        }

        #[inline(always)]
        fn $wr(&self, bits: u8) {
            self.port.$reg.write(|w| unsafe {w.bits(bits) });
        }

        #[inline(always)]
        fn $set(&self, bits: u8) {
            unsafe { self.port.$reg.modify(|r, w| w.bits(r.bits() | bits)) }
        }

        #[inline(always)]
        fn $clear(&self, bits: u8) {
            unsafe { self.port.$reg.modify(|r, w| w.bits(r.bits() & bits)) }
        }
    }
}

macro_rules! reg_methods16 {
    ($reg:ident, $rd:ident, $wr:ident, $set:ident, $clear:ident) => {
        #[inline(always)]
        fn $rd(&self) -> u16 {
            self.$reg.read().bits()
        }

        #[inline(always)]
        fn $wr(&self, bits: u16) {
            self.$reg.write(|w| unsafe {w.bits(bits) });
        }

        #[inline(always)]
        fn $set(&self, bits: u16) {
            unsafe { self.$reg.modify(|r, w| w.bits(r.bits() | bits)) }
        }

        #[inline(always)]
        fn $clear(&self, bits: u16) {
            unsafe { self.$reg.modify(|r, w| w.bits(r.bits() & bits)) }
        }
    }
}

macro_rules! gpio_impl {
    ($px:ident $pxx:ident: $Px:ident =>
     $pxin:ident, $pxout:ident, $pxdir:ident, $pxren:ident, $pxselc:ident, $pxsel0:ident, $pxsel1:ident
     $(, [$pxies:ident, $pxie:ident, $pxifg:ident, $pxiv:ident])?
    ) => {
        pub mod $px {
            use super::*;

            // pub struct $pxx {
            //     port : pac::$Px
            // }

            // impl Steal for pac::$Px {
            impl Steal for $pxx {
                #[inline(always)]
                unsafe fn steal() -> Self {
                    // pac::Peripherals::steal().$Px;
                    $pxx{
                        port : pac::Peripherals::steal().$Px
                    }
                }
            }

            // impl GpioPeriph for pac::$Px {
            impl GpioPeriph for $pxx {
                #[inline(always)]
                fn pxin_rd(&self) -> u8 {
                    self.port.$pxin.read().bits()
                }

                #[inline(always)]
                fn pxselc_wr(&self, bits: u8) {
                    self.port.$pxselc.write(|w| unsafe { w.bits(bits) })
                }

                #[inline(always)]
                fn pxout_toggle(&self, bits: u8) {
                    // unsafe { self.$pxout.toggle_bits(|w| w.bits(bits)) };
                    unsafe {
                        self.port.$pxout.modify(
                            |r, w|
                            w.bits(!(r.bits() & bits))
                            ,
                        );
                    }
                }

                reg_methods!($pxout, pxout_rd, pxout_wr, pxout_set, pxout_clear);
                reg_methods!($pxdir, pxdir_rd, pxdir_wr, pxdir_set, pxdir_clear);
                reg_methods!($pxren, pxren_rd, pxren_wr, pxren_set, pxren_clear);
                reg_methods!($pxsel0, pxsel0_rd, pxsel0_wr, pxsel0_set, pxsel0_clear);
                reg_methods!($pxsel1, pxsel1_rd, pxsel1_wr, pxsel1_set, pxsel1_clear);
            }

            $(
                // impl IntrPeriph for pac::$Px {
                impl IntrPeriph for $pxx {
                    reg_methods!($pxies, pxies_rd, pxies_wr, pxies_set, pxies_clear);
                    reg_methods!($pxie, pxie_rd, pxie_wr, pxie_set, pxie_clear);
                    reg_methods!($pxifg, pxifg_rd, pxifg_wr, pxifg_set, pxifg_clear);

                    #[inline(always)]
                    fn pxiv_rd(&self) -> u16 {
                        self.port.$pxiv.read().bits()
                    }
                }
            )?
        }
    };
}

macro_rules! gpio_impl16 {
    ($px:ident: $Px:ident =>
     $pxin:ident, $pxout:ident, $pxdir:ident, $pxren:ident, $pxselc:ident, $pxsel0:ident, $pxsel1:ident
     $(, [$pxies:ident, $pxie:ident, $pxifg:ident, $pxiv:ident])?
    ) => {
        mod $px {
            use super::*;

            impl Steal for pac::$Px {
                #[inline(always)]
                unsafe fn steal() -> Self {
                    pac::Peripherals::steal().$Px
                }
            }

            impl GpioPeriph16 for pac::$Px {
                #[inline(always)]
                fn pxin_rd(&self) -> u16 {
                    self.$pxin.read().bits()
                }

                #[inline(always)]
                fn pxselc_wr(&self, bits: u16) {
                    self.$pxselc.write(|w| unsafe { w.bits(bits) })
                }

                #[inline(always)]
                fn pxout_toggle(&self, bits: u16) {
                    // unsafe { self.$pxout.toggle_bits(|w| w.bits(bits)) };
                    unsafe {
                        self.$pxout.modify(
                            |r, w|
                            w.bits(!(r.bits() & bits))
                            ,
                        );
                    }
                }

                reg_methods16!($pxout, pxout_rd, pxout_wr, pxout_set, pxout_clear);
                reg_methods16!($pxdir, pxdir_rd, pxdir_wr, pxdir_set, pxdir_clear);
                reg_methods16!($pxren, pxren_rd, pxren_wr, pxren_set, pxren_clear);
                reg_methods16!($pxsel0, pxsel0_rd, pxsel0_wr, pxsel0_set, pxsel0_clear);
                reg_methods16!($pxsel1, pxsel1_rd, pxsel1_wr, pxsel1_set, pxsel1_clear);
            }

            $(
                impl IntrPeriph16 for pac::$Px {
                    reg_methods16!($pxies, pxies_rd, pxies_wr, pxies_set, pxies_clear);
                    reg_methods16!($pxie, pxie_rd, pxie_wr, pxie_set, pxie_clear);
                    reg_methods16!($pxifg, pxifg_rd, pxifg_wr, pxifg_set, pxifg_clear);

                    #[inline(always)]
                    fn pxiv_rd(&self) -> u16 {
                        self.$pxiv.read().bits()
                    }
                }
            )?
        }
    };
}

gpio_impl!(p1 P1: PORT_1_2 => p1in, p1out, p1dir, p1ren, p1selc, p1sel0, p1sel1, [p1ies, p1ie, p1ifg, p1iv]);
gpio_impl!(p2 P2: PORT_1_2 => p2in, p2out, p2dir, p2ren, p2selc, p2sel0, p2sel1, [p2ies, p2ie, p2ifg, p2iv]);
gpio_impl!(p3 P3: PORT_3_4 => p3in, p3out, p3dir, p3ren, p3selc, p3sel0, p3sel1, [p3ies, p3ie, p3ifg, p3iv]);
gpio_impl!(p4 P4: PORT_3_4 => p4in, p4out, p4dir, p4ren, p4selc, p4sel0, p4sel1, [p4ies, p4ie, p4ifg, p4iv]);
gpio_impl16!(p6: PORT_J => pjin, pjout, pjdir, pjren, pjselc, pjsel0, pjsel1);

//gpio_impl!(p6: P6 => p6in, p6out, p6dir, p6ren, p6selc, p6sel0, p6sel1);
