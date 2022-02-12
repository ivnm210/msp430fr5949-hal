use super::Steal;
use msp430fr5949 as pac;

pub enum Tbssel {
    Tbxclk,
    Aclk,
    Smclk,
    Inclk,
}

/// Timer clock divider
pub enum TimerDiv {
    /// No division
    _1,
    /// Divide by 2
    _2,
    /// Divide by 4
    _4,
    /// Divide by 8
    _8,
}

/// Timer expansion clock divider, applied on top of the normal clock divider
pub enum TimerExDiv {
    /// No division
    _1,
    /// Divide by 2
    _2,
    /// Divide by 3
    _3,
    /// Divide by 4
    _4,
    /// Divide by 5
    _5,
    /// Divide by 6
    _6,
    /// Divide by 7
    _7,
    /// Divide by 8
    _8,
}

pub enum Outmod {
    Out,
    Set,
    ToggleReset,
    SetReset,
    Toggle,
    Reset,
    ToggleSet,
    ResetSet,
}

pub enum Cm {
    NoCap,
    RisingEdge,
    FallingEdge,
    BothEdges,
}

pub enum Ccis {
    InputA,
    InputB,
    Gnd,
    Vcc,
}

pub trait TimerB: Steal {
    /// Reset timer countdown
    fn reset(&self);

    /// Set to upmode, reset timer, and clear interrupts
    fn upmode(&self);
    /// Set to continuous mode, reset timer, and clear interrupts
    fn continuous(&self);

    /// Apply clock select settings
    fn config_clock(&self, tbssel: Tbssel, div: TimerDiv);

    /// Check if timer is stopped
    fn is_stopped(&self) -> bool;

    /// Stop timer
    fn stop(&self);

    /// Set expansion register clock divider settings
    fn set_tbidex(&self, tbidex: TimerExDiv);

    fn tbifg_rd(&self) -> bool;
    fn tbifg_clr(&self);

    fn tbie_set(&self);
    fn tbie_clr(&self);

    fn tbxiv_rd(&self) -> u16;
}

pub trait CCRn<C>: Steal {
    fn set_ccrn(&self, count: u16);
    fn get_ccrn(&self) -> u16;

    fn config_outmod(&self, outmod: Outmod);
    fn config_cap_mode(&self, cm: Cm, ccis: Ccis);

    fn ccifg_rd(&self) -> bool;
    fn ccifg_clr(&self);

    fn ccie_set(&self);
    fn ccie_clr(&self);

    fn cov_ccifg_rd(&self) -> (bool, bool);
    fn cov_ccifg_clr(&self);
}

/// Label for capture-compare register 0
pub struct CCR0;
/// Label for capture-compare register 1
pub struct CCR1;
/// Label for capture-compare register 2
pub struct CCR2;
/// Label for capture-compare register 3
pub struct CCR3;
/// Label for capture-compare register 4
pub struct CCR4;
/// Label for capture-compare register 5
pub struct CCR5;
/// Label for capture-compare register 6
pub struct CCR6;

macro_rules! ccrn_impl {
    ($TBx:ident, $CCRn:ident, $tbxcctln:ident, $tbxccrn:ident) => {
        impl CCRn<$CCRn> for pac::$TBx {
            #[inline(always)]
            fn set_ccrn(&self, count: u16) {
                self.$tbxccrn.write(|w| unsafe { w.bits(count) });
            }

            #[inline(always)]
            fn get_ccrn(&self) -> u16 {
                self.$tbxccrn.read().bits()
            }

            #[inline(always)]
            fn config_outmod(&self, outmod: Outmod) {
                self.$tbxcctln.write(|w| w.outmod().bits(outmod as u8));
            }

            #[inline(always)]
            fn config_cap_mode(&self, cm: Cm, ccis: Ccis) {
                self.$tbxcctln.write(|w| {
                    w.cap()
                        // .capture()
                        .set_bit()
                        .scs()
                        // .sync()
                        .set_bit()
                        .cm()
                        .bits(cm as u8)
                        .ccis()
                        .bits(ccis as u8)
                });
            }

            #[inline(always)]
            fn ccifg_rd(&self) -> bool {
                self.$tbxcctln.read().ccifg().bit()
            }

            #[inline(always)]
            fn ccifg_clr(&self) {
                // unsafe { self.$tbxcctln.clear_bits(|w| w.ccifg().clear_bit()) };
                self.$tbxcctln.write(|w| w.ccifg().clear_bit());
            }

            #[inline(always)]
            fn ccie_set(&self) {
                // unsafe { self.$tbxcctln.set_bits(|w| w.ccie().set_bit()) };
                self.$tbxcctln.write(|w| w.ccie().set_bit());
            }

            #[inline(always)]
            fn ccie_clr(&self) {
                // unsafe { self.$tbxcctln.clear_bits(|w| w.ccie().clear_bit()) };
                self.$tbxcctln.write(|w| w.ccie().clear_bit());
            }

            #[inline(always)]
            fn cov_ccifg_rd(&self) -> (bool, bool) {
                let cctl = self.$tbxcctln.read();
                (cctl.cov().bit(), cctl.ccifg().bit())
            }

            #[inline(always)]
            fn cov_ccifg_clr(&self) {
                // self.$tbxcctln
                // .clear_bits(|w| w.ccifg().clear_bit().cov().clear_bit())
                self.$tbxcctln
                    .write(|w| w.ccifg().clear_bit().cov().clear_bit());
            }
        }
    };
}

macro_rules! timera_impl {
    ($TBx:ident, $tbx:ident, $tbxctl:ident, $tbxex:ident, $tbxiv:ident, $([$CCRn:ident, $tbxcctln:ident, $tbxccrn:ident]),*) => {
        impl Steal for pac::$TBx {
            #[inline(always)]
            unsafe fn steal() -> Self {
                pac::Peripherals::steal().$TBx
            }
        }

        impl TimerB for pac::$TBx {
            #[inline(always)]
            fn reset(&self) {
                // unsafe { self.$tbxctl.set_bits(|w| w.tbclr().set_bit()) };
                self.$tbxctl.write(|w|{
                    w.taclr().set_bit()
                });
            }

            #[inline(always)]
            fn upmode(&self) {
                self.$tbxctl.modify(|r, w| {
                    unsafe { w.bits(r.bits()) }
                        .taclr()
                        .set_bit()
                        .taifg()
                        .clear_bit()
                        .mc()
                        // .up()
                        .mc_1()
                });
            }

            #[inline(always)]
            fn continuous(&self) {
                self.$tbxctl.modify(|r, w| {
                    unsafe { w.bits(r.bits()) }
                        .taclr()
                        .set_bit()
                        .taifg()
                        .clear_bit()
                        .mc()
                        // .continuous()
                        .mc_2()
                });
            }

            #[inline(always)]
            fn config_clock(&self, tbssel: Tbssel, div: TimerDiv) {
                self.$tbxctl
                    .write(|w| w.tassel().bits(tbssel as u8).id().bits(div as u8));
            }

            #[inline(always)]
            fn is_stopped(&self) -> bool {
                // self.$tbxctl.read().mc().is_stop()
                self.$tbxctl.read().mc().is_mc_0()
            }

            #[inline(always)]
            fn stop(&self) {
                // unsafe { self.$tbxctl.clear_bits(|w| w.mc().stop()) };
                self.$tbxctl.write(|w|{
                    w.mc().mc_0()
                });
            }

            #[inline(always)]
            fn set_tbidex(&self, tbidex: TimerExDiv) {
                self.$tbxex.write(|w| w.taidex().bits(tbidex as u8));
            }

            #[inline(always)]
            fn tbifg_rd(&self) -> bool {
                self.$tbxctl.read().taifg().bit()
            }

            #[inline(always)]
            fn tbifg_clr(&self) {
                // unsafe { self.$tbxctl.clear_bits(|w| w.tbifg().clear_bit()) };
                self.$tbxctl.write(|w|{
                    w.taifg().clear_bit()
                });
            }

            #[inline(always)]
            fn tbie_set(&self) {
                // unsafe { self.$tbxctl.set_bits(|w| w.tbie().set_bit()) };
                self.$tbxctl.write(|w|{
                    w.taie().set_bit()
                });
            }

            #[inline(always)]
            fn tbie_clr(&self) {
                // unsafe { self.$tbxctl.clear_bits(|w| w.tbie().clear_bit()) };
                self.$tbxctl.write(|w|{
                    w.taie().clear_bit()
                });
            }

            #[inline(always)]
            fn tbxiv_rd(&self) -> u16 {
                self.$tbxiv.read().bits()
            }
        }

        $(ccrn_impl!($TBx, $CCRn, $tbxcctln, $tbxccrn);)*
    };
}

macro_rules! timerb_impl {
    ($TBx:ident, $tbx:ident, $tbxctl:ident, $tbxex:ident, $tbxiv:ident, $([$CCRn:ident, $tbxcctln:ident, $tbxccrn:ident]),*) => {
        impl Steal for pac::$TBx {
            #[inline(always)]
            unsafe fn steal() -> Self {
                pac::Peripherals::steal().$TBx
            }
        }

        impl TimerB for pac::$TBx {
            #[inline(always)]
            fn reset(&self) {
                // unsafe { self.$tbxctl.set_bits(|w| w.tbclr().set_bit()) };
                self.$tbxctl.write(|w|{
                    w.tbclr().set_bit()
                });
            }

            #[inline(always)]
            fn upmode(&self) {
                self.$tbxctl.modify(|r, w| {
                    unsafe { w.bits(r.bits()) }
                        .tbclr()
                        .set_bit()
                        .tbifg()
                        .clear_bit()
                        .mc()
                        // .up()
                        .mc_1()
                });
            }

            #[inline(always)]
            fn continuous(&self) {
                self.$tbxctl.modify(|r, w| {
                    unsafe { w.bits(r.bits()) }
                        .tbclr()
                        .set_bit()
                        .tbifg()
                        .clear_bit()
                        .mc()
                        // .continuous()
                        .mc_2()
                });
            }

            #[inline(always)]
            fn config_clock(&self, tbssel: Tbssel, div: TimerDiv) {
                self.$tbxctl
                    .write(|w| w.tbssel().bits(tbssel as u8).id().bits(div as u8));
            }

            #[inline(always)]
            fn is_stopped(&self) -> bool {
                // self.$tbxctl.read().mc().is_stop()
                self.$tbxctl.read().mc().is_mc_0()
            }

            #[inline(always)]
            fn stop(&self) {
                // unsafe { self.$tbxctl.clear_bits(|w| w.mc().stop()) };
                self.$tbxctl.write(|w|{
                    w.mc().mc_0()
                });
            }

            #[inline(always)]
            fn set_tbidex(&self, tbidex: TimerExDiv) {
                self.$tbxex.write(|w| w.tbidex().bits(tbidex as u8));
            }

            #[inline(always)]
            fn tbifg_rd(&self) -> bool {
                self.$tbxctl.read().tbifg().bit()
            }

            #[inline(always)]
            fn tbifg_clr(&self) {
                // unsafe { self.$tbxctl.clear_bits(|w| w.tbifg().clear_bit()) };
                self.$tbxctl.write(|w|{
                    w.tbifg().clear_bit()
                });
            }

            #[inline(always)]
            fn tbie_set(&self) {
                // unsafe { self.$tbxctl.set_bits(|w| w.tbie().set_bit()) };
                self.$tbxctl.write(|w|{
                    w.tbie().set_bit()
                });
            }

            #[inline(always)]
            fn tbie_clr(&self) {
                // unsafe { self.$tbxctl.clear_bits(|w| w.tbie().clear_bit()) };
                self.$tbxctl.write(|w|{
                    w.tbie().clear_bit()
                });
            }

            #[inline(always)]
            fn tbxiv_rd(&self) -> u16 {
                self.$tbxiv.read().bits()
            }
        }

        $(ccrn_impl!($TBx, $CCRn, $tbxcctln, $tbxccrn);)*
    };
}

timera_impl!(
    TIMER_0_A3,
    timer_0_a3,
    ta0ctl,
    ta0ex0,
    ta0iv,
    [CCR0, ta0cctl0, ta0ccr0],
    [CCR1, ta0cctl1, ta0ccr1],
    [CCR2, ta0cctl2, ta0ccr2]
);

timera_impl!(
    TIMER_1_A3,
    timer_1_a3,
    ta1ctl,
    ta1ex0,
    ta1iv,
    [CCR0, ta1cctl0, ta1ccr0],
    [CCR1, ta1cctl1, ta1ccr1],
    [CCR2, ta1cctl2, ta1ccr2]
);

timera_impl!(
    TIMER_2_A2,
    timer_2_a2,
    ta2ctl,
    ta2ex0,
    ta2iv,
    [CCR0, ta2cctl0, ta2ccr0],
    [CCR1, ta2cctl1, ta2ccr1]
);

timera_impl!(
    TIMER_3_A2,
    timer_3_a2,
    ta3ctl,
    ta3ex0,
    ta3iv,
    [CCR0, ta3cctl0, ta3ccr0],
    [CCR1, ta3cctl1, ta3ccr1]
);

/*
 * timerb_impl!(
    TIMER_0_B7,
    timer_0_b7,
    tb0ctl,
    tb0ex0,
    tb0iv,
    [CCR0, tb0cctl0, tb0ccr0],
    [CCR1, tb0cctl1, tb0ccr1],
    [CCR2, tb0cctl2, tb0ccr2]
);
*/

timerb_impl!(
    TIMER_0_B7,
    timer_0_b7,
    tb0ctl,
    tb0ex0,
    tb0iv,
    [CCR0, tb0cctl0, tb0ccr0],
    [CCR1, tb0cctl1, tb0ccr1],
    [CCR2, tb0cctl2, tb0ccr2],
    [CCR3, tb0cctl3, tb0ccr3],
    [CCR4, tb0cctl4, tb0ccr4],
    [CCR5, tb0cctl5, tb0ccr5],
    [CCR6, tb0cctl6, tb0ccr6]
);
