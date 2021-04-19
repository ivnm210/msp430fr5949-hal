//! Clock system for configuration of MCLK, SMCLK, and ACLK.
//!
//! Once configuration is complete, `Aclk` and `Smclk` clock objects are returned. The clock
//! objects are used to set the clock sources on other peripherals.
//! Configuration of MCLK and SMCLK *must* occur, though SMCLK can be disabled. In that case, only
//! `Aclk` is returned.
//!
//! DCO with FLL is supported on MCLK for select frequencies. Supporting arbitrary frequencies on
//! the DCO requires complex calibration routines not supported by the HAL.

use crate::fram::{Fram, WaitStates};
use msp430fr5949 as pac;
// use pac::cs::csctl1::DCORSEL_W;
use pac::cs::csctl2::{SELA_A, SELM_A, SELS_A};
pub use pac::cs::csctl2::{SELS_A as SmclkSel};
pub use pac::cs::csctl3::{DIVM_A as MclkDiv, DIVS_A as SmclkDiv};

/// REFOCLK frequency
pub const REFOCLK: u16 = 32768;
/// VLOCLK frequency
pub const VLOCLK: u16 = 10000;

enum MclkSel {
    Refoclk,
    Vloclk,
    Dcoclk(DcoclkFreqSel),
}

impl MclkSel {
    #[inline]
    fn freq(&self) -> u32 {
        match self {
            MclkSel::Vloclk => VLOCLK as u32,
            MclkSel::Refoclk => REFOCLK as u32,
            MclkSel::Dcoclk(sel) => sel.freq(),
        }
    }

    #[inline(always)]
    fn selm(&self) -> SELM_A {
        match self {
            MclkSel::Vloclk => SELM_A::SELM_1, //SELM_A::VLOCLK,
            MclkSel::Refoclk => SELM_A::SELM_0, //SELM_A::REFOCLK,
            MclkSel::Dcoclk(_) => SELM_A::SELM_3, //SELM_A::DCOCLKDIV,
        }
    }
}

#[derive(Clone, Copy)]
enum AclkSel {
    Vloclk,
    Refoclk,
}

impl AclkSel {
    #[inline(always)]
    fn sela(self) -> SELA_A {
        match self {
            AclkSel::Vloclk => SELA_A::SELA_1, //SELA_A::VLOCLK,
            AclkSel::Refoclk => SELA_A::SELA_0, //SELA_A::REFOCLK,
        }
    }

    #[inline(always)]
    fn freq(self) -> u16 {
        match self {
            AclkSel::Vloclk => VLOCLK,
            AclkSel::Refoclk => REFOCLK,
        }
    }
}

#[derive(Clone, Copy)]
enum SclkSel {
    Vloclk,
    Refoclk,
    Dcoclk(DcoclkFreqSel),
}

impl SclkSel {
    #[inline(always)]
    fn sels(self) -> SELS_A {
        match self {
            SclkSel::Dcoclk(_) => SELS_A::SELS_3, //SELA_A::VLOCLK,
            SclkSel::Vloclk => SELS_A::SELS_1, //SELA_A::VLOCLK,
            SclkSel::Refoclk => SELS_A::SELS_0, //SELA_A::REFOCLK,
        }
    }

    #[inline(always)]
    fn freq(self) -> u32 {
        match self {
            SclkSel::Dcoclk(sel) => sel.freq(),
            SclkSel::Vloclk => VLOCLK as u32,
            SclkSel::Refoclk => REFOCLK as u32,
        }
    }
}

/// Selectable DCOCLK frequencies when using factory trim settings.
/// Actual frequencies may be slightly higher.
#[derive(Clone, Copy)]
pub enum DcoclkFreqSel {
    /// 1 MHz
    _1MHz,
    /// 2 MHz
    _2MHz,
    /// 4 MHz
    _4MHz,
    /// 8 MHz
    _7MHz,
    /// 12 MHz
    _8MHz,
    /// 16 MHz
    _16MHz,
    /// 20 MHz
    _21MHz,
    /// 24 MHz
    _24MHz,
}

impl DcoclkFreqSel {
    #[inline(always)]
    fn dcorsel(self) -> bool {
        match self {
            DcoclkFreqSel::_1MHz => false, //0,
            DcoclkFreqSel::_2MHz => false, //0,
            DcoclkFreqSel::_4MHz => false, //0,
            DcoclkFreqSel::_7MHz => false, //0,
            DcoclkFreqSel::_8MHz => true, //1,
            DcoclkFreqSel::_16MHz => true, //1,
            DcoclkFreqSel::_21MHz => true, //1,
            DcoclkFreqSel::_24MHz => true, //1,
        }
    }

    #[inline(always)]
    fn multiplier(self) -> u16 {
        match self {
            DcoclkFreqSel::_1MHz => 32,
            DcoclkFreqSel::_2MHz => 61,
            DcoclkFreqSel::_4MHz => 122,
            DcoclkFreqSel::_7MHz => 245,
            DcoclkFreqSel::_8MHz => 366,
            DcoclkFreqSel::_16MHz => 490,
            DcoclkFreqSel::_21MHz => 610,
            DcoclkFreqSel::_24MHz => 732,
        }
    }
    
    #[inline(always)]
    fn dcofsel(self) -> u8 {
        match self {
            DcoclkFreqSel::_1MHz => 0x00,
            DcoclkFreqSel::_2MHz => 0x01,
            DcoclkFreqSel::_4MHz => 0x03,
            DcoclkFreqSel::_7MHz => 0x05,
            DcoclkFreqSel::_8MHz => 0x03,
            DcoclkFreqSel::_16MHz => 0x04,
            DcoclkFreqSel::_21MHz => 0x05,
            DcoclkFreqSel::_24MHz => 0x06,
        }
    }

    /// Numerical frequency
    #[inline]
    pub fn freq(self) -> u32 {
        (self.multiplier() as u32) * (REFOCLK as u32)
    }
}

/// Typestate for `ClockConfig` that represents unconfigured clocks
pub struct NoClockDefined;
/// Typestate for `ClockConfig` that represents a configured MCLK
pub struct MclkDefined(MclkSel);
/// Typestate for `ClockConfig` that represents a configured SMCLK
pub struct SmclkDefined(SmclkDiv);
/// Typestate for `ClockConfig` that represents disabled SMCLK
pub struct SmclkDisabled;

// Using SmclkState as a trait bound outside the HAL will never be useful, since we only configure
// the clock once, so just keep it hidden
#[doc(hidden)]
pub trait SmclkState {
    fn div(&self) -> Option<SmclkDiv>;
}

impl SmclkState for SmclkDefined {
    #[inline(always)]
    fn div(&self) -> Option<SmclkDiv> {
        Some(self.0)
    }
}

impl SmclkState for SmclkDisabled {
    #[inline(always)]
    fn div(&self) -> Option<SmclkDiv> {
        None
    }
}

/// Builder object that configures system clocks
///
/// Can only commit configurations to hardware if both MCLK and SMCLK settings have been
/// configured. ACLK configurations are optional, with its default source being REFOCLK.
pub struct ClockConfig<MCLK, SMCLK> {
    periph: pac::CS,
    mclk: MCLK,
    mclk_div: MclkDiv,
    aclk_sel: AclkSel,
    sclk_sel: SclkSel,
    smclk: SMCLK,
}

macro_rules! make_clkconf {
    ($conf:expr, $mclk:expr, $smclk:expr) => {
        ClockConfig {
            periph: $conf.periph,
            mclk: $mclk,
            mclk_div: $conf.mclk_div,
            aclk_sel: $conf.aclk_sel,
            sclk_sel: $conf.sclk_sel,
            smclk: $smclk,
        }
    };
}

impl ClockConfig<NoClockDefined, NoClockDefined> {
    /// Converts CS into a fresh, unconfigured clock builder object
    pub fn new(cs: pac::CS) -> Self {
        ClockConfig {
            periph: cs,
            smclk: NoClockDefined,
            mclk: NoClockDefined,
            mclk_div: MclkDiv::DIVM_1,
            aclk_sel: AclkSel::Refoclk,
            sclk_sel: SclkSel::Refoclk,
        }
    }
}

impl<MCLK, SMCLK> ClockConfig<MCLK, SMCLK> {
    /// Select REFOCLK for ACLK
    #[inline]
    pub fn aclk_refoclk(mut self) -> Self {
        self.aclk_sel = AclkSel::Refoclk;
        self
    }

    /// Select VLOCLK for ACLK
    #[inline]
    pub fn aclk_vloclk(mut self) -> Self {
        self.aclk_sel = AclkSel::Vloclk;
        self
    }

    /// Select DCOCLK for Sclk
    #[inline]
    pub fn sclk_dcoclk(mut self,
        target_freq: DcoclkFreqSel
    ) -> Self {
        self.sclk_sel = SclkSel::Dcoclk(target_freq);
        self
    }

    /// Select REFOCLK for Sclk
    #[inline]
    pub fn sclk_refoclk(mut self) -> Self {
        self.sclk_sel = SclkSel::Refoclk;
        self
    }

    /// Select VLOCLK for ACLK
    #[inline]
    pub fn sclk_vloclk(mut self) -> Self {
        self.sclk_sel = SclkSel::Vloclk;
        self
    }

    /// Select REFOCLK for MCLK and set the MCLK divider. Frequency is `10000 / mclk_div` Hz.
    #[inline]
    pub fn mclk_refoclk(self, mclk_div: MclkDiv) -> ClockConfig<MclkDefined, SMCLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Refoclk), self.smclk)
        }
    }

    /// Select VLOCLK for MCLK and set the MCLK divider. Frequency is `32768 / mclk_div` Hz.
    #[inline]
    pub fn mclk_vcoclk(self, mclk_div: MclkDiv) -> ClockConfig<MclkDefined, SMCLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Vloclk), self.smclk)
        }
    }

    /// Select DCOCLK for MCLK with FLL for stabilization. Frequency is `target_freq / mclk_div` Hz.
    /// This setting selects the default factory trim for DCO trimming and performs no extra
    /// calibration, so only a select few frequency targets can be selected.
    #[inline]
    pub fn mclk_dcoclk(
        self,
        target_freq: DcoclkFreqSel,
        mclk_div: MclkDiv,
    ) -> ClockConfig<MclkDefined, SMCLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Dcoclk(target_freq)), self.smclk)
        }
    }

    /// Enable SMCLK and set SMCLK divider, which divides the MCLK frequency
    #[inline]
    pub fn smclk_on(self,
                    div: SmclkDiv,
                    target_freq: DcoclkFreqSel) -> ClockConfig<MCLK, SmclkDefined> {
        ClockConfig {
        sclk_sel: SclkSel::Dcoclk(target_freq),
        ..make_clkconf!(self, self.mclk, SmclkDefined(div))

        }
    }

    /// Disable SMCLK
    #[inline]
    pub fn smclk_off(self) -> ClockConfig<MCLK, SmclkDisabled> {
        make_clkconf!(self, self.mclk, SmclkDisabled)
    }
}

#[inline(always)]
fn fll_off() {
    const FLAG: u8 = 1 << 6;
    unsafe { llvm_asm!("bis.b $0, SR" :: "i"(FLAG) : "memory" : "volatile") };
}

#[inline(always)]
fn fll_on() {
    const FLAG: u8 = 1 << 6;
    unsafe { llvm_asm!("bic.b $0, SR" :: "i"(FLAG) : "memory" : "volatile") };
}
   
/*
    CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
    // Set user's frequency selection for DCO
    CSCTL1 = DCOFSEL_4 | DCORSEL;             // Set DCO to 16MHz

    FRCTL0_H = 0xA5;
    FRCTL0_L = 1 << 4;
    
    //configure MCLK, SMCLK to be sourced by DCOCLK
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  // Set SMCLK = MCLK = DCO
    CSCTL3 = DIVA__2 | DIVS__1 | DIVM__1; //C DIVM__2;
    //Initialize LFXT1
    CSCTL4 |= LFXTDRIVE_3;
    // Lock CS control register
    CSCTL0_H = 0;                             // Lock CS registers
*/

impl<SMCLK: SmclkState> ClockConfig<MclkDefined, SMCLK> {
    #[inline]
    fn configure_dco_fll(&self) {
        // Run FLL configuration procedure from the user's guide if we are using DCO
        if let MclkSel::Dcoclk(target_freq) = self.mclk.0 {
            fll_off();
            // self.periph.csctl3.write(|w| w.selref().refoclk());
            self.periph.csctl0.write(|w| unsafe { w.bits(0xa500) });
            self.periph
                .csctl1
                // .write(|w| w.dcorsel().variant(target_freq.dcorsel()));
                .write(|w|{ 
                    if target_freq.dcorsel() {
                        w.dcorsel().set_bit().dcofsel().bits(target_freq.dcofsel())
                    } else {
                        w.dcorsel().clear_bit().dcofsel().bits(target_freq.dcofsel())
                    }
                });
            // self.periph.csctl2.write(|w| {
                // unsafe { w.flln().bits(target_freq.multiplier() - 1) }
                    // .flld()
                    // ._1()
            // });

            msp430::asm::nop();
            msp430::asm::nop();
            msp430::asm::nop();
            fll_on();

            // while !self.periph.csctl7.read().fllunlock().is_fllunlock_0() {}
        }
    }

    #[inline]
    fn configure_cs(&self) {
        // Configure clock selector and divisors
        self.periph.csctl2.write(|w| {
            w.sela()
                .variant(self.aclk_sel.sela())
                .selm()
                .variant(self.mclk.0.selm())
                .sels()
                .variant(self.sclk_sel.sels())
        });

        self.periph.csctl3.write(|w| {
            let w = w.divm().variant(self.mclk_div).diva().diva_1();
            match self.smclk.div() {
                //Some(div) => w.divs().variant(div),
                Some(div) => w.divs().variant(SmclkDiv::DIVS_0),
                None => w.divs().variant(SmclkDiv::DIVS_0),
            }
        });
        self.periph.csctl4.write(|w| {
            w.lfxtdrive().lfxtdrive_3()
        });
        //self.periph.csctl0.modify(|r,w| unsafe { w.bits(r.bits() & 0x00ff) });
    }

    #[inline]
    unsafe fn configure_fram(fram: &mut Fram, mclk_freq: u32) {
        if mclk_freq > 16_000_000 {
            fram.set_wait_states(WaitStates::Wait2);
        } else if mclk_freq > 8_000_000 {
            fram.set_wait_states(WaitStates::Wait1);
        } else {
            fram.set_wait_states(WaitStates::Wait0);
        }
    }
}

impl ClockConfig<MclkDefined, SmclkDefined> {
    /// Apply clock configuration to hardware and return SMCLK and ACLK clock objects
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Smclk, Aclk) {
        let mclk_freq = self.mclk.0.freq() >> (self.mclk_div as u32);
        unsafe { Self::configure_fram(fram, mclk_freq) };
        self.configure_dco_fll();
        self.configure_cs();
        (
            Smclk(mclk_freq >> (self.smclk.0 as u32)),
            Aclk(self.aclk_sel.freq()),
        )
    }
}

impl ClockConfig<MclkDefined, SmclkDisabled> {
    /// Apply clock configuration to hardware and return ACLK clock object, as SMCLK is disabled
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> Aclk {
        let mclk_freq = self.mclk.0.freq() >> (self.mclk_div as u32);
        self.configure_dco_fll();
        unsafe { Self::configure_fram(fram, mclk_freq) };
        self.configure_cs();
        Aclk(self.aclk_sel.freq())
    }
}

/// SMCLK clock object
pub struct Smclk(u32);
/// ACLK clock object
pub struct Aclk(u16);

/// Trait for configured clock objects
pub trait Clock {
    /// Type of the returned frequency value
    type Freq;

    /// Frequency of the clock
    fn freq(&self) -> Self::Freq;
}

impl Clock for Smclk {
    type Freq = u32;

    /// Returning a 32-bit frequency may seem suspect, since we're on a 16-bit system, but it is
    /// required as SMCLK can go up to 24 MHz. Clock frequencies are usually for initialization
    /// tasks such as computing baud rates, which should be optimized away, avoiding the extra cost
    /// of 32-bit computations.
    #[inline]
    fn freq(&self) -> u32 {
        self.0
    }
}

impl Clock for Aclk {
    type Freq = u16;

    #[inline]
    fn freq(&self) -> u16 {
        self.0
    }
}

