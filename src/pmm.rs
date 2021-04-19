//! Power management module

use msp430fr5949::PMM;

/// PMM type
pub struct Pmm(());

impl Pmm {
    /// Sets the LOCKLPM5 bit and returns a `Pmm`
    pub fn new(pmm: PMM) -> Pmm {
        // pmm.pm5ctl0.write(|w| w.locklpm5());
        pmm.pm5ctl0.modify(|_, w| w.locklpm5().clear_bit());
        Pmm(())
    }
}
