#![no_main]
#![no_std]

use msp430_rt::entry;
use panic_msp430 as _;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, SmclkSel},
    fram::Fram,
    gpio::{Batch, GpioVector, Output, Pin, Pin2, Pin3, PxIV, P1, P2, P3},
    pmm::Pmm,
    serial::*,
    spi::*,
    timer::*,
    watchdog::{Wdt, WdtClkPeriods},
};

#[entry]
fn main() -> ! {
    if let Some(periph) = msp430fr5949::Peripherals::take() {
        let _wdt = Wdt::constrain(periph.WATCHDOG_TIMER);

        let pmm = Pmm::new(periph.PMM);
        let p1 = Batch::new(P2 {
            port: periph.PORT_1_2,
        })
        .split(&pmm);

        // Convert P1.0 to SMCLK output
        // Alternate 1 to alternate 2 conversion requires using SELC register
        // Expect red LED to light up
        p1.pin0.to_output().to_alternate1().to_alternate2();

        loop {
            msp430::asm::nop();
        }
    }
    loop {}
}
