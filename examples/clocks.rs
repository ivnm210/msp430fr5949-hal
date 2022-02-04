#![no_main]
#![no_std]

use embedded_hal::digital::v2::*;
use embedded_hal::timer::CountDown;
use embedded_hal::watchdog::WatchdogEnable;
use msp430_rt::entry;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    watchdog::{Wdt, WdtClkPeriods},
};
use nb::block;
use panic_msp430 as _;

// Red LED should blink 1 second on, 1 second off
#[entry]
fn main() -> ! {
    let periph = msp430fr5949::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.FRAM);
    let wdt = Wdt::constrain(periph.WATCHDOG_TIMER);

    let pmm = Pmm::new(periph.PMM);
    let p3 = Batch::new(P3 { port : periph.PORT_3_4})
        .config_pin1(|p| p.to_output())
        .split(&pmm);
    let mut p3_1 = p3.pin1;

    let (smclk, _aclk) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::DIVM_0)
        .smclk_on(SmclkDiv::DIVS_1, DcoclkFreqSel:_16MHz)
        .aclk_vloclk()
        .freeze(&mut fram);

    const DELAY: WdtClkPeriods = WdtClkPeriods::_8192K;

    // blinks should be 1 second on, 1 second off
    let mut wdt = wdt.to_interval();
    p3_1.set_high().ok();
    wdt.set_smclk(&smclk).start(DELAY);

    block!(wdt.wait()).ok();
    p3_1.set_low().ok();

    let mut wdt = wdt.to_watchdog();
    wdt.start(DELAY);

    loop {
        msp430::asm::nop();
    }
}
