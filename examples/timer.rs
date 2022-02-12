#![no_main]
#![no_std]

use embedded_hal::digital::v2::*;
use embedded_hal::prelude::*;
use msp430_rt::entry;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, P3},
    pmm::Pmm,
    timer::{CapCmp, SubTimer, Timer, TimerConfig, TimerDiv, TimerExDiv, TimerParts3, TimerPeriph},
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;
use void::ResultVoidExt;

// 0.5 second on, 0.5 second off
#[entry]
fn main() -> ! {
    let periph = msp430fr5949::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.FRAM);
    Wdt::constrain(periph.WATCHDOG_TIMER);

    let pmm = Pmm::new(periph.PMM);
    let p3 = Batch::new(P3 {
        port: periph.PORT_3_4,
    })
    .config_pin1(|p| p.to_output())
    .split(&pmm);
    let mut p3_1 = p3.pin1;

    let (_smclk, aclk) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::DIVM_0)
        .smclk_on(SmclkDiv::DIVS_1, DcoclkFreqSel::_16MHz)
        .aclk_vloclk()
        .freeze(&mut fram);

    let parts = TimerParts3::new(
        periph.TIMER_0_A3,
        TimerConfig::aclk(&aclk).clk_div(TimerDiv::_2, TimerExDiv::_5),
    );
    let mut timer = parts.timer;
    let mut subtimer = parts.subtimer2;

    set_time(&mut timer, &mut subtimer, 500);
    loop {
        block!(subtimer.wait()).void_unwrap();
        p3_1.set_high().void_unwrap();
        // first 0.5 s of timer countdown expires while subtimer expires, so this should only block
        // for 0.5 s
        block!(timer.wait()).void_unwrap();
        p3_1.set_low().void_unwrap();
    }
}

fn set_time<T: TimerPeriph + CapCmp<C>, C>(
    timer: &mut Timer<T>,
    subtimer: &mut SubTimer<T, C>,
    delay: u16,
) {
    timer.start(delay + delay);
    subtimer.set_count(delay);
}
