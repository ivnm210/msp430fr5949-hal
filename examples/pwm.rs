#![no_main]
#![no_std]

use embedded_hal::prelude::*;
use msp430_rt::entry;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    gpio::*,
    pmm::Pmm,
    pwm::{Pwm, PwmParts7, PwmPeriph, TimerConfig},
    watchdog::Wdt,
};
use panic_msp430 as _;

// P6.4 LED should be bright, P6.3 LED should be dim
#[entry]
fn main() -> ! {
    let periph = msp430fr5949::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.FRAM);
    Wdt::constrain(periph.WATCHDOG_TIMER);

    let pmm = Pmm::new(periph.PMM);
    // let p6 = Batch::new(periph.PORT_3_4).split(&pmm);
    let p3 = Batch::new(P3 {
        port: periph.PORT_3_4,
    })
    .split(&pmm);

    let (smclk, aclk) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::DIVM_0)
        .smclk_on(SmclkDiv::DIVS_1, DcoclkFreqSel::_16MHz)
        .aclk_vloclk()
        .freeze(&mut fram);

    let pwm = PwmParts7::new(periph.TIMER_0_B7, TimerConfig::smclk(&smclk), 5000);
    let mut pwm4 = pwm.pwm4.init(p3.pin5.to_output().to_alternate1());
    let mut pwm5 = pwm.pwm5.init(p3.pin6.to_output().to_alternate1());

    config_pwm(&mut pwm4, 100);
    config_pwm(&mut pwm5, 3795);

    loop {}
}

fn config_pwm<T: PwmPeriph<C>, C>(pwm: &mut Pwm<T, C>, duty: u16) {
    pwm.enable();
    pwm.set_duty(duty);
}
