#![no_main]
#![no_std]

use embedded_hal::digital::v2::*;
use embedded_hal::prelude::*;
use embedded_hal::timer::Cancel;
use msp430_rt::entry;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, SmclkSel},
    fram::Fram,
    gpio::{Batch, GpioVector, Output, Pin, Pin2, Pin3, PxIV, P1, P2, P3},
    pmm::Pmm,
    rtc::{Rtc},
    watchdog::Wdt,
};
use panic_msp430 as _;

// Red LED blinks 2 seconds on, 2 off
// Pressing P2.3 button toggles red LED and halts program
#[entry]
fn main() -> ! {
    let periph = msp430fr5949::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.FRAM);
    // let _wdt = Wdt::constrain(periph.WATCHDOG_TIMER);
    let mut wdt = Wdt::constrain(periph.WATCHDOG_TIMER).to_interval();

    let (smclk, aclk) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::DIVM_0)
        .smclk_on(SmclkDiv::DIVS_1, DcoclkFreqSel::_16MHz)
        .aclk_vloclk()
        .freeze(&mut fram);

    let pmm = Pmm::new(periph.PMM);
    
    let p1 = Batch::new(P1 {
        port: periph.PORT_1_2,
    })
    .split(&pmm);
    let mut button = p1.pin0.pulldown();

    let p3 = Batch::new(P3 {
        port: periph.PORT_3_4,
    })
    .config_pin3(|p| p.to_output())
    .split(&pmm);
    let mut led = p3.pin3;

    let mut rtc = Rtc::new(periph.RTC_B_REAL_TIME_CLOCK);
    // rtc.set_clk_div(RtcDiv::_10);

    button.select_falling_edge_trigger();
    led.set_high().ok();

    loop {
        // 2 seconds
        // rtc.start(2000u16);
        rtc.start(58u16); // 60 - 2 min
        while let Err(nb::Error::WouldBlock) = rtc.wait() {
            if let Ok(_) = button.wait_for_ifg() {
                led.toggle().ok();
                rtc.cancel().ok();
            }
        }
        led.toggle().ok();
    }
}
