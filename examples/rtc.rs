#![no_main]
#![no_std]

use embedded_hal::digital::v2::*;
use embedded_hal::prelude::*;
use embedded_hal::timer::Cancel;
use msp430_rt::entry;
use msp430fr5969_hal::{
    clock::{ClockConfig, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    pmm::Pmm,
    rtc::{Rtc, RtcDiv},
    watchdog::Wdt,
};
use panic_msp430 as _;

// Red LED blinks 2 seconds on, 2 off
// Pressing P2.3 button toggles red LED and halts program
#[entry]
fn main() -> ! {
    let periph = msp430fr5969::Peripherals::take().unwrap();

    Wdt::constrain(periph.WATCHDOG_TIMER);

    let pmm = Pmm::new(periph.PMM);
    let p1 = Batch::new(periph.PORT_1_2)
        .config_pin0(|p| p.to_output())
        .split(&pmm);
    let p2 = Batch::new(periph.PORT_3_4)
        .config_pin3(|p| p.pullup())
        .split(&pmm);
    let mut led = p1.pin0;
    let mut button = p2.pin3;

    let (_smclk, _aclk) = ClockConfig::new(periph.CS)
        .mclk_refoclk(MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_vloclk()
        .freeze(&mut Fram::new(periph.FRAM));

    let mut rtc = Rtc::new(periph.RTC).use_vloclk();
    rtc.set_clk_div(RtcDiv::_10);

    button.select_falling_edge_trigger();
    led.set_high().ok();

    loop {
        // 2 seconds
        rtc.start(2000u16);
        while let Err(nb::Error::WouldBlock) = rtc.wait() {
            if let Ok(_) = button.wait_for_ifg() {
                led.toggle().ok();
                rtc.cancel().ok();
            }
        }
        led.toggle().ok();
    }
}
