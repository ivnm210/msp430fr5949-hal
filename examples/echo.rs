#![no_main]
#![no_std]

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use msp430_rt::entry;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, SmclkSel},
    fram::Fram,
    gpio::{Batch, P1, P2, P4},
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use nb::block;

// #[cfg(debug_assertions)]
use panic_msp430 as _;

// #[cfg(not(debug_assertions))]
// use panic_never as _;

// Prints "HELLO" when started then echos on UART1
// Serial settings are listed in the code
#[entry]
fn main() -> ! {
    if let Some(periph) = msp430fr5949::Peripherals::take() {
        let mut fram = Fram::new(periph.FRAM);
        let _wdt = Wdt::constrain(periph.WATCHDOG_TIMER);

        let (smclk, _aclk) = ClockConfig::new(periph.CS)
            .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::DIVM_1)
            .smclk_on(SmclkDiv::DIVS_1, DcoclkFreqSel::_16MHz)
            .aclk_vloclk()
            .freeze(&mut fram);

        let pmm = Pmm::new(periph.PMM);
        let mut led = Batch::new(P4 { port : periph.PORT_3_4 }).split(&pmm).pin6.to_output();
        let p2 = Batch::new(P2 {port : periph.PORT_1_2 }).split(&pmm);
        led.set_low().ok();

        let (mut tx, mut rx) = SerialConfig::new(
            periph.USCI_A0_UART_MODE,
            BitOrder::LsbFirst,
            BitCount::EightBits,
            StopBits::OneStopBit,
            // Launchpad UART-to-USB converter doesn't handle parity, so we don't use it
            Parity::NoParity,
            Loopback::NoLoop,
            115200, //9600,
        )
        .use_smclk(&smclk)
        .split(p2.pin0.to_alternate2(), p2.pin1.to_alternate2());

        led.set_high().ok();
        tx.bwrite_all(b"HELLO\n").ok();

        loop {
            let ch = match block!(rx.read()) {
                Ok(c) => c,
                Err(err) => {
                    (match err {
                        RecvError::Parity => '!',
                        RecvError::Overrun(_) => '}',
                        RecvError::Framing => '?',
                    }) as u8
                }
            };
            block!(tx.write(ch)).ok();
        }
    } else {
        loop {}
    }
}
