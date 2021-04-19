#![no_main]
#![no_std]

use embedded_hal::digital::v2::*;
use msp430_rt::entry;
use msp430::{asm, interrupt};
// use msp430fr5949_hal::{gpio::Batch, pmm::Pmm, watchdog::Wdt};
// use msp430fr5949_hal::gpio::{P1, P2, P3};
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, SmclkSel},
    fram::Fram,
    gpio::{Batch, P1, P2, P3},
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use msp430fr5949::*;
use panic_msp430 as _;

fn delay(n: u32) {
    let mut i = 0;
    loop {
        asm::nop();
        i += 1;
        if i == n {
            break;
        }
    }
}

fn uart1_init()
{
    // let periph = msp430fr5949::Peripherals::take().unwrap();
    // let p2 = periph.PORT_1_2;
    // p2.p2sel0.modify(|_r,w| w.p2sel0.p2sel0_5().clear_bit());

}

fn uart_putc(ch : u8)
{

}

fn uart_puts(buf : &str)
{

}

// Red onboard LED should blink at a steady period.
// Green onboard LED should go on when P2.3 button is pressed
#[entry]
fn main() -> ! {
    let periph = msp430fr5949::Peripherals::take().unwrap();
    let mut fram = Fram::new(periph.FRAM);
    let _wdt = Wdt::constrain(periph.WATCHDOG_TIMER);

    let (smclk, _aclk) = ClockConfig::new(periph.CS)
            .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::DIVM_1)
            .smclk_on(SmclkDiv::DIVS_1, DcoclkFreqSel::_16MHz)
            .aclk_vloclk()
            .freeze(&mut fram);

    let pmm = Pmm::new(periph.PMM);
    let p3 = Batch::new(P3 { port : periph.PORT_3_4})
        .config_pin1(|p| p.to_output())
        .config_pin3(|p| p.to_output())
        .config_pin4(|p| p.to_output())
        .config_pin6(|p| p.to_output())
        .split(&pmm);

    //65et periph = msp430fr594965Peripherals::take().unwr65();
    // let p2 = Batc6::new(P2 { port : periph.PORT_1_2})
    //      .config_pin3(|p| p.pullup())
    //      .split(&pmm);

    // let p6 = Batch::new(periph.PORT_J)
    //     .config_pin6(|p| p.to_output())
    //     .split(&pmm);

    // let mut p3_5 = p3.pin5.to_output();
    let mut p3_1 = p3.pin1;
    let mut p3_3 = p3.pin3;
    let mut p3_4 = p3.pin4;
    let mut p3_6 = p3.pin6;
    // let p2_3 = p2.pin3;
    // let mut p6_6 = p6.pin6;
    let mut count = 0;

    loop {
        delay(500_000);
        p3_4.toggle().ok();

        if count & 4 == 4 {
            p3_3.set_high().unwrap();
        } else {
            p3_3.set_low().unwrap();
        }
        if count & 2 == 2 {
            p3_1.set_high().unwrap();
        } else {
            p3_1.set_low().unwrap();
        }
        if count & 8 == 8 {
            p3_6.set_high().unwrap();
        } else {
            p3_6.set_low().unwrap();
        }
        count = count + 1;

        // for _ in 0..5000 {
        //     if p2_3.is_high().unwrap() {
        //         // p6_6.set_low().ok();
        //         p1_0.set_low().ok();
        //     } else {
        //         p1_0.set_high().ok();
        //         // p6_6.set_high().ok();
        //     }
        // }

    }
}
