#![no_main]
#![no_std]

use embedded_hal::digital::v2::*;
use msp430::{asm, interrupt};
use msp430_rt::entry;
// use msp430fr5949_hal::{gpio::Batch, pmm::Pmm, watchdog::Wdt};
// use msp430fr5949_hal::gpio::{P1, P2, P3};
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use msp430fr5949::*;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, SmclkSel},
    fram::Fram,
    gpio::{Batch, P1, P2, P3},
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use panic_msp430 as _;

pub struct Delay {}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new() -> Self {
        Delay {}
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) {}
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(ms as u32);
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(ms as u32);
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        // Tricky to get this to not overflow
        let mut i = us;
        asm::nop();
        loop {
            asm::nop();
            i -= 1;
            if i == 0 {
                break;
            }
        }
        // for j in 0..i/2 {
        // asm::nop();
        // }
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(us as u32)
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(us as u32)
    }
}

fn uart1_init() {
    // let periph = msp430fr5949::Peripherals::take().unwrap();
    // let p2 = periph.PORT_1_2;
    // p2.p2sel0.modify(|_r,w| w.p2sel0.p2sel0_5().clear_bit());
}

fn uart_putc(ch: u8) {}

fn uart_puts(buf: &str) {}

// Red onboard LED should blink at a steady period.
// Green onboard LED should go on when P2.3 button is pressed
#[entry]
fn main() -> ! {
    let periph = msp430fr5949::Peripherals::take().unwrap();
    let mut fram = Fram::new(periph.FRAM);
    let _wdt = Wdt::constrain(periph.WATCHDOG_TIMER);

    // let (smclk, _aclk) = ClockConfig::new(periph.CS)
    // .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::DIVM_1)
    // .smclk_on(SmclkDiv::DIVS_1, DcoclkFreqSel::_16MHz)
    // .aclk_vloclk()
    // .freeze(&mut fram);
    let (smclk, aclk) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::DIVM_0)
        .smclk_on(SmclkDiv::DIVS_1, DcoclkFreqSel::_16MHz)
        .aclk_vloclk()
        .freeze(&mut fram);

    let pmm = Pmm::new(periph.PMM);
    let p3 = Batch::new(P3 {
        port: periph.PORT_3_4,
    })
    .config_pin1(|p| p.to_output())
    .config_pin3(|p| p.to_output())
    .config_pin5(|p| p.to_output())
    .config_pin6(|p| p.to_output())
    .config_pin7(|p| p.to_output())
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
    let mut p3_5 = p3.pin5;
    let mut p3_6 = p3.pin6;
    let mut p3_7 = p3.pin7;
    // let p2_3 = p2.pin3;
    // let mut p6_6 = p6.pin6;
    let mut count = 0;
    p3_5.set_low().unwrap();
    p3_6.set_low().unwrap();
    p3_6.set_high().unwrap();
    p3_6.set_low().unwrap();
    p3_6.set_high().unwrap();
    p3_6.set_low().unwrap();
    p3_6.set_high().unwrap();
    p3_6.set_low().unwrap();
    p3_6.set_high().unwrap();
    p3_7.set_low().unwrap();
    let mut delayd = Delay::new();

    loop {
        p3_6.toggle().ok();
        p3_5.toggle().ok();
        p3_6.toggle().ok();
        p3_7.toggle().ok();
        p3_6.toggle().ok();
        p3_3.toggle().ok();
        p3_6.toggle().ok();
        delayd.delay_us(100u32); // delay(5_000);

        // if count & 4 == 4 {
        // p3_3.set_high().unwrap();
        // } else {
        // p3_3.set_low().unwrap();
        // }
        // if count & 2 == 2 {
        // p3_1.set_high().unwrap();
        // } else {
        // p3_1.set_low().unwrap();
        // }
        // if count & 8 == 8 {
        // p3_6.set_high().unwrap();
        // } else {
        // p3_6.set_low().unwrap();
        // }
        // count = count + 1;

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
