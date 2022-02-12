#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

extern crate msp430fr5949;

use msp430fr5949::interrupt;

use core::cell::RefCell;
use embedded_hal::digital::v2::*;
use embedded_hal::timer::*;
use msp430::interrupt::{enable as enable_int, free, Mutex};
use msp430_rt::entry;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, SmclkSel},
    fram::Fram,
    gpio::{Batch, GpioVector, Output, Pin, Pin0, PxIV, P1, P2, P3},
    pmm::Pmm,
    watchdog::{Wdt, WdtClkPeriods},
};
use nb::block;
use panic_msp430 as _;

static RED_LED: Mutex<RefCell<Option<Pin<P1, Pin0, Output>>>> = Mutex::new(RefCell::new(None));
static P1IV: Mutex<RefCell<Option<PxIV<P1>>>> = Mutex::new(RefCell::new(None));

// Red LED should blink 2 seconds on, 2 seconds off
// Both green and red LEDs should blink when P2.3 LED is pressed
#[entry]
fn main() -> ! {
    let periph = msp430fr5949::Peripherals::take().unwrap();
    let mut fram = Fram::new(periph.FRAM);
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
    .config_pin1(|p| p.pullup())
    .split(&pmm);

    let p3 = Batch::new(P3 {
        port: periph.PORT_3_4,
    })
    .config_pin4(|p| p.to_output())
    .split(&pmm);

    let red_led = p1.pin0.to_output();
    // Onboard button with interrupt disabled
    let mut button = p1.pin1;
    // Some random pin with interrupt enabled. IFG will be set manually.
    let mut pin = p3.pin5.pulldown();
    let mut green_led = p3.pin4;
    let p1iv = p1.pxiv;

    free(|cs| *RED_LED.borrow(*cs).borrow_mut() = Some(red_led));
    free(|cs| *P1IV.borrow(*cs).borrow_mut() = Some(p1iv));

    wdt.set_aclk(&aclk)
        .enable_interrupts()
        // .start(WdtClkPeriods::_32K);
        .start(4);
    pin.select_rising_edge_trigger().enable_interrupts();
    button.select_falling_edge_trigger();

    unsafe { enable_int() };

    loop {
        block!(button.wait_for_ifg()).ok();
        green_led.toggle().ok();
        pin.set_ifg();
    }
}

#[interrupt]
fn PORT1() {
    free(|cs| {
        RED_LED.borrow(*cs).borrow_mut().as_mut().map(|red_led| {
            match P1IV
                .borrow(*cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .get_interrupt_vector()
            {
                GpioVector::Pin7Isr => red_led.toggle().ok(),
                _ => panic!(),
            }
        })
    });
}

#[interrupt]
fn WDT() {
    free(|cs| {
        RED_LED.borrow(*cs).borrow_mut().as_mut().map(|red_led| {
            red_led.toggle().ok();
        })
    });
}
