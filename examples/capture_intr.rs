#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::UnsafeCell;
use embedded_hal::digital::v2::ToggleableOutputPin;
use msp430::interrupt::{enable, Mutex};
use msp430_rt::entry;
use critical_section::with;
use msp430fr5949::interrupt;
use msp430fr5949_hal::{
    capture::{
        CapCmp, CapTrigger, Capture, CaptureParts7, CaptureVector, TBxIV, TimerConfig, CCR3,
    },
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::Batch,
    gpio::*,
    pmm::Pmm,
    watchdog::Wdt,
};
use void::ResultVoidExt;

// #[cfg(debug_assertions)]
use panic_msp430 as _;

// #[cfg(not(debug_assertions))]
// use panic_never as _;

static CAPTURE: Mutex<UnsafeCell<Option<Capture<msp430fr5949::TIMER_0_B7, CCR3>>>> =
    Mutex::new(UnsafeCell::new(None));
static VECTOR: Mutex<UnsafeCell<Option<TBxIV<msp430fr5949::TIMER_0_B7>>>> =
    Mutex::new(UnsafeCell::new(None));
static RED_LED: Mutex<UnsafeCell<Option<Pin<P1, Pin0, Output>>>> =
    Mutex::new(UnsafeCell::new(None));

// Connect push button input to P1.6. When button is pressed, red LED should toggle. No debouncing,
// so sometimes inputs are missed.
#[entry]
fn main() -> ! {
    if let Some(periph) = msp430fr5949::Peripherals::take() {
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
        .config_pin0(|p| p.to_output())
        .split(&pmm);

        let red_led = p1.pin0;

        let p3 = Batch::new(P3 {
            port: periph.PORT_3_4,
        })
        .split(&pmm);

        let captures = CaptureParts7::config(periph.TIMER_0_B7, TimerConfig::aclk(&aclk))
            .config_cap3_input_A(p3.pin4.to_alternate1())
            .config_cap3_trigger(CapTrigger::FallingEdge)
            // .config_cap3_trigger(CapTrigger::BothEdges)
            .commit();
        let mut capture = captures.cap3;
        let vectors = captures.tbxiv;

        setup_capture(&mut capture);
        with(|cs| {
            unsafe { *RED_LED.borrow(cs).get() = Some(red_led)}
            unsafe { *CAPTURE.borrow(cs).get() = Some(capture) }
            unsafe { *VECTOR.borrow(cs).get() = Some(vectors) }
        });
        unsafe { enable() };
    }

    loop {}
}

fn setup_capture<T: CapCmp<C>, C>(capture: &mut Capture<T, C>) {
    capture.enable_interrupts();
}

#[interrupt]
fn TIMER0_B1() {
    with(|cs| {
        if let Some(vector) = unsafe { &mut *VECTOR.borrow(cs).get() }.as_mut() {
            if let Some(capture) = unsafe { &mut *CAPTURE.borrow(cs).get() }.as_mut() {
                match vector.interrupt_vector() {
                    CaptureVector::Capture3(cap) => {
                        if cap.interrupt_capture(capture).is_ok() {
                            if let Some(led) = unsafe { &mut *RED_LED.borrow(cs).get() }.as_mut() {
                                led.toggle().void_unwrap();
                            }
                        }
                    }
                    _ => {}
                };
            }
        }
    });
}
