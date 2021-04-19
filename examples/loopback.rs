#![no_main]
#![no_std]

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use embedded_hal::serial::Read;
use msp430_rt::entry;
use msp430fr5969_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, Smclk, SmclkDiv},
    fram::Fram,
    gpio::{Batch, P1, P2, P4},
    pmm::Pmm,
    serial::*,
    watchdog::Wdt,
};
use nb::block;
use panic_msp430 as _;

fn setup_uart<S: SerialUsci>(
    usci: S,
    tx: S::TxPin,
    rx: S::RxPin,
    parity: Parity,
    loopback: Loopback,
    baudrate: u32,
    smclk: &Smclk,
) -> (Tx<S>, Rx<S>) {
    SerialConfig::new(
        usci,
        BitOrder::LsbFirst,
        BitCount::EightBits,
        StopBits::TwoStopBits,
        parity,
        loopback,
        baudrate,
    )
    .use_smclk(&smclk)
    .split(tx, rx)
}

fn read_unwrap<R: Read<u8>>(rx: &mut R, err: char) -> u8 {
    match block!(rx.read()) {
        Ok(c) => c,
        Err(_) => err as u8,
    }
}

// Echoes serial input on UART1 by roundtripping to UART0
// Only UART1 settings matter for the host
#[entry]
fn main() -> ! {
    let periph = msp430fr5969::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.FRAM);
    let _wdt = Wdt::constrain(periph.WATCHDOG_TIMER);

    let (smclk, _aclk) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_4MHz, MclkDiv::DIVM_1)
        .smclk_on(SmclkDiv::DIVS_2)
        .aclk_refoclk()
        .freeze(&mut fram);

    let pmm = Pmm::new(periph.PMM);
    let p1 = Batch::new(P1 {port : periph.PORT_1_2}).split(&pmm);
    let periph = msp430fr5969::Peripherals::take().unwrap();
    let p2 = Batch::new(P2 {port : periph.PORT_1_2}).split(&pmm);
    let periph = msp430fr5969::Peripherals::take().unwrap();
    let p4 = Batch::new(P4 {port : periph.PORT_3_4}).split(&pmm);
    let mut led = p1.pin0.to_output();
    let mut led2 = p4.pin6.to_output();
    led.set_low().ok();

    let (mut tx0, mut rx0) = setup_uart(
        periph.USCI_A0_UART_MODE,
        p2.pin0.to_alternate2().into(),
        p2.pin1.to_alternate2().into(),
        Parity::EvenParity,
        Loopback::Loopback,
        20000,
        &smclk,
    );

    let (mut tx1, mut rx1) = setup_uart(
        periph.USCI_A1_UART_MODE,
        p2.pin5.to_alternate2().into(),
        p2.pin6.to_alternate2().into(),
        Parity::NoParity,
        Loopback::NoLoop,
        19200,
        &smclk,
    );

    led.set_high().ok();

    loop {
        led2.set_low().ok();

        let ch = read_unwrap(&mut rx1, '!');
        block!(tx0.write(ch)).ok();
        let ch = read_unwrap(&mut rx0, '?');
        block!(tx1.write(ch)).ok();
        led2.set_high().ok();
    }
}
