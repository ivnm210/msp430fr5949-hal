#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::RefCell;
use core::fmt;
use core::fmt::Debug;
use embedded_hal::blocking::spi::Transfer as SpiTransfer;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use msp430::asm;
use msp430::interrupt::{enable as enable_int, free, Mutex};
use msp430_rt::entry;
use msp430fr5949::interrupt;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, Output, Pin, Pin3, P1, P2, P3},
    pmm::Pmm,
    serial::*,
    spi::*,
    watchdog::Wdt,
};

// #[cfg(debug_assertions)]
use panic_msp430 as _;

static BLUE_LED: Mutex<RefCell<Option<Pin<P3, Pin3, Output>>>> = Mutex::new(RefCell::new(None));

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

struct MYNRF<E: Debug, CE: OutputPin<Error = E>, CSN: OutputPin<Error = E>, SPI: SpiTransfer<u8>> {
    ce: CE,
    csn: CSN,
    spi: SPI,
}

impl<
        E: Debug,
        CE: OutputPin<Error = E>,
        CSN: OutputPin<Error = E>,
        SPI: SpiTransfer<u8, Error = SPIE>,
        SPIE: Debug,
    > fmt::Debug for MYNRF<E, CE, CSN, SPI>
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "NRF24L01")
    }
}

impl<
        E: Debug,
        CE: OutputPin<Error = E>,
        CSN: OutputPin<Error = E>,
        SPI: SpiTransfer<u8, Error = SPIE>,
        SPIE: Debug,
    > MYNRF<E, CE, CSN, SPI>
{
    pub fn new(ce: CE, csn: CSN, spi: SPI) -> Self {
        // ce.set_low().ok();
        // csn.set_high().ok();
        let mut device = MYNRF { ce, csn, spi };
        device.ce.set_low().unwrap();
        device.csn.set_high().unwrap();
        // match device
        return device;
    }

    pub fn send_command(&mut self, adr: u8, data: &[u8]) -> u8 {
        let mut buf_data = [0; 33];
        let len = data.len();
        let buf = &mut buf_data[0..len + 1];
        buf[0] = adr;
        buf[1..len + 1].copy_from_slice(data);
        self.csn.set_low().ok();
        let _transfer = self.spi.transfer(buf).map(|_| {});
        self.csn.set_high().ok();
        let stat = data[0];
        return stat;
    }

    // pub fn init(&mut self) {
        // let mut data = [1u8];
        // data[0] = 0;
        // self.send_command(0x01u8, &data);
        // data[0] = 1;
        // self.send_command(0x02u8, &data);
        // data[0] = 1;
        // self.send_command(0x03u8, &data);
        // data[0] = 0x90;
        // self.send_command(0x05u8, &data);
    // }
}

fn myprint_u8(buf: &mut [u8], val: u8) {
    const B: &[u8; 16] = b"0123456789abcdef";
    buf[0] = B[(val / 16) as usize];
    buf[1] = B[(val % 16) as usize];
}

// #[cfg(not(debug_assertions))]
// use panic_never as _;

// Prints "HELLO" when started then echos on UART1
// Serial settings are listed in the code
#[entry]
fn main() -> ! {
    if let Some(periph) = msp430fr5949::Peripherals::take() {
        let mut fram = Fram::new(periph.FRAM);
        //let _wdt = Wdt::constrain(periph.WATCHDOG_TIMER);
        let mut wdt = Wdt::constrain(periph.WATCHDOG_TIMER).to_interval();

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
        .config_pin4(|p| p.to_output())
        .config_pin6(|p| p.to_output())
        .split(&pmm);

        let p2 = Batch::new(P2 {
            port: periph.PORT_1_2,
        })
        .split(&pmm);

        let (mut tx, mut _rx) = SerialConfig::new(
            periph.USCI_A1_UART_MODE,
            BitOrder::LsbFirst,
            BitCount::EightBits,
            StopBits::OneStopBit,
            // Launchpad UART-to-USB converter doesn't handle parity, so we don't use it
            Parity::NoParity,
            Loopback::NoLoop,
            115200,
        )
        .use_smclk(&smclk)
        .split(p2.pin5.to_alternate2(), p2.pin6.to_alternate2());
        let p = unsafe { msp430fr5949::Peripherals::steal() };
        let p1 = Batch::new(P1 { port: p.PORT_1_2 }).split(&pmm);

        let mode = embedded_hal::spi::MODE_0;
        let spi = Spi::spi0(
            periph.USCI_B0_SPI_MODE,
            (
                p2.pin2.to_alternate2(),
                p1.pin7.to_alternate2(),
                p1.pin6.to_alternate2(),
            ),
            mode,
            // tx,
        );
        let ce = p2.pin4.to_output();
        let csn = p3.pin0.to_output();
        tx.bwrite_all(b"we have csn ce\r\n").ok();
        let mut nrf = MYNRF::new(ce, csn, spi);
        // nrf.init();

        let mut p3_1 = p3.pin1;
        let mut p3_3 = p3.pin3;
        let mut p3_4 = p3.pin4;
        let mut p3_6 = p3.pin6;
        p3_1.set_high().unwrap();
        p3_3.set_high().unwrap();
        p3_4.set_low().unwrap();
        p3_6.set_high().unwrap();

        free(|cs| *BLUE_LED.borrow(*cs).borrow_mut() = Some(p3_3));

        // let p2_3 = p2.pin3;
        // let mut p6_6 = p6.pin6;
        let mut count = 0;

        tx.bwrite_all(b"HELLO\n").ok();
        let test = nrf.send_command(0x3, &[0x0]);
        let mut output = [0u8; 2];
        myprint_u8(&mut output, test);
        tx.bwrite_all(&output).ok();

        wdt.set_aclk(&aclk)
            // wdt.set_smclk(&smclk)
            .enable_interrupts()
            // .start(WdtClkPeriods::_32K);
            .start(7);
        unsafe { enable_int() };

        loop {
            delay(500_000);
            // p3_4.toggle().ok();

            if count & 1 == 1 {
                p3_4.set_low().unwrap();
            } else {
                p3_4.set_high().unwrap();
            }
            if count & 2 == 2 {
                p3_6.set_low().unwrap();
            } else {
                p3_6.set_high().unwrap();
            }
            if count & 4 == 4 {
                p3_1.set_low().unwrap();
            } else {
                p3_1.set_high().unwrap();
            }
            count = count + 1;
            // block!(tx.write('.' as u8)).ok();
        }
    } else {
        loop {}
    }
}

#[interrupt]
fn WDT() {
    free(|cs| {
        BLUE_LED.borrow(*cs).borrow_mut().as_mut().map(|blue_led| {
        blue_led.set_low().ok();
        blue_led.set_high().ok();
        })
    });
}
