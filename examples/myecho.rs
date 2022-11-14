#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::UnsafeCell;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use msp430::asm;
use msp430::interrupt::{enable as enable_int, Mutex};
use msp430_rt::entry;
use critical_section::with;
use msp430fr5949::interrupt;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv},
    fram::Fram,
    gpio::{Batch, Output, Pin, Pin3, P2, P3},
    pmm::Pmm,
    watchdog::Wdt,
};

// #[cfg(debug_assertions)]
use panic_msp430 as _;

static BLUE_LED: Mutex<UnsafeCell<Option<Pin<P3, Pin3, Output>>>> = Mutex::new(UnsafeCell::new(None));

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
/*
struct Spi {
    cs : *mut u8,
}

impl Spi {
    fn init(c : &mut u8) -> Self
    {
        let p = unsafe { msp430fr5949::Peripherals::steal()};
        let p1 = p.PORT_1_2;
        p1.p1sel0.modify(|_, w| w.p1sel0_6().clear_bit().p1sel0_7().clear_bit());
        p1.p1sel1.modify(|_, w| w.p1sel1_6().set_bit().p1sel1_7().set_bit());
        let p = unsafe { msp430fr5949::Peripherals::steal()};
        let p2 = p.PORT_1_2;
        p2.p2sel0.modify(|_, w| w.p2sel0_2().clear_bit());
        p2.p2sel1.modify(|_, w| w.p2sel1_2().set_bit());
        let s = p.USCI_B0_SPI_MODE;
        s.ucb0ctl0_spi.write(|w| unsafe { w.bits(1) } );
        s.ucb0ctl0_spi.modify(|r,w| unsafe { w.bits(r.bits() | 0x80) } );
        s.ucb0ctl1_spi.modify(|_,w| unsafe { w.bits(0xA9) } );
        s.ucb0ctl0_spi.modify(|r,w| unsafe { w.bits(r.bits() & 0xFE) } );
        return Spi{cs : c};
    }

    #[inline(always)]
    fn xfer_byte(&self, b : u8) -> u8
    {
        let p = unsafe { msp430fr5949::Peripherals::steal()};
        let s = p.USCI_B0_SPI_MODE;
        s.ucb0txbuf_spi.write(|w| unsafe { w.bits(b as u16) } );
        while s.ucb0ifg_spi.read().uctxifg() == false || s.ucb0ifg_spi.read().ucrxifg() == false {
            asm::nop;
        }
        return s.ucb0rxbuf_spi.read().bits() as u8;
    }

    #[inline(always)]
    fn csl(&self)
    {
        unsafe {  *(self.cs) = 0; }
    }

    #[inline(always)]
    fn csh(&self)
    {
        unsafe {  *(self.cs) = 1; }
    }
}

struct Nrf <'a>{
    spi : &'a Spi,
}

impl<'a> Nrf <'a>{
    fn new(s : &'a Spi) -> Self
    {
        return Nrf{spi : s};
    }
    fn write_register(&self, adr : u8, data : &[u8]) -> u8
    {
        self.spi.csl();
        let stat = self.spi.xfer_byte(adr);
        for ch in 0..data.len() {
            self.spi.xfer_byte(ch as u8);
        }
        self.spi.csh();
        return stat;
    }

    fn init(&self)
    {
        let mut data = [1u8];
        data[0] = 0;
        self.write_register(0x01u8, &data);
        data[0] = 1;
        self.write_register(0x02u8, &data);
        data[0] = 1;
        self.write_register(0x03u8, &data);
        data[0] = 0x90;
        self.write_register(0x05u8, &data);
    }
}
*/
/*
struct Uart0;

impl Uart0 {
    fn init() -> Self
    {
        let p = unsafe { msp430fr5949::Peripherals::steal()};
        let uart = p.USCI_A0_UART_MODE;
        uart.uca0ctl0.write(|w| w.ucswrst().set_bit() );
        uart.uca0br0.write(|w| unsafe { w.bits(0) } );
        uart.uca0br1.write(|w| unsafe { w.bits(8) } );
        uart.uca0mctlw.write(|w| w.ucos16().bit(true).
                                        ucbrs7().bit(true).
                                        ucbrs6().bit(true).
                                        ucbrs5().bit(true).
                                        ucbrs4().bit(true).
                                        ucbrs3().bit(false).
                                        ucbrs2().bit(true).
                                        ucbrs1().bit(true).
                                        ucbrs0().bit(true).
                                        ucbrf().bits(0xA));

        uart.uca0ctl0.modify(|_,w| unsafe { w.ucsselx().bits(2) } );
        uart.uca0ctl0.modify(|_,w| w.ucswrst().clear_bit() );
        uart.uca0ie.modify(|_,w| w.ucrxie().set_bit());
        return Uart0{};
    }

    #[inline(always)]
    fn putc(&self, ch : char)
    {
        let p = unsafe { msp430fr5949::Peripherals::steal() };
        let u = p.USCI_A0_UART_MODE;
        // if u.uca1ie.read().uctxie() == false {
            while u.uca0ifg.read().uctxifg().bit() == false {
                asm::nop();
            }
        // }
        u.uca0txbuf.write(|w| unsafe{ w.bits(ch as u16) } );
    }

    fn puts(&self, buf : &str)
    {
        // let len = buf.len();
        for ch in buf.bytes() {
            self.putc(ch as char);
        }
    }
}
*/

struct Uart1;

impl Uart1 {
    fn init() -> Self {
        let p = unsafe { msp430fr5949::Peripherals::steal() };
        let uart = p.USCI_A1_UART_MODE;
        uart.uca1ctl0
            .write(|w| unsafe { w.ucswrst().set_bit().ucsselx().bits(2) });
        uart.uca1br0.write(|w| unsafe { w.bits(8) });
        uart.uca1br1.write(|w| unsafe { w.bits(0) });
        uart.uca1mctlw.write(|w| {
            w.ucos16()
                .bit(true)
                .ucbrs7()
                .bit(true)
                .ucbrs6()
                .bit(true)
                .ucbrs5()
                .bit(true)
                .ucbrs4()
                .bit(true)
                .ucbrs3()
                .bit(false)
                .ucbrs2()
                .bit(true)
                .ucbrs1()
                .bit(true)
                .ucbrs0()
                .bit(true)
                .ucbrf()
                .bits(0xA)
        });

        // uart.uca1ctl0.modify(|_,w| unsafe { w.ucsselx().bits(2) } );
        uart.uca1ctl0.modify(|_, w| w.ucswrst().clear_bit());
        uart.uca1ie.modify(|_, w| w.ucrxie().set_bit());
        return Uart1 {};
    }

    #[inline(always)]
    fn putc(&self, ch: char) {
        let p = unsafe { msp430fr5949::Peripherals::steal() };
        let u = p.USCI_A1_UART_MODE;
        // if u.uca1ie.read().uctxie() == false {
        while u.uca1ifg.read().uctxifg().bit() == false {
            asm::nop();
        }
        // }
        u.uca1txbuf.write(|w| unsafe { w.bits(ch as u16) });
    }

    fn puts(&self, buf: &str) {
        // let len = buf.len();
        for ch in buf.bytes() {
            self.putc(ch as char);
        }
    }
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

        let (_smclk, aclk) = ClockConfig::new(periph.CS)
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

        // let p2 = Batch::new(P2 { port : periph.PORT_1_2})
        //     .config_pin5(|p| p.to_output())
        //     .config_pin6(|p| p.to_output())
        //     .split(&pmm);

        // let p2 = p.PORT_1_2;
        // p2.p2sel0.modify(|_, w| w.p2sel0_5().clear_bit().p2sel0_6().clear_bit());
        // p2.p2sel1.modify(|_, w| w.p2sel1_5().set_bit().p2sel1_6().set_bit());
        let p2 = Batch::new(P2 {
            port: periph.PORT_1_2,
        })
        .split(&pmm);
        let mut _p2_5 = p2.pin5.to_alternate2();
        let mut _p2_6 = p2.pin6.to_alternate2();

        // let mut p2_5 = p2.pin5;
        // let mut p2_6 = p2.pin6;

        // for i in 0..10 {
        //     p2_5.set_low();
        //     p2_5.set_high();
        // }
        // for i in 0..10 {
        //     p2_6.set_low();
        //     p2_6.set_high();
        // }
        // let p2 = Batch::new(P2 {port : periph.PORT_1_2 }).split(&pmm);

        // let (mut tx, mut rx) = SerialConfig::new(
        //     periph.USCI_A1_UART_MODE,
        //     BitOrder::LsbFirst,
        //     BitCount::EightBits,
        //     StopBits::OneStopBit,
        //     // Launchpad UART-to-USB converter doesn't handle parity, so we don't use it
        //     Parity::NoParity,
        //     Loopback::NoLoop,
        //     // 115200,
        //     9600,
        // )
        // //.use_aclk(&aclk)
        // .use_smclk(&smclk)
        // .split(p2.pin5.to_alternate2(), p2.pin6.to_alternate2());

        let uart = Uart1::init();

        // let mut cs = 1;
        // let spi = Spi::init(&mut cs);
        // let nrf = Nrf::new(&spi);
        // nrf.init();

        let mut p3_1 = p3.pin1;
        let mut p3_3 = p3.pin3;
        let mut p3_4 = p3.pin4;
        let mut p3_6 = p3.pin6;
        p3_1.set_high().unwrap();
        p3_3.set_high().unwrap();
        p3_4.set_low().unwrap();
        p3_6.set_high().unwrap();

        with(|cs| {
            unsafe { *BLUE_LED.borrow(cs).get() = Some(p3_3)}
        });

        // let p2_3 = p2.pin3;
        // let mut p6_6 = p6.pin6;
        let mut count = 0;

        // tx.bwrite_all(b"HELLO\n").ok();
        uart.puts("Hello there\r\n");
        //uart.putc('1');
        //uart.putc('2');
        //uart.putc('3');
        //uart.putc('4');

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
            // if count & 8 == 8 {
            //     p3_3.set_low().unwrap();
            // } else {
            //     p3_3.set_high().unwrap();
            // }
            count = count + 1;
            with(|_cs| {
                uart.putc('.');
            });
            // let ch = match block!(rx.read()) {
            //     Ok(c) => c,
            //     Err(err) => {
            //         (match err {
            //             RecvError::Parity => '!',
            //             RecvError::Overrun(_) => '}',
            //             RecvError::Framing => '?',
            //         }) as u8
            //     }
            // };
            // block!(tx.write(ch)).ok();
            // block!(tx.write('.' as u8)).ok();
        }
    } else {
        loop {}
    }
}

#[interrupt]
fn USCI_A1() {
    with(|cs| {
        let p = unsafe { msp430fr5949::Peripherals::steal() };
        let uart1 = p.USCI_A1_UART_MODE;
        if uart1.uca1ifg.read().ucrxifg() == true {
            let _ch = uart1.uca1rxbuf.read();
        }
        if let Some(blue_led) = unsafe {
            &mut *BLUE_LED.borrow(cs).get()
        } .as_mut() {
            blue_led.set_low().ok();
            blue_led.set_high().ok();
        }
    });
}

#[interrupt]
fn WDT() {
    with(|_cs| {
        //BLUE_LED.borrow(*cs).borrow_mut().as_mut().map(|blue_led| {
        //blue_led.set_low().ok();
        //blue_led.set_high().ok();
        //})
    });
}
