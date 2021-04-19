#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use msp430::{asm};
use core::cell::RefCell;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use msp430_rt::entry;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, SmclkSel},
    fram::Fram,
    gpio::{Batch, GpioVector, PxIV, P1, P2, P3, Output, Pin, Pin3},
    pmm::Pmm,
    serial::*,
    spi::*,
    watchdog::Wdt,
};
use nb::block;
use msp430::interrupt::{enable as enable_int, free, Mutex};
use msp430fr5949::interrupt;
extern crate embedded_nrf24l01;
use embedded_nrf24l01::*;
use embedded_nrf24l01 as nrf24;

// #[cfg(debug_assertions)]
use panic_msp430 as _;

static BLUE_LED: Mutex<RefCell<Option<Pin<P3, Pin3, Output>>>> = Mutex::new(RefCell::new(None));
static P1IV: Mutex<RefCell<Option<PxIV<P1>>>> = Mutex::new(RefCell::new(None));

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

struct LSpi {
    cs : *mut u8,
}

impl LSpi {
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
        s.ucb0br0_spi.write(|w| unsafe { w.bits(2) } );
        s.ucb0ctl0_spi.modify(|r,w| unsafe { w.bits(r.bits() & 0xFE) } );
        return LSpi{cs : c};
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

struct LNrf <'a>{
    spi : &'a LSpi,
}

impl<'a> LNrf <'a>{
    fn new(s : &'a LSpi) -> Self
    {
        return LNrf{spi : s};
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
        let p3 = Batch::new(P3 { port : periph.PORT_3_4})
            .config_pin1(|p| p.to_output())
            .config_pin3(|p| p.to_output())
            .config_pin4(|p| p.to_output())
            .config_pin6(|p| p.to_output())
            .split(&pmm);

        let p2 = Batch::new(P2 { port : periph.PORT_1_2})
             .split(&pmm);

        let (mut tx, mut rx) = SerialConfig::new(
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
        let p1 = Batch::new(P1 { port : p.PORT_1_2})
             .split(&pmm);

    //    let mut scs = 1;
    //    let spi = LSpi::init(&mut scs);
        let mode = embedded_hal::spi::MODE_0;
        let spi = Spi::spi0(
                        periph.USCI_B0_SPI_MODE,
                        (p2.pin2.to_alternate2(),
                        p1.pin7.to_alternate2(),
                        p1.pin6.to_alternate2()),
                        mode,
                        // tx,
                    );
        // let nrf = LNrf::new(&spi);
        // nrf.init();
        let ce = p2.pin4.to_output();
        let cs = p3.pin0.to_output();
        tx.bwrite_all(b"we have cs ce\r\n").ok();
        let mut nrf24 = NRF24L01::new(ce, cs, spi).unwrap();

        nrf24.set_frequency(108).unwrap();
        tx.bwrite_all(b"We have frequency\r\n");
        nrf24.set_auto_retransmit(0, 0).unwrap();
        nrf24.set_rf(&nrf24::DataRate::R2Mbps, 3).unwrap();
        nrf24.set_pipes_rx_enable(&[true, false, false, false, false, false]).unwrap();
        nrf24.set_auto_ack(&[false; 6]).unwrap();
        nrf24.set_crc(nrf24::CrcMode::Disabled).unwrap();
        nrf24.set_tx_addr(&b"fnord"[..]).unwrap();
        nrf24.set_rx_addr(0, &b"fnord"[..]).unwrap();

        let mut p3_1 = p3.pin1;
        let mut p3_3 = p3.pin3;
        let mut p3_4 = p3.pin4;
        let mut p3_6 = p3.pin6;
        p3_1.set_high().unwrap();
        p3_3.set_high().unwrap();
        p3_4.set_low().unwrap();
        p3_6.set_high().unwrap();
        let p1iv = p1.pxiv;

        free(|cs| *BLUE_LED.borrow(&cs).borrow_mut() = Some(p3_3));
        free(|cs| *P1IV.borrow(&cs).borrow_mut() = Some(p1iv));

        let mut count = 0;

        tx.bwrite_all(b"HELLO\n\r").ok();

        wdt.set_aclk(&aclk)
        // wdt.set_smclk(&smclk)
        .enable_interrupts()
        // .start(WdtClkPeriods::_32K);
        .start(7);
        unsafe { enable_int() };


//
// let mut nrf24 = NRF24L01::new(ce, csn, spi).unwrap();
//
// This will provide an instance of Standby. You can use .rx() or .tx() to transfer
// into a RXMode and TXMode instances. They implement .standby() methods to get back
// to Standby and then switch to the other mode.
//
//
// use rx.can_read() to poll (returning a pipe number) then rx.read() to receive payload
// use tx.send() to enqueue a packet
// use tx.can_send() to prevent sending on full queue, and tx.wait_empty() to flush
//
//

        // let mut nrf24rx = match nrf24.rx() {
        //     Ok(nrf) => nrf,
        //     Err(_) => {
        //         tx.bwrite_all(b"panic nrf24.rx\r\n");
        //         panic!() },
        // };
        let mut nrf24rx = nrf24.rx().unwrap();

        // if let Ok(Some(pipe)) = nrf24rx.can_read() {
        if let pipe = nrf24rx.can_read().unwrap() {
            let pl = nrf24rx.read();
        }

        let mut nrf24tx = nrf24rx.standby().tx().unwrap();

        // if let Ok(test) = nrf24tx.can_send() {
        if let test = nrf24tx.can_send().unwrap() {
            let pkt = [1u8, 2u8, 3u8, 4u8];
            nrf24tx.send(&pkt);
        }

        nrf24 = nrf24tx.standby().unwrap();

        loop {
            delay(500_000);
            // p3_4.toggle().ok();
            nrf24rx = nrf24.rx().unwrap();

            if let pipe = nrf24rx.can_read().unwrap() {
                let pl = nrf24rx.read();
            }

            nrf24tx = nrf24rx.standby().tx().unwrap();

            if let test = nrf24tx.can_send().unwrap() {
                let pkt = [1u8, 2u8, 3u8, 4u8];
                nrf24tx.send(&pkt);
            }

            nrf24 = nrf24tx.standby().unwrap();

            // if let Ok(mut nrf24rx) = nrf24.rx() {
            //     if let Ok(Some(pipe)) = nrf24rx.can_read() {
            //         let pl = nrf24rx.read();
            //     }
            //     if let Ok(mut nrf24tx) = nrf24rx.standby().tx() {
            //         if let Ok(test) = nrf24tx.can_send() {
            //             let pkt = [1u8, 2u8, 3u8, 4u8];
            //             nrf24tx.send(&pkt);
            //         }
            //         if let Ok(nrf) = nrf24tx.standby() {
            //             nrf24 = nrf;
            //         }
            //     }
            // }

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
            // free(|cs| {
                // uart.putc('.');
            // });
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
            //block!(tx.write(ch)).ok();
            // block!(tx.write('.' as u8)).ok();
        }
    } else {
        loop {}
    }
}

// #[interrupt]
// fn USCI_A1() {
//     free(|cs| {
//         let p = unsafe { msp430fr5949::Peripherals::steal() };
//         let uart1 = p.USCI_A1_UART_MODE;
//         if uart1.uca1ifg.read().ucrxifg() == true {
//             let ch = uart1.uca1rxbuf.read();
//         }
//          BLUE_LED.borrow(&cs).borrow_mut().as_mut().map(|blue_led| {
//              blue_led.set_low().ok();
//              blue_led.set_high().ok();
//          })
//     });
// }
#[interrupt]
fn PORT1() {
    free(|cs| {
        BLUE_LED.borrow(&cs).borrow_mut().as_mut().map(|blue_led| {
            match P1IV
                .borrow(&cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .get_interrupt_vector()
            {
                GpioVector::Pin2Isr => {blue_led.set_low().ok();
                blue_led.set_high().ok()},
                _ => panic!(),
            }
        })
    });
}

#[interrupt]
fn WDT() {
    free(|cs| {
        //BLUE_LED.borrow(&cs).borrow_mut().as_mut().map(|blue_led| {
            //blue_led.set_low().ok();
            //blue_led.set_high().ok();
        //})
    });
}
