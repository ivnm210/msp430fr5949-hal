#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::UnsafeCell;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use msp430::asm;
use critical_section::with;
use msp430::interrupt::{enable as enable_int, Mutex};
use msp430_rt::entry;
use msp430fr5949::interrupt;
use msp430fr5949_hal::gpio::{Input, Pulldown};
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, SmclkSel},
    fram::Fram,
    gpio::{Batch, GpioVector, Output, Pin, Pin2, Pin3, PxIV, P1, P2, P3},
    pmm::Pmm,
    serial::*,
    spi::*,
    timer::*,
    watchdog::{Wdt, WdtClkPeriods},
};
use nb::block;
extern crate embedded_nrf24l01;
use embedded_nrf24l01 as nrf24;
use embedded_nrf24l01::*;

//#[cfg(debug_assertions)]
use panic_msp430 as _;

//#[cfg(not(debug_assertions))]
//use panic_never as _;

static BLUE_LED: Mutex<UnsafeCell<Option<Pin<P3, Pin3, Output>>>> = Mutex::new(UnsafeCell::new(None));
static INT_PIN: Mutex<UnsafeCell<Option<Pin<P1, Pin2, Input<Pulldown>>>>> =
    Mutex::new(UnsafeCell::new(None));
static P1IV: Mutex<UnsafeCell<Option<PxIV<P1>>>> = Mutex::new(UnsafeCell::new(None));
static MYBOOL: Mutex<UnsafeCell<Option<Mbool>>> = Mutex::new(UnsafeCell::new(None));

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

#[derive(Clone, Copy)]
enum RadioState {
    RadioStandby,
    RadioRx,
    RadioTx,
}

struct Mbool {
    bl: bool,
}

impl Mbool {
    fn new(bl: bool) -> Self {
        Mbool { bl }
    }
    fn set(&mut self) {
        self.bl = true;
    }
    fn clear(&mut self) {
        self.bl = false;
    }
    fn is_set(&self) -> bool {
        self.bl
    }
}

fn print_u8<U: SerialUsci>(tx: &mut Tx<U>, num: u8) {
    write(tx, '0');
    write(tx, 'x');
    print_hex(tx, ((num >> 4) & 0xF) as u16);
    print_hex(tx, (num & 0xF) as u16);
    write(tx, '\n');
}

fn print_u16<U: SerialUsci>(tx: &mut Tx<U>, num: u16) {
    write(tx, '0');
    write(tx, 'x');
    print_hex(tx, num >> 12);
    print_hex(tx, (num >> 8) & 0xF);
    print_hex(tx, (num >> 4) & 0xF);
    print_hex(tx, num & 0xF);
    write(tx, '\r');
    write(tx, '\n');
}

#[inline(always)]
fn print_hex<U: SerialUsci>(tx: &mut Tx<U>, h: u16) {
    let hex = "0123456789abcdef";
    write(tx, hex.as_bytes()[h as usize] as char);
    //block!(tx.write(hex.as_bytes()[h as usize])).unwrap();
}

#[inline(always)]
fn write<U: SerialUsci>(tx: &mut Tx<U>, ch: char) {
    block!(tx.write(ch as u8)).unwrap();
}

fn progress<U: SerialUsci>(tx: &mut Tx<U>, cnt: u8) {
    let ch: char = match cnt {
        1 => '\\',
        3 => '/',
        _ => '-',
    };
    block!(tx.write(ch as u8)).unwrap();
    block!(tx.write('\r' as u8)).unwrap();
    // block!(tx.write('\n' as u8)).unwrap();
}

fn prepare_pkt(pkt: &mut [u8; 32], cnt: u16) {
    let mut i = 0u8;
    //if pkt[0] == 0x41 {
    //pkt[0] = 0x61;
    //} else {
    //pkt[0] = 0x41
    //}
    for n in 0..29 {
        pkt[n] = pkt[0] + i;
        i = i + 1;
    }
    pkt[30] = ((cnt >> 8) & 0xFF) as u8;
    pkt[31] = (cnt & 0xff) as u8;
}

fn print_rec_pkt<U: SerialUsci>(tx: &mut Tx<U>, buf: &[u8], len: usize) {
    tx.bwrite_all(b"read = ").ok();
    tx.bwrite_all(&buf[0..len - 1]).ok();
    let cnt: u16 = ((buf[30] as u16) << 8) + (buf[31] as u16);
    tx.bwrite_all(b" ").ok();
    print_u16(tx, cnt);
}

fn print_snd_pkt<U: SerialUsci>(tx: &mut Tx<U>, buf: &[u8], len: usize) {
    tx.bwrite_all(b"send = ").ok();
    tx.bwrite_all(&buf[0..len - 1]).ok();
    let cnt: u16 = ((buf[30] as u16) << 8) + (buf[31] as u16);
    tx.bwrite_all(b" ").ok();
    print_u16(tx, cnt);
}

fn set_time<T: TimerPeriph + CapCmp<C>, C>(
    timer: &mut Timer<T>,
    subtimer: &mut SubTimer<T, C>,
    delay: u16,
) {
    if delay < 32768u16 {
        timer.start(delay + delay);
    } else {
        timer.start(65535u16);
    }
    subtimer.set_count(delay);
}

// Prints "HELLO" when started then echos on UART1
// Serial settings are listed in the code
#[entry]
fn main() -> ! {
    if let Some(periph) = msp430fr5949::Peripherals::take() {
        let mut fram = Fram::new(periph.FRAM);
        // let _wdt = Wdt::constrain(periph.WATCHDOG_TIMER);
        let mut wdt = Wdt::constrain(periph.WATCHDOG_TIMER).to_interval();

        let (smclk, aclk) = ClockConfig::new(periph.CS)
            .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::DIVM_0)
            .smclk_on(SmclkDiv::DIVS_1, DcoclkFreqSel::_16MHz)
            .aclk_vloclk()
            .freeze(&mut fram);

        let pmm = Pmm::new(periph.PMM);

        //delay(100000);
        // delay(1000);

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
        tx.bwrite_all(b"HELLO\n\r").ok();
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
        let mut p1_2 = p1.pin2.pulldown();
        p1_2.select_rising_edge_trigger().enable_interrupts();

        let mut p3_1 = p3.pin1;
        let mut p3_3 = p3.pin3;
        let mut p3_4 = p3.pin4;
        let mut p3_6 = p3.pin6;
        p3_1.set_high().unwrap();
        p3_1.set_low().unwrap();
        p3_1.set_high().unwrap();
        p3_3.set_high().unwrap();
        p3_3.set_low().unwrap();
        p3_3.set_high().unwrap();
        p3_4.set_low().unwrap();
        p3_6.set_high().unwrap();
        let p1iv = p1.pxiv;
        let parts = TimerParts3::new(
            periph.TIMER_0_A3,
            TimerConfig::smclk(&smclk).clk_div(TimerDiv::_2, TimerExDiv::_5),
        );
        let mut timer = parts.timer;
        let mut subtimer = parts.subtimer2;
        let mut regctl = timer.readctl_reg().unwrap();
        print_u16(&mut tx, regctl);

        set_time(&mut timer, &mut subtimer, 1500);
        let mut regctl = timer.readctl_reg().unwrap();
        print_u16(&mut tx, regctl);

        let mybool = Mbool::new(false);
        with(|cs| {
            unsafe { *BLUE_LED.borrow(cs).get() = Some(p3_3)}
            unsafe { *INT_PIN.borrow(cs).get() = Some(p1_2)}
            unsafe { *P1IV.borrow(cs).get() = Some(p1iv)}
            unsafe { *MYBOOL.borrow(cs).get() = Some(mybool)}
        });

        let ce = p2.pin4.to_output();
        let csn = p3.pin0.to_output();
        // ce.set_low().unwrap();
        // csn.set_high().unwrap();
        tx.bwrite_all(b"we have ce csn\r\n").ok();
        let mut nrf24 = NRF24L01::new(ce, csn, spi).unwrap();
        tx.bwrite_all(b"We have nrf\r\n").ok();

        nrf24.set_frequency(90).unwrap();
        tx.bwrite_all(b"We have frequency\r\n").ok();
        nrf24.set_auto_retransmit(0, 0).unwrap();
        nrf24.set_rf(&nrf24::DataRate::R2Mbps, 3).unwrap();
        nrf24
            .set_pipes_rx_lengths(&[Some(32), None, None, None, None, None])
            .unwrap();
        nrf24
            .set_pipes_rx_enable(&[true, false, false, false, false, false])
            .unwrap();
        nrf24.set_auto_ack(&[false; 6]).unwrap();
        nrf24.set_crc(nrf24::CrcMode::Disabled).unwrap();
        nrf24.set_tx_addr(&b"fnord"[..]).unwrap();
        nrf24.set_rx_addr(0, &b"fnord"[..]).unwrap();
        nrf24.set_interrupt_mask(true, true, false).unwrap();

        let mut count = 0u8;

        tx.bwrite_all(b"START\n\r").ok();

        wdt.set_aclk(&aclk)
            .enable_interrupts()
            .start(WdtClkPeriods::DIV15);
        //unsafe { enable_int() };

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
        let mut radiost = RadioState::RadioStandby;
        let mut txpkt = [0x41u8; 32];
        prepare_pkt(&mut txpkt, 0u16);
        let mut nextradiost = RadioState::RadioTx;
        let mut pktcnt = 0;
        txpkt[30] = 0; 
        txpkt[31] = 0;

        loop {
            radiost = nextradiost;

            match radiost {
                RadioState::RadioRx => {
                    let mut nrf24rx = nrf24.rx().unwrap();
                    let mut cnt = 0u16;
                    loop {
                        // block!(subtimer.wait()).void_unwrap();
                        // block!(timer.wait()).void_unwrap();
                        match subtimer.wait() {
                            Ok(_) => {
                                // timeout
                                tx.bwrite_all(b"Timeout rec\n\r").ok();
                                nextradiost = RadioState::RadioTx;
                                set_time(&mut timer, &mut subtimer, 12500);
                                break;
                            },
                            Err(_) =>
                                // not timeout, try to receive
                                if let Ok(Some(pipe)) = nrf24rx.can_read() {
                                    if let Ok(pl) = nrf24rx.read() {
                                        if pl.len() > 0 {
                                            pktcnt =
                                                ((pl.as_ref()[30] as u16) << 8) + (pl.as_ref()[31] as u16);
                                            // print_rec_pkt(&mut tx, pl.as_ref(), pl.len() - 2);
                                            pktcnt = pktcnt + 1;
                                            prepare_pkt(&mut txpkt, pktcnt);
                                            nextradiost = RadioState::RadioTx;
                                            set_time(&mut timer, &mut subtimer, 12500);
                                            break;
                                        }
                                    }
                                },
                        }
                    }
                    nrf24rx.flush_rx().unwrap();
                    nrf24 = nrf24rx.standby();
                }
                RadioState::RadioTx => {
                    let mut nrf24tx = nrf24.tx().unwrap();
                    loop {
                        match timer.wait() {
                            Ok(_) => {
                                // timeout, time to transmit again
                                tx.bwrite_all(b"Tx\n\r").ok();
                                if let Ok(test) = nrf24tx.can_send() {
                                    if test {
                                        // print_snd_pkt(&mut tx, &txpkt, txpkt.len() - 2);
                                        nrf24tx.send(&txpkt).unwrap();
                                        nrf24tx.wait_empty().unwrap();
                                        nrf24tx.clear_interrupts().unwrap();
                                        nextradiost = RadioState::RadioRx;
                                        nrf24 = nrf24tx.standby().unwrap();
                                        set_time(&mut timer, &mut subtimer, 12500);
                                        break;
                                    }
                                }
                            },
                            Err(_) => {
                                // not timeout, do nothing ... or get ch?
                                // delay(15000);
                                // let mut regifg = timer.readctl_reg().unwrap();
                                // print_u16(&mut tx, regifg);
                                // regifg = subtimer.read().unwrap();
                                // print_u16(&mut tx, regifg);
                            },
                        }
                    }
                }
                RadioState::RadioStandby => {}
            }
            let ch = match rx.read() {
                Ok(c) => c,
                Err(err) => {
                    (match err {
                        nb::Error::Other(RecvError::Parity) => '!',
                        nb::Error::Other(RecvError::Overrun(_)) => '}',
                        nb::Error::Other(RecvError::Framing) => '?',
                        nb::Error::WouldBlock => '#',
                    }) as u8
                }
            };
            tx.write(ch);

            if ch == 0x41u8 {
                txpkt[0] = 0x41u8;
                block!(tx.write(ch as u8)).unwrap();
            }
            if ch == 0x61u8 {
                txpkt[0] = 0x61u8;
                block!(tx.write(ch as u8)).unwrap();
            }
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
        }
    } else {
        loop {}
    }
}

#[interrupt]
fn PORT1() {
    with(|cs| {
        if let Some(vector) = unsafe {&mut *P1IV.borrow(cs).get()}.as_mut() {
            match vector.get_interrupt_vector() {
                GpioVector::Pin2Isr => {
                    if let Some(blue_led) = unsafe {
                        &mut *BLUE_LED.borrow(cs).get()
                    } .as_mut() {
                        blue_led.set_low().ok();
                        blue_led.set_high().ok();
                    }
                }
                _ => {},
            }
        }
        if let Some(int_pin) = unsafe {
            &mut *INT_PIN.borrow(cs).get()
        }.as_mut() {
            int_pin.clear_ifg();
        }
        // MYBOOL.borrow(*cs).borrow_mut().as_mut().map(|mybool| {
        //     *mybool = true;
        // });
    });
}

#[interrupt]
fn WDT() {
    with(|cs| {
        // if let Some(blue_led) = unsafe {
        //     &mut *BLUE_LED.borrow(cs).get()
        // } .as_mut() {
        //     blue_led.set_low().ok();
        //     blue_led.set_high().ok();
        // }
    });
}

#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
