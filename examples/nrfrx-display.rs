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
    delay::Delay,
    fram::Fram,
    gpio::{Batch, GpioVector, Output, Pin, Pin2, Pin3, PxIV, P1, P2, P3},
    pmm::Pmm,
    serial::*,
    spi::*,
    timer::*,
    watchdog::{Wdt, WdtClkPeriods},
};
use nb::block;
extern crate display_interface_spi;
extern crate embedded_graphics;
extern crate embedded_hal;
extern crate embedded_nrf24l01;
extern crate st7735_lcd;
use core::panic::PanicInfo;
use embedded_nrf24l01 as nrf24;
use embedded_nrf24l01::*;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use st7735_lcd::Orientation;

// #[cfg(debug_assertions)]
//use panic_msp430 as _;

static BLUE_LED: Mutex<UnsafeCell<Option<Pin<P3, Pin3, Output>>>> = Mutex::new(UnsafeCell::new(None));
static P1IV: Mutex<UnsafeCell<Option<PxIV<P1>>>> = Mutex::new(UnsafeCell::new(None));
static INT_PIN: Mutex<UnsafeCell<Option<Pin<P1, Pin2, Input<Pulldown>>>>> =
    Mutex::new(UnsafeCell::new(None));
static MYBOOL: Mutex<UnsafeCell<Option<Mbool>>> = Mutex::new(UnsafeCell::new(None));

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

fn prepare_pkt(pkt: &mut [u8; 32], cnt: u16) {
    pkt[30] = ((cnt >> 8) & 0xFF) as u8;
    pkt[31] = (cnt & 0xff) as u8;
}

static B: &[u8; 16] = b"0123456789abcdef";

fn myprint_u8_as_hex(val: u8) -> [u8; 2] {
    let mut b = [0u8; 2];
    for i in 0..=1 {
        b[i] = B[((val >> ((1 - i)*4)) & 0xF) as usize];
    }
    b
}
fn myprint_u16_as_hex(val: u16) -> [u8; 4] {
    let mut b = [0u8; 4];
    for i in 0..=3 {
        b[i] = B[((val >> ((3 - i)*4)) & 0xF) as usize];
    }
    b
}

fn myprint_u8_as_dec(val: u8) -> [u8; 3] {
    const B: &[u8; 10] = b"0123456789";
    let b0 = B[(val / 100) as usize];
    let b1 = B[((val / 10) % 10) as usize];
    let b2 = B[(val % 10) as usize];
    [b0, b1, b2]
}

// fn display_u8<SPI, DC, RST>(display: &st7735_lcd::ST7735, point: Point, count: u8) {
// let bytes = myprint_u8_as_dec(count);
// Rectangle::new(
// point,
// Size {
// width: 17,
// height: 12,
// },
// )
// .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
// .draw(&mut display)
// .unwrap();
//
// let output = core::str::from_utf8(&bytes).unwrap();
// Text::with_baseline(&output, point, text_style, Baseline::Top)
// .draw(&mut display.color_converted())
// .unwrap();
// }

// fn print_rec_pkt<U: SerialUsci>(tx: &mut Tx<U>, buf: &[u8], len: usize) -> u16 {
//     tx.bwrite_all(b"read = ").unwrap();
//     tx.bwrite_all(&buf[0..len - 1]).unwrap();
//     let cnt: u16 = ((buf[30] as u16) << 8) + (buf[31] as u16);
//     tx.bwrite_all(b" ").unwrap();
//     print_u16(tx, cnt);
//     cnt
// }

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


// #[cfg(not(debug_assertions))]
// use panic_never as _;

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

        let mut delay = Delay::new();

        let p3 = Batch::new(P3 {
            port: periph.PORT_3_4,
        })
        .config_pin1(|p| p.to_output())
        .config_pin3(|p| p.to_output())
        .config_pin5(|p| p.to_output())
        .config_pin6(|p| p.to_output())
        .config_pin7(|p| p.to_output())
        .split(&pmm);

        let p2 = Batch::new(P2 {
            port: periph.PORT_1_2,
        })
        .split(&pmm);

        // let (mut tx, mut rx) = SerialConfig::new(
        // periph.USCI_A1_UART_MODE,
        // BitOrder::LsbFirst,
        // BitCount::EightBits,
        // StopBits::OneStopBit,
        // Parity::NoParity,
        // Loopback::NoLoop,
        // 115200,
        // )
        // .use_smclk(&smclk)
        // .split(p2.pin5.to_alternate2(), p2.pin6.to_alternate2());

        let p = unsafe { msp430fr5949::Peripherals::steal() };
        let p1 = Batch::new(P1 { port: p.PORT_1_2 }).split(&pmm);

        let mode1 = embedded_hal::spi::MODE_0;
        let spid = Spi::spi1(
            periph.USCI_A0_SPI_MODE,
            (
                p1.pin5.to_alternate2(),
                p2.pin1.to_alternate2(),
                p2.pin0.to_alternate2(),
            ),
            mode1,
        );

        let mut dc = p3.pin5;
        let mut rst = p3.pin6;
        let mut csd = p3.pin7;
        dc.set_high().unwrap();
        rst.set_high().unwrap();
        csd.set_high().unwrap();
        csd.set_low().unwrap();
        let mut display = st7735_lcd::ST7735::new(spid, dc, rst, false, false, 160, 128);

        let mode = embedded_hal::spi::MODE_0;
        let spi = Spi::spi0(
            periph.USCI_B0_SPI_MODE,
            (
                p2.pin2.to_alternate2(),
                p1.pin7.to_alternate2(),
                p1.pin6.to_alternate2(),
            ),
            mode,
        );
        let mut p1_2 = p1.pin2.pulldown();
        p1_2.select_falling_edge_trigger().enable_interrupts();

        let mut p3_1 = p3.pin1;
        let mut p3_3 = p3.pin3;
        p3_1.set_high().unwrap();
        p3_1.set_low().unwrap();
        p3_1.set_high().unwrap();
        p3_3.set_high().unwrap();
        p3_3.set_low().unwrap();
        p3_3.set_high().unwrap();
        let p1iv = p1.pxiv;
        let parts = TimerParts3::new(
            periph.TIMER_0_A3,
            TimerConfig::smclk(&smclk).clk_div(TimerDiv::_2, TimerExDiv::_5),
        );
        let mut timer = parts.timer;
        let mut subtimer = parts.subtimer2;

        set_time(&mut timer, &mut subtimer, 35000);

        let mybool = Mbool::new(false);
        with(|cs| {
            unsafe { *BLUE_LED.borrow(cs).get() = Some(p3_3)}
            unsafe { *INT_PIN.borrow(cs).get() = Some(p1_2)}
            unsafe { *P1IV.borrow(cs).get() = Some(p1iv)}
            unsafe { *MYBOOL.borrow(cs).get() = Some(mybool)}
        });

        let mut count = 0u8;
        display.init(&mut delay).unwrap();
        display.set_orientation(&Orientation::Landscape).unwrap();
        display.clear(Rgb565::BLACK).unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        let mut ce = p2.pin4.to_output();
        let mut csn = p3.pin0.to_output();
        ce.set_low().unwrap();
        csn.set_high().unwrap();
        Text::with_baseline("Hello Rust!", Point::zero(), text_style, Baseline::Top)
            .draw(&mut display.color_converted())
            .unwrap();
        // tx.bwrite_all(b"we have ce csn\r\n").ok();
        let mut nrf24 = NRF24L01::new(ce, csn, spi).unwrap();
        // tx.bwrite_all(b"We have nrf\r\n").ok();

        nrf24.set_frequency(90).unwrap();
        Text::with_baseline(
            "We have frequency",
            Point::new(0, 11),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display.color_converted())
        .unwrap();

        // tx.bwrite_all(b"We have frequency\r\n").ok();
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
        nrf24.set_interrupt_mask(false, false, false).unwrap();
        nrf24.clear_interrupts().unwrap();

        wdt.set_aclk(&aclk)
            .enable_interrupts()
            .start(WdtClkPeriods::DIV15);
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
        let mut radiost = RadioState::RadioStandby;
        let mut txpkt = [0x00u8; 32]; //send back OK
        txpkt[0] = 0x4f;
        txpkt[1] = 0x4b;
        let mut nextradiost = RadioState::RadioRx;
        let mut rpktcnt = 0u16;
        let mut npktcnt = 0u16;
        let mut tocnt = 0u16;
        let mut on = false;
        Text::with_baseline(
            "We have nrf start loop",
            Point::new(0, 22),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display.color_converted())
        .unwrap();

        loop {
            radiost = nextradiost;

            match radiost {
                RadioState::RadioRx => {
                    let mut nrf24rx = nrf24.rx().unwrap();
                    let mut cnt = 0u16;
                    loop {
                        match subtimer.wait() {
                            Ok(_) => {
                                // timeout
                                let mut bytes = [0u8; 5];
                                for (place,data) in bytes[1..=4].iter_mut().zip(myprint_u16_as_hex(tocnt).iter()) {
                                    *place = *data;
                                }
                                tocnt = tocnt + 1;
                                Rectangle::new(
                                    Point::new(0, 34),
                                    Size {
                                        width: 30,
                                        height: 12,
                                    },
                                )
                                .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                                .draw(&mut display)
                                .unwrap();
                                bytes[0] = 'T' as u8;

                                let output = core::str::from_utf8(&bytes).unwrap();
                                Text::with_baseline(&output, Point::new(0, 34), text_style, Baseline::Top)
                                .draw(&mut display.color_converted())
                                .unwrap();
                            },
                            Err(_) =>
                                if let Ok(Some(pipe)) = nrf24rx.can_read() {
                                    if let Ok(pl) = nrf24rx.read() {
                                        if pl.len() > 0 {
                                            // pktcnt = print_rec_pkt(&mut tx, pl.as_ref(), pl.len() - 2);
                                            rpktcnt =
                                                ((pl.as_ref()[30] as u16) << 8) + (pl.as_ref()[31] as u16);
                                            txpkt[30] = ((rpktcnt >> 8) & 0xFF) as u8;
                                            txpkt[31] = (rpktcnt & 0xff) as u8;
                                            if pl.as_ref()[0] == 0x41u8 {
                                                on = true;
                                            }
                                            if pl.as_ref()[0] == 0x61u8 {
                                                on = false;
                                            }
                                            nextradiost = RadioState::RadioTx;
                                            // set_time(&mut timer, &mut subtimer, 12500);
                                            break;
                                        }
                                    }
                                }
                        }
                    }
                    nrf24 = nrf24rx.standby();
                }
                RadioState::RadioTx => {
                    let mut bytes = [0u8; 5];
                    for (place,data) in bytes[1..=4].iter_mut().zip(myprint_u16_as_hex(rpktcnt).iter()) {
                        *place = *data;
                    }
                    Rectangle::new(
                        Point::new(0, 34),
                        Size {
                            width: 30,
                            height: 12,
                        },
                    )
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(&mut display)
                    .unwrap();
                    if on {
                        bytes[0] = 'A' as u8;
                    } else {
                        bytes[0] = 'a' as u8;
                    }

                    let output = core::str::from_utf8(&bytes).unwrap();
                    Text::with_baseline(&output, Point::new(0, 34), text_style, Baseline::Top)
                        .draw(&mut display.color_converted())
                        .unwrap();
                    let mut nrf24tx = nrf24.tx().unwrap();
                    loop {
                        if let Ok(test) = nrf24tx.can_send() {
                            if test {
                                //            print_snd_pkt(&mut tx, &txpkt, txpkt.len()-2);
                                nrf24tx.send(&txpkt).unwrap();
                                nrf24tx.wait_empty().unwrap();
                                nrf24tx.clear_interrupts().unwrap();
                                nextradiost = RadioState::RadioRx;
                                nrf24 = nrf24tx.standby().unwrap();
                                break;
                            }
                        }
                    }
                }
                RadioState::RadioStandby => {}
            }
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
        if let Some(blue_led) = unsafe {
            &mut *BLUE_LED.borrow(cs).get()
        } .as_mut() {
            blue_led.set_low().ok();
            blue_led.set_high().ok();
        }
        // MYBOOL.borrow(*cs).borrow_mut().as_mut().map(|mybool| {
        //     mybool.set();
        // });
    });
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    with(|cs| {
        if let Some(blue_led) = unsafe {
            &mut *BLUE_LED.borrow(cs).get()
        } .as_mut() {
            blue_led.set_low().ok();
        }
    });
    panic!();
}

#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
