#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::cell::RefCell;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use msp430::asm;
use msp430::interrupt::{enable as enable_int, free, Mutex};
use msp430_rt::entry;
use msp430fr5949::interrupt;
use msp430fr5949_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv, SmclkSel},
    fram::Fram,
    gpio::{Batch, GpioVector, Output, Pin, Pin3, PxIV, P1, P2, P3},
    pmm::Pmm,
    serial::*,
    spi::*,
    watchdog::Wdt,
};
use nb::block;
extern crate display_interface_spi;
extern crate embedded_graphics;
extern crate embedded_hal;
extern crate st7735_lcd;
use core::panic::PanicInfo;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
//use embedded_graphics::style::*;
use embedded_graphics::draw_target::DrawTarget;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use st7735_lcd::Orientation;

// #[cfg(debug_assertions)]
//use panic_msp430 as _;

static BLUE_LED: Mutex<RefCell<Option<Pin<P3, Pin3, Output>>>> = Mutex::new(RefCell::new(None));
static P1IV: Mutex<RefCell<Option<PxIV<P1>>>> = Mutex::new(RefCell::new(None));

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

fn myprint_u8_as_hex(val: u8) -> [u8; 2] {
    const B: &[u8; 16] = b"0123456789abcdef";
    let b0 = B[(val / 16) as usize];
    let b1 = B[(val % 16) as usize];
    [b0, b1]
}

fn myprint_u8_as_dec(val: u8) -> [u8; 3] {
    const B: &[u8; 10] = b"0123456789";
    let b0 = B[(val / 100) as usize];
    let b1 = B[((val / 10) % 10) as usize];
    let b2 = B[(val % 10) as usize];
    [b0, b1, b2]
}

// #[cfg(not(debug_assertions))]
// use panic_never as _;

// Prints "HELLO" when started then echos on UART1
// Serial settings are listed in the code
#[entry]
fn main() -> ! {
    if let Some(periph) = msp430fr5949::Peripherals::take() {
        let mut fram = Fram::new(periph.FRAM);
        let _wdt = Wdt::constrain(periph.WATCHDOG_TIMER);
        // let mut wdt = Wdt::constrain(periph.WATCHDOG_TIMER).to_interval();

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

        delay.delay_us(1000u32);
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
        //
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
        let mut display = st7735_lcd::ST7735::new(spid, dc, rst, false, false, 130, 130);
        // let mut display = st7735_lcd::ST7735::new(spid, dc, rst, false, false, 160, 128);

        let mut p3_1 = p3.pin1;
        let mut p3_3 = p3.pin3;
        p3_1.set_high().unwrap();
        p3_1.set_low().unwrap();
        // p3_1.set_high().unwrap();
        p3_3.set_high().unwrap();
        p3_3.set_low().unwrap();
        p3_3.set_high().unwrap();
        // let p1iv = p1.pxiv;
        //
        free(|cs| *BLUE_LED.borrow(*cs).borrow_mut() = Some(p3_3));
        // free(|cs| *P1IV.borrow(*cs).borrow_mut() = Some(p1iv));

        display.init(&mut delay).unwrap();
        display.set_orientation(&Orientation::Landscape).unwrap();
        display.clear(Rgb565::BLACK).unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
            .draw(&mut display.color_converted())
            .unwrap();

        Text::with_baseline("Hello Rust!", Point::new(0, 12), text_style, Baseline::Top)
            .draw(&mut display.color_converted())
            .unwrap();

        Circle::new(Point::new(64, 64), 64)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE))
            .draw(&mut display)
            .unwrap();

        let mut count = 0u8;

        loop {
            // let bytes = myprint_u8_as_hex(count);
            let bytes = myprint_u8_as_dec(count);
            Rectangle::new(
                Point::new(0, 24),
                Size {
                    width: 17,
                    height: 12,
                },
            )
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(&mut display)
            .unwrap();

            let output = core::str::from_utf8(&bytes).unwrap();
            Text::with_baseline(&output, Point::new(0, 24), text_style, Baseline::Top)
                .draw(&mut display.color_converted())
                .unwrap();
            count += 1;
            delay.delay_ms(1000u32);
        }
    } else {
        loop {}
    }
}

#[interrupt]
fn PORT1() {
    free(|cs| {
        // BLUE_LED.borrow(*cs).borrow_mut().as_mut().map(|blue_led| {
        // match P1IV
        // .borrow(*cs)
        // .borrow_mut()
        // .as_mut()
        // .unwrap()
        // .get_interrupt_vector()
        // {
        // GpioVector::Pin2Isr => {
        // blue_led.set_low().ok();
        // blue_led.set_high().ok()
        // }
        // _ => panic!(),
        // }
        // });
    });
}

#[interrupt]
fn WDT() {
    free(|cs| {
        // BLUE_LED.borrow(*cs).borrow_mut().as_mut().map(|blue_led| {
        // blue_led.set_low().ok();
        // blue_led.set_high().ok();
        // });
    });
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    free(|cs| {
        BLUE_LED.borrow(*cs).borrow_mut().as_mut().map(|blue_led| {
            blue_led.set_low().ok();
        });
    });
    loop {}
}
