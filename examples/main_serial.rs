#![no_std]
#![no_main]
// #![feature(restricted_std)]
// use core::fmt::{self, Write};
use arrform::{arrform, ArrForm};

use cortex_m::asm::delay as cycle_delay;
use cortex_m::peripheral::NVIC;
use heapless::String;
use panic_halt as _;

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use bsp::hal;
use bsp::pac;
use metro_m0 as bsp;

use bsp::entry;
use hal::clock::GenericClockController;
use hal::prelude::*;
use hal::usb::UsbBus;
use pac::{interrupt, CorePeripherals, Peripherals};

// macro_rules! format {
//     ($($arg:tt)*) => {{
//         let mut s = String::new();
//         uwrite!(s, "{:?}", pair).unwrap();
//         // let res = ufmt:: core::fmt::Write(s, $($arg)*);
//         let res = ufmt::
//         res
//     }}
// }

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let pins = bsp::Pins::new(peripherals.PORT);
    let mut red_led: bsp::RedLed = pins.d13.into();

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(bsp::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        USB_SERIAL = Some(SerialPort::new(bus_allocator));
        USB_BUS = Some(
            UsbDeviceBuilder::new(bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build(),
        );
    }

    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    // let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    // Flash the LED in a spin loop to demonstrate that USB is
    // entirely interrupt driven.
    let mut test: usize = 0;
    loop {
        cycle_delay(15 * 1024 * 1024);
        red_led.toggle().ok();

        // let mut s = String::new();
        // uwrite!(s, "{:?}", test).unwrap();

        // printSerial(b"Hello world\n");
        // let mut s = format!("Time is now 0x{:08x}", 123);
        let af = arrform!(64, "Hello World {}\n", test);
        printSerial(af.as_bytes());
        // let teststr = format!("hello {}", test);
        test += 1;
    }
}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

fn poll_usb() {
    unsafe {
        if let Some(usb_dev) = USB_BUS.as_mut() {
            if let Some(serial) = USB_SERIAL.as_mut() {
                usb_dev.poll(&mut [serial]);
                let mut buf = [0u8; 64];

                if let Ok(count) = serial.read(&mut buf) {
                    for (i, c) in buf.iter().enumerate() {
                        if i >= count {
                            break;
                        }
                        serial.write("Received: ".as_bytes()).ok();
                        // serial.write(&[c.clone()]).ok();
                        // serial.write("\r\n".as_bytes()).ok();
                        serial.write(&[*c]).ok();
                    }
                };
            };
        }
    };
}

#[interrupt]
fn USB() {
    poll_usb();
}

fn printSerial(input: &[u8]) {
    unsafe {
        if let Some(usb_dev) = USB_BUS.as_mut() {
            if let Some(serial) = USB_SERIAL.as_mut() {
                // serial.write("Idle 4...\n".as_bytes()).ok();
                serial.write(input).ok();
            };
        }
    };
}
