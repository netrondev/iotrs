#![no_main]
#![no_std]

use arrform::{arrform, ArrForm};

use bsp::hal::usb::UsbBus;
use bsp::usb_allocator;
// use cortex_m::asm::delay as cycle_delay;
use cortex_m::peripheral::NVIC;

use bsp::hal;
use bsp::pac;

use hal::clock::GenericClockController;

use hal::prelude::*;

use hal::delay::Delay;
use metro_m0 as bsp;
use pac::{interrupt, CorePeripherals, Peripherals};

use panic_halt as _;
// use rtt_target::{rprintln, rtt_init_print};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[cortex_m_rt::entry]
fn main() -> ! {
    // rtt_init_print!();
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let pins = bsp::Pins::new(peripherals.PORT);

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(usb_allocator(
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

    let mut test: usize = 0;

    let mut red_led: bsp::RedLed = pins.d13.into();
    let mut delay = Delay::new(core.SYST, &mut clocks);

    loop {
        // rprintln!("Hello, world!");
        delay.delay_ms(200u8);
        red_led.set_high().unwrap();
        delay.delay_ms(200u8);
        red_led.set_low().unwrap();

        let af = arrform!(64, "Hello World {}\n", test);
        print_serial(af.as_bytes());
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

fn print_serial(input: &[u8]) {
    unsafe {
        if let Some(_usb_dev) = USB_BUS.as_mut() {
            if let Some(serial) = USB_SERIAL.as_mut() {
                // serial.write("Idle 4...\n".as_bytes()).ok();
                serial.write(input).ok();
            };
        }
    };
}
