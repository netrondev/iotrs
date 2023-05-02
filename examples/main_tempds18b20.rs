#![no_std]
#![no_main]

use embedded_hal::can::Error;
use embedded_hal::digital::v1_compat::OldOutputPin;
#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use arrform::{arrform, ArrForm};
use cortex_m::asm::delay as cycle_delay;
use cortex_m::peripheral::NVIC;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use bsp::hal;
use bsp::pac;
use metro_m0 as bsp;

use bsp::entry;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::usb::UsbBus;

use core::fmt::Debug;
// use core::fmt::{Debug, Write};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use one_wire_bus::OneWire;
use pac::{interrupt, CorePeripherals, Peripherals};

struct OneWirePin {
    state: bool,
    res: Result<(), ()>,
}

impl InputPin for OneWirePin {
    type Error = ();

    fn is_low(&self) -> Result<bool, Self::Error> {
        self.state.map(|v| v == false)
    }
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.state.map(|v| v == true)
    }
}

impl OutputPin for OneWirePin {
    type Error = ();

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        self.res
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        self.res
    }
}

// use cortex_m::prelude::_embedded_hal_digital_OutputPin;

// impl<I, M> _embedded_hal_digital_OutputPin for atsamd_hal::gpio::Pin<I, M> {
//     fn set_low(&mut self) {}
//     fn set_high(&mut self) {}
// }

// struct OldOutputPinImpl {}

// impl v1::OutputPin for OldOutputPinImpl {
//     fn set_low(&mut self) {}
//     fn set_high(&mut self) {}
// }

// struct NewOutputPinConsumer<T: v2::OutputPin> {
//     _pin: T,
// }

// impl<T> NewOutputPinConsumer<T>
// where
//     T: v2::OutputPin,
// {
//     pub fn new(pin: T) -> NewOutputPinConsumer<T> {
//         NewOutputPinConsumer { _pin: pin }
//     }
// }

// struct OneWirePin<T> {
//     pin: T,
// }

// impl<T> OneWirePin<T> {
//     fn new(pin: T) {
//         return { pin };
//     }
// }

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let pins = bsp::Pins::new(peripherals.PORT);
    let mut red_led: bsp::RedLed = pins.d13.into();
    let mut delay = Delay::new(core.SYST, &mut clocks);

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

    // let mut ds18b20pin = pins.d5.as_ref();
    // let mut ds18b20pin: hal::gpio::ReadableOutput = pins.d5.as_mut();

    // let mut ds18b20pin: v2::OutputPin + v2::InputPin = pins.d5.into();
    let mut ds18b20pin: OutputPin = pins.d5.into();

    // let mut onewirepin: v2::OutputPin + v2::InputPin = OneWirePin::new(ds18b20pin).into();

    let mut one_wire_bus = OneWire::new(ds18b20pin).unwrap();

    let mut test: usize = 0;

    let mut delay = Delay::new(core.SYST, &mut clocks);

    loop {
        // delay.delay_ms(100u8);

        // red_led.set_high().unwrap();
        // delay.delay_ms(100u8);

        // red_led.set_low().unwrap();

        cycle_delay(15 * 1024 * 1024);
        red_led.toggle().ok();

        for device_address in one_wire_bus.devices(false, &mut delay) {
            // The search could fail at any time, so check each result. The iterator automatically
            // ends after an error.
            let device_address = device_address.unwrap();

            // The family code can be used to identify the type of device
            // If supported, another crate can be used to interact with that device at the given address
            // writeln!(
            //     tx,
            //     "Found device at address {:?} with family code: {:#x?}",
            //     device_address,
            //     device_address.family_code()
            // )
            // .unwrap();
        }

        test += 1;
    }
}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

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
