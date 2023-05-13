#![no_main]
#![no_std]

// use alloc::boxed::Box;
use arrform::{arrform, ArrForm};

use bsp::hal::usb::UsbBus;
use bsp::usb_allocator;
// use cortex_m::asm::delay as cycle_delay;
use cortex_m::peripheral::NVIC;

use bsp::hal;
use bsp::pac;

use ds18b20::Resolution;
use hal::clock::GenericClockController;

use hal::prelude::*;

use hal::delay::Delay;
use metro_m0 as bsp;
use onewire::OneWire;
use onewire::OneWireError;
use onewire::OneWireResult;
use pac::{interrupt, CorePeripherals, Peripherals};

use panic_halt as _;
// use rtt_target::{rprintln, rtt_init_print};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;

mod onewire;
use core::fmt::Debug;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use crate::ds18b20::Ds18b20;
mod ds18b20;

// struct onewirepin {}

// impl InputPin for onewirepin {
//     fn is_high(&self) -> Result<bool, Self::Error> {}

//     fn is_low(&self) -> Result<bool, Self::Error> {}

//     type Error = Self::Error;
// }

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

    // let mut one_wire_pin = pins.d5.into_readable_output();
    let mut one_wire_pin = pins.d5.into_readable_output();

    let mut one_wire_bus = OneWire::new(one_wire_pin).unwrap();

    loop {
        // rprintln!("Hello, world!");
        delay.delay_ms(200u8);
        red_led.set_high().unwrap();
        delay.delay_ms(200u8);
        red_led.set_low().unwrap();

        let af = arrform!(64, "Hello World {}\n", test);
        print_serial(af.as_bytes());
        test += 1;

        get_temperature(&mut delay, &mut one_wire_bus);
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

fn get_temperature<P, E>(
    delay: &mut (impl DelayUs<u16> + DelayMs<u16>),
    one_wire_bus: &mut OneWire<P>,
) -> OneWireResult<(), E>
where
    P: OutputPin<Error = E> + InputPin<Error = E>,
    E: Debug,
{
    // initiate a temperature measurement for all connected devices
    ds18b20::start_simultaneous_temp_measurement(one_wire_bus, delay)?;

    // wait until the measurement is done. This depends on the resolution you specified
    // If you don't know the resolution, you can obtain it from reading the sensor data,
    // or just wait the longest time, which is the 12-bit resolution (750ms)
    Resolution::Bits12.delay_for_measurement_time(delay);

    // iterate over all the devices, and report their temperature
    let mut search_state = None;
    loop {
        if let Some((device_address, state)) =
            one_wire_bus.device_search(search_state.as_ref(), false, delay)?
        {
            search_state = Some(state);
            if device_address.family_code() != ds18b20::FAMILY_CODE {
                // skip other devices
                continue;
            }
            // You will generally create the sensor once, and save it for later
            let sensor = Ds18b20::new(device_address)?;

            // contains the read temperature, as well as config info such as the resolution used
            let sensor_data = sensor.read_data(one_wire_bus, delay)?;

            let af = arrform!(
                64,
                "Device at {:?} is {}°C",
                device_address,
                sensor_data.temperature
            );
            print_serial(af.as_bytes());

            // writeln!(
            //     tx,
            //     "Device at {:?} is {}°C",
            //     device_address, sensor_data.temperature
            // );
        } else {
            break;
        }
    }
    Ok(())
}
