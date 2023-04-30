#![no_std]
#![no_main]

#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use bsp::hal;
use bsp::pac;
use metro_m0 as bsp;

use bsp::entry;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use pac::{CorePeripherals, Peripherals};

use core::fmt::{Debug, Write};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use one_wire_bus::OneWire;

fn find_devices<P, E>(delay: &mut impl DelayUs<u16>, tx: &mut impl Write, one_wire_pin: P)
where
    P: OutputPin<Error = E> + InputPin<Error = E>,
    E: Debug,
{
    let mut one_wire_bus = OneWire::new(one_wire_pin).unwrap();
    for device_address in one_wire_bus.devices(false, delay) {
        // The search could fail at any time, so check each result. The iterator automatically
        // ends after an error.
        let device_address = device_address.unwrap();

        // The family code can be used to identify the type of device
        // If supported, another crate can be used to interact with that device at the given address
        writeln!(
            tx,
            "Found device at address {:?} with family code: {:#x?}",
            device_address,
            device_address.family_code()
        )
        .unwrap();
    }
}

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

    // let mut ds18b20pin = pins.d5.as_mut();

    loop {
        delay.delay_ms(100u8);

        red_led.set_high().unwrap();
        delay.delay_ms(100u8);

        red_led.set_low().unwrap();

        // find_devices(delay.delay_ms(100u8), ds18b20pin,)
    }
}
