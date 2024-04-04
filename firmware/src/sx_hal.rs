use embassy_stm32::spi::{Spi, Instance};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::exti::ExtiInput;
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use radio_sx128x::base::Hal;
// Assume Error and PinState are defined or imported from somewhere
use radio_sx128x::Error;

use log::{error, warn, info, debug, trace};


pub struct ChungusHal<'d, T: Instance, Tx, Rx> {
    spi: Spi<'d, T, Tx, Rx>,
    cs: Output<'d>,
    busy: Input<'d>,
    reset: Output<'d>,
    dio1_exti: ExtiInput<'d>,
    delay: Delay,
}

impl<'d, T, Tx, Rx> ChungusHal<'d, T, Tx, Rx>
where
    T: Instance,
    Tx: 'd,
    Rx: 'd,
{
    pub fn new(spi: Spi<'d, T, Tx, Rx>, mut cs: Output<'d>, busy: Input<'d>, mut reset: Output<'d>, dio1_exti: ExtiInput<'d>) -> Self {
        cs.set_high();
        reset.set_high();
        Self {
            spi: spi,
            cs: cs,
            busy: busy,
            reset: reset,
            dio1_exti: dio1_exti,
            delay: Delay,
        }
    }
}


impl<'d, T, Tx, Rx> Hal for ChungusHal<'d, T, Tx, Rx>
where
    T: Instance + 'd,  // Ensures T implements Instance and ties its lifetime to 'd
    Tx: 'd,  // Ensures Tx's lifetime is tied to 'd
    Rx: 'd,  // Ensures Rx's lifetime is tied to 'd
{
    type CommsError = ();  // Replace SomeErrorType with the actual error type
    type PinError = ();  // Replace SomePinErrorType with the actual pin error type

    fn reset(&mut self) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        self.reset.set_low();
        self.delay_ms(20);
        self.reset.set_high();
        self.delay_ms(20);
        self.reset.set_low();
        self.delay_ms(20);
        self.reset.set_high();
        self.delay_ms(20);
        Ok(())
    }

    fn get_busy(&mut self) -> Result<bool, Error<Self::CommsError, Self::PinError>> {
        match self.busy.get_level() {
            embassy_stm32::gpio::Level::Low => Ok(false),
            embassy_stm32::gpio::Level::High => Ok(true),
        }
    }

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        const BUSY_TIMEOUT_MS: u16 = 1000;
        let mut timeout = 0;
        while self.get_busy()? == true {
            self.delay_ms(1);
            timeout += 1;

            if timeout > BUSY_TIMEOUT_MS {
                error!("Busy timeout after {} ms", BUSY_TIMEOUT_MS);
                return Err(Error::BusyTimeout);
            }
        }

        Ok(())
    }

    /// Fetch radio device ready / irq (DIO) pin value
    fn get_dio1(&mut self) -> Result<bool, Error<Self::CommsError, Self::PinError>> {
        match self.dio1_exti.get_level() {
            embassy_stm32::gpio::Level::Low => Ok(false),
            embassy_stm32::gpio::Level::High => Ok(true),
        }
    }

    fn wait_dio1(&mut self) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        self.delay_ms(1000);
        Ok(())
    }

    fn wait_dio2(&mut self) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        self.delay_ms(1000);
        Ok(())
    }

    /// Fetch radio device ready / irq (DIO) pin value
    fn get_dio2(&mut self) -> Result<bool, Error<Self::CommsError, Self::PinError>> {
        match self.dio1_exti.get_level() {
            embassy_stm32::gpio::Level::Low => Ok(false),
            embassy_stm32::gpio::Level::High => Ok(true),
        }
    }

    fn prefix_read(
        &mut self,
        prefix: &[u8],
        data: &mut [u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        self.cs.set_low();

        self.spi.blocking_write(&prefix).unwrap();
        self.spi.blocking_read(data).unwrap();
        self.cs.set_high();

        Ok(())
    }

    fn prefix_write(
        &mut self,
        prefix: &[u8],
        data: &[u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        self.cs.set_low();

        self.spi.blocking_write(&prefix).unwrap();
        self.spi.blocking_write(&data).unwrap();
        self.cs.set_high();

        Ok(())
    }

    /// Delay for the specified time
    fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    /// Delay for the specified time
    fn delay_us(&mut self, us: u32) {
        self.delay.delay_us(us);
    }

    /// Delay for the specified time
    fn delay_ns(&mut self, ns: u32) {
        self.delay.delay_us(ns);
    }
}