use embassy_stm32::spi::{Spi, Instance};
use embassy_stm32::gpio::Output;
use embassy_stm32::exti::ExtiInput;
use radio_sx128x::base::Hal;
// Assume Error and PinState are defined or imported from somewhere
use radio_sx128x::Error;

use embassy_time::{with_timeout, TimeoutError};

#[allow(unused_imports)]
use log::{error, warn, info, debug, trace};


pub struct ChungusHal<'d, T: Instance, Tx, Rx> {
    spi: Spi<'d, T, Tx, Rx>,
    cs: Output<'d>,
    busy: ExtiInput<'d>,
    reset: Output<'d>,
    dio1: ExtiInput<'d>,
    dio2: ExtiInput<'d>,
}

impl<'d, T, Tx, Rx> ChungusHal<'d, T, Tx, Rx>
where
    T: Instance,
    Tx: 'd,
    Rx: 'd,
{
    pub fn new(spi: Spi<'d, T, Tx, Rx>, mut cs: Output<'d>, busy: ExtiInput<'d>, mut reset: Output<'d>, dio1: ExtiInput<'d>, dio2: ExtiInput<'d>) -> Self {
        cs.set_high();
        reset.set_high();
        Self {
            spi: spi,
            cs: cs,
            busy: busy,
            reset: reset,
            dio1: dio1,
            dio2: dio2,
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

    async fn reset(&mut self) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        self.reset.set_low();
        self.delay_ms(20).await;
        self.reset.set_high();
        self.delay_ms(20).await;
        self.reset.set_low();
        self.delay_ms(20).await;
        self.reset.set_high();
        self.delay_ms(20).await;
        Ok(())
    }

    async fn get_busy(&mut self) -> Result<bool, Error<Self::CommsError, Self::PinError>> {
        match self.busy.get_level() {
            embassy_stm32::gpio::Level::Low => Ok(false),
            embassy_stm32::gpio::Level::High => Ok(true),
        }
    }

    /// Wait on radio device busy
    async fn wait_busy(&mut self) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        self.busy.wait_for_low().await;
        Ok(())
    }

    /// Fetch radio device ready / irq (DIO) pin value
    async fn get_dio1(&mut self) -> Result<bool, Error<Self::CommsError, Self::PinError>> {
        match self.dio1.get_level() {
            embassy_stm32::gpio::Level::Low => Ok(false),
            embassy_stm32::gpio::Level::High => Ok(true),
        }
    }

    async fn wait_dio1(&mut self, timeout_ms: Option<u64>) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        let fut = self.dio1.wait_for_high();

        match timeout_ms {
            Some(timeout) => {
                match with_timeout(embassy_time::Duration::from_millis(timeout), fut).await {
                    Ok(_) => Ok(()),
                    Err(_) => Err(Error::Timeout)
                }
            },
            None => {
                fut.await;
                Ok(())
            }
        }
    }

    async fn wait_dio2(&mut self, timeout_ms: Option<u64>) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        let fut = self.dio2.wait_for_high();

        match timeout_ms {
            Some(timeout) => {
                match with_timeout(embassy_time::Duration::from_millis(timeout), fut).await {
                    Ok(_) => Ok(()),
                    Err(_) => Err(Error::Timeout)
                }
            },
            None => {
                fut.await;
                Ok(())
            }
        }
    }

    /// Fetch radio device ready / irq (DIO) pin value
    async fn get_dio2(&mut self) -> Result<bool, Error<Self::CommsError, Self::PinError>> {
        match self.dio1.get_level() {
            embassy_stm32::gpio::Level::Low => Ok(false),
            embassy_stm32::gpio::Level::High => Ok(true),
        }
    }

    async fn prefix_read(
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

    async fn prefix_write(
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
    async fn delay_us(&mut self, us: u32) {
        embassy_time::Timer::after_micros(us.into()).await;
    }
}