use embassy_stm32::spi::{Spi, Instance};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::exti::ExtiInput;
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use radio_sx128x::base::Hal;
// Assume Error and PinState are defined or imported from somewhere
use embedded_hal::digital::PinState;
use radio_sx128x::Error;
use radio_sx128x::device::Commands;

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

    fn get_busy(&mut self) -> Result<PinState, Error<Self::CommsError, Self::PinError>> {
        match self.busy.get_level() {
            embassy_stm32::gpio::Level::Low => Ok(PinState::Low),
            embassy_stm32::gpio::Level::High => Ok(PinState::High),
        }
    }

    /// Fetch radio device ready / irq (DIO) pin value
    fn get_dio(&mut self) -> Result<PinState, Error<Self::CommsError, Self::PinError>> {
        match self.dio1_exti.get_level() {
            embassy_stm32::gpio::Level::Low => Ok(PinState::Low),
            embassy_stm32::gpio::Level::High => Ok(PinState::High),
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

    /// Write the specified command and data
    fn write_cmd(
        &mut self,
        command: u8,
        data: &[u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        // Setup register write command
        let out_buf: [u8; 1] = [command];

        trace!("write_cmd cmd: {:02x?} data: {:02x?}", out_buf, data);

        self.wait_busy()?;

        let r = self.prefix_write(&out_buf, data);

        self.wait_busy()?;
        r
    }

    /// Read the specified command and data
    fn read_cmd(
        &mut self,
        command: u8,
        data: &mut [u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        // Setup register read command
        let out_buf: [u8; 2] = [command, 0x00];

        self.wait_busy()?;

        let r = self.prefix_read(&out_buf, data);

        self.wait_busy()?;

        trace!("read_cmd cmd: {:02x?} data: {:02x?}", out_buf, data);

        r
    }

    /// Write to the specified register
    fn write_regs(
        &mut self,
        reg: u16,
        data: &[u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        // Setup register write command
        let out_buf: [u8; 3] = [
            Commands::WiteRegister as u8,
            ((reg & 0xFF00) >> 8) as u8,
            (reg & 0x00FF) as u8,
        ];

        trace!("write_regs cmd: {:02x?} data: {:02x?}", out_buf, data);

        self.wait_busy()?;

        let r = self.prefix_write(&out_buf, data);

        self.wait_busy()?;
        r
    }

    /// Read from the specified register
    fn read_regs(
        &mut self,
        reg: u16,
        data: &mut [u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        // Setup register read command
        let out_buf: [u8; 4] = [
            Commands::ReadRegister as u8,
            ((reg & 0xFF00) >> 8) as u8,
            (reg & 0x00FF) as u8,
            0,
        ];

        self.wait_busy()?;

        let r = self.prefix_read(&out_buf, data);

        self.wait_busy()?;

        trace!("read_regs cmd: {:02x?} data: {:02x?}", out_buf, data);

        r
    }

    /// Write to the specified buffer
    fn write_buff(
        &mut self,
        offset: u8,
        data: &[u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        // Setup register write command
        let out_buf: [u8; 2] = [Commands::WriteBuffer as u8, offset];

        trace!("write_buff cmd: {:02x?}", out_buf);

        self.wait_busy()?;

        let r = self.prefix_write(&out_buf, data);

        self.wait_busy()?;
        r
    }

    /// Read from the specified buffer
    fn read_buff(
        &mut self,
        offset: u8,
        data: &mut [u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        // Setup register read command
        let out_buf: [u8; 3] = [Commands::ReadBuffer as u8, offset, 0];
        trace!(" data: {:02x?}", out_buf);

        self.wait_busy()?;

        let r = self.prefix_read(&out_buf, data);

        self.wait_busy()?;

        trace!("read_buff cmd: {:02x?} data: {:02x?}", out_buf, data);

        r
    }

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        const BUSY_TIMEOUT_MS: u16 = 1000;
        let mut timeout = 0;
        while self.get_busy()? == PinState::High {
            self.delay_ms(1);
            timeout += 1;

            if timeout > BUSY_TIMEOUT_MS {
                error!("Busy timeout after {} ms", BUSY_TIMEOUT_MS);
                return Err(Error::BusyTimeout);
            }
        }

        Ok(())
    }

    /// Read a single u8 value from the specified register
    fn read_reg(&mut self, reg: u16) -> Result<u8, Error<Self::CommsError, Self::PinError>> {
        let mut incoming = [0u8; 1];
        self.read_regs(reg, &mut incoming)?;
        Ok(incoming[0])
    }

    /// Write a single u8 value to the specified register
    fn write_reg(
        &mut self,
        reg: u16,
        value: u8,
    ) -> Result<(), Error<Self::CommsError, Self::PinError>> {
        self.write_regs(reg, &[value])?;
        Ok(())
    }

    /// Update the specified register with the provided value & mask
    fn update_reg(
        &mut self,
        reg: u16,
        mask: u8,
        value: u8,
    ) -> Result<u8, Error<Self::CommsError, Self::PinError>> {
        let existing = self.read_reg(reg)?;
        let updated = (existing & !mask) | (value & mask);
        self.write_reg(reg, updated)?;
        Ok(updated)
    }
}