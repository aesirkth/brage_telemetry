//! Basic HAL functions for communicating with the radio device

use core::fmt::Debug;

use log::trace;

use crate::{device::*, Error};

/// Hal implementation can be generic over SPI or UART connections
pub trait Hal {
    type CommsError: Debug + 'static;
    type PinError: Debug + 'static;

    /// Reset the device
    fn reset(&mut self) -> Result<(), Error<Self::CommsError, Self::PinError>>;

    /// Fetch radio device busy pin value
    fn get_busy(&mut self) -> Result<bool, Error<Self::CommsError, Self::PinError>>;

    /// Fetch radio device ready / irq (DIO) pin value
    fn get_dio1(&mut self) -> Result<bool, Error<Self::CommsError, Self::PinError>>;

    fn get_dio2(&mut self) -> Result<bool, Error<Self::CommsError, Self::PinError>>;

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Error<Self::CommsError, Self::PinError>>;

    fn wait_dio1(& mut self) -> Result<(), Error<Self::CommsError, Self::PinError>>;

    fn wait_dio2(& mut self) -> Result<(), Error<Self::CommsError, Self::PinError>>;

    /// Delay for the specified time
    fn delay_ms(&mut self, ms: u32);

    /// Delay for the specified time
    fn delay_us(&mut self, us: u32);

    /// Delay for the specified time
    fn delay_ns(&mut self, us: u32);



    fn prefix_read(
        &mut self,
        prefix: &[u8],
        data: &mut [u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>>;

    fn prefix_write(
        &mut self,
        prefix: &[u8],
        data: &[u8],
    ) -> Result<(), Error<Self::CommsError, Self::PinError>>;

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

pub trait HalError {
    type E: Debug;
}

impl<T> HalError for T
where
    T: Hal,
{
    type E = Error<<T as Hal>::CommsError, <T as Hal>::PinError>;
}