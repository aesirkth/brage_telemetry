//! Sx128x Radio Driver
// Copyright 2018 Ryan Kurte

#![no_std]
#![feature(associated_type_defaults)]

use core::convert::TryFrom;
use core::fmt::Debug;

#[allow(unused_imports)]
use defmt::{debug, info, warn, error, trace};


pub mod base;

pub mod device;
use device::{lora::LoRaBandwidth, ranging::RangingConfig, *};
pub use device::{Config, State};

pub mod prelude;


/// Sx128x device object
pub struct Sx128x<Base> {
    config: Config,
    packet_type: PacketType,
    hal: Base,
}

pub const FREQ_MIN: u32 = 2_400_000_000;
pub const FREQ_MAX: u32 = 2_500_000_000;

pub const NUM_RETRIES: usize = 3;

/// Sx128x error type
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "thiserror", derive(thiserror::Error))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<CommsError: Debug + 'static, PinError: Debug + 'static> {
    #[cfg_attr(feature = "thiserror", error("communication error: {:?}", 0))]
    /// Communications (SPI or UART) error
    Comms(CommsError),

    #[cfg_attr(feature = "thiserror", error("pin error: {:?}", 0))]
    /// Pin control error
    Pin(PinError),

    #[cfg_attr(feature = "thiserror", error("transaction aborted"))]
    /// Transaction aborted
    Aborted,

    #[cfg_attr(feature = "thiserror", error("transaction timeout"))]
    /// Timeout by device
    Timeout,

    #[cfg_attr(feature = "thiserror", error("busy timeout"))]
    /// Timeout awaiting busy pin de-assert
    BusyTimeout,

    #[cfg_attr(feature = "thiserror", error("invalid message CRC"))]
    /// CRC error on received message
    InvalidCrc,

    #[cfg_attr(feature = "thiserror", error("invalid message length"))]
    /// Invalid message length
    InvalidLength,

    #[cfg_attr(feature = "thiserror", error("invalid sync word"))]
    /// TODO
    InvalidSync,

    #[cfg_attr(feature = "thiserror", error("transaction aborted"))]
    /// TODO
    Abort,

    #[cfg_attr(
        feature = "thiserror",
        error("invalid state (expected {:?} actual {:?})", 0, 1)
    )]
    /// TODO
    InvalidState(State, State),

    #[cfg_attr(
        feature = "thiserror",
        error("invalid device version (received {:?})", 0)
    )]
    /// Radio returned an invalid device firmware version
    InvalidDevice(u16),

    #[cfg_attr(
        feature = "thiserror",
        error("invalid circuit state (received {:?})", 0)
    )]
    /// Radio returned an invalid response
    InvalidCircuitState(u8),

    #[cfg_attr(
        feature = "thiserror",
        error("invalid command state (received {:?})", 0)
    )]
    /// Radio returned an invalid response
    InvalidCommandStatus(u8),

    #[cfg_attr(feature = "thiserror", error("invalid configuration"))]
    /// Invalid configuration option provided
    InvalidConfiguration,

    #[cfg_attr(feature = "thiserror", error("invalid state command"))]
    /// Invalid state command
    InvalidStateCommand,

    #[cfg_attr(
        feature = "thiserror",
        error("invalid frequency or frequency out of range")
    )]
    /// Frequency out of range
    InvalidFrequency,

    #[cfg_attr(feature = "thiserror", error("device communication failed"))]
    /// No SPI communication detected
    NoComms,
}

impl<Hal> Sx128x<Hal>
where
    Hal: base::Hal,
    <Hal as base::Hal>::CommsError: Debug + 'static,
    <Hal as base::Hal>::PinError: Debug + 'static,
{
    /// Create a new Sx128x instance over a generic Hal implementation
    pub async fn new(
        hal: Hal,
        config: &Config,
    ) -> Result<Self, Error<<Hal as base::Hal>::CommsError, <Hal as base::Hal>::PinError>> {
        let mut sx128x = Self::build(hal).await;
        sx128x.hal.delay_ms(100).await;


        debug!("Resetting device");

        // Reset IC
        sx128x.hal.reset().await?;

        debug!("Checking firmware version");

        // Check communication with the radio
        let firmware_version = sx128x.firmware_version().await?;

        if firmware_version == 0xFFFF || firmware_version == 0x0000 {
            return Err(Error::NoComms);
        } else if firmware_version != 0xA9B5 {
            warn!(
                "Invalid firmware version! expected: 0x{:x} actual: 0x{:x}",
                0xA9B5, firmware_version
            );
        }

        if firmware_version != 0xA9B5 && !config.skip_version_check {
            // Disable version check. Known F/W version is: 0x8E8E
            return Err(Error::InvalidDevice(firmware_version));
        }

        // TODO: do we need to calibrate things here?
        sx128x.calibrate(CalibrationParams::all()).await?;

        debug!("Configuring device");

        // Configure device prior to use
        sx128x.configure(config).await?;

        // Ensure state is idle
        sx128x.set_state(State::StandbyRc).await?;

        // Enable IRQs
        let irqs = Irq::RX_DONE
        | Irq::CRC_ERROR
        | Irq::RX_TX_TIMEOUT
        | Irq::SYNCWORD_ERROR
        | Irq::HEADER_ERROR
        | Irq::RANGING_MASTER_RESULT_TIMEOUT
        | Irq::RANGING_MASTER_RESULT_VALID;

        let dio1_irqs = Irq::TX_DONE
        | Irq::RX_TX_TIMEOUT;

        let dio2_irqs = Irq::RX_DONE
        | Irq::CRC_ERROR
        | Irq::RX_TX_TIMEOUT
        | Irq::SYNCWORD_ERROR
        | Irq::HEADER_ERROR;


        let dio3_irqs = Irq::RANGING_MASTER_RESULT_TIMEOUT
        | Irq::RANGING_MASTER_RESULT_VALID;

        sx128x.set_irq_dio_mask(irqs, dio1_irqs, dio2_irqs, dio3_irqs).await?;

        Ok(sx128x)
    }

    pub async fn reset(&mut self) -> Result<(), <Hal as base::HalError>::E> {
        debug!("Resetting device");

        self.hal.reset().await?;

        Ok(())
    }

    pub(crate) async fn build(hal: Hal) -> Self {
        Sx128x {
            config: Config::default(),
            packet_type: PacketType::None,
            hal,
        }
    }

    pub async fn configure(&mut self, config: &Config) -> Result<(), <Hal as base::HalError>::E> {
        // Switch to standby mode
        self.set_state(State::StandbyRc).await?;

        // Check configs match
        match (&config.modem, &config.channel) {
            (Modem::LoRa(_), Channel::LoRa(_)) => (),
            (Modem::Flrc(_), Channel::Flrc(_)) => (),
            (Modem::Gfsk(_), Channel::Gfsk(_)) => (),
            (Modem::Ranging(_), Channel::Ranging(_)) => (),
            _ => return Err(Error::InvalidConfiguration),
        }

        debug!("configuring regulator");
        // Update regulator mode
        self.set_regulator_mode(config.regulator_mode).await?;
        self.config.regulator_mode = config.regulator_mode;

        debug!("configuring channel");
        // Update modem and channel configuration
        self.set_channel(&config.channel).await?;
        self.config.channel = config.channel.clone();

        debug!("configuring modem");
        self.configure_modem(&config.modem).await?;
        self.config.modem = config.modem.clone();

        debug!("configuring pa");
        // Update power amplifier configuration
        self.set_power_ramp(config.pa_config.power, config.pa_config.ramp_time).await?;
        self.config.pa_config = config.pa_config.clone();

        debug!("configuring ranging");
        self.configure_ranging(&config.ranging).await?;

        Ok(())
    }



    pub async fn firmware_version(&mut self) -> Result<u16, <Hal as base::HalError>::E> {
        let mut d = [0u8; 2];

        self.hal
            .read_regs(Registers::LrFirmwareVersionMsb as u16, &mut d).await?;

        Ok((d[0] as u16) << 8 | (d[1] as u16))
    }

    pub async fn set_frequency(&mut self, f: u32) -> Result<(), <Hal as base::HalError>::E> {
        let c = self.config.freq_to_steps(f as f32) as u32;

        trace!("Setting frequency ({:?} MHz, {} index)", f / 1000 / 1000, c);

        let data: [u8; 3] = [(c >> 16) as u8, (c >> 8) as u8, c as u8];

        self.hal.write_cmd(Commands::SetRfFrequency as u8, &data).await
    }

    pub(crate) async fn set_power_ramp(
        &mut self,
        power: i8,
        ramp: RampTime,
    ) -> Result<(), <Hal as base::HalError>::E> {
        if !(-18..=13).contains(&power) {
            warn!("TX power out of range (-18 < p < 13)");
        }

        // Limit to -18 to +13 dBm
        let power = core::cmp::max(power, -18);
        let power = core::cmp::min(power, 13);
        let power_reg = (power + 18) as u8;

        trace!(
            "Setting TX power to {} dBm {:?} ramp ({}, {})",
            power,
            ramp,
            power_reg,
            ramp as u8
        );
        self.config.pa_config.power = power;
        self.config.pa_config.ramp_time = ramp;

        self.hal
            .write_cmd(Commands::SetTxParams as u8, &[power_reg, ramp as u8]).await
    }

    /// Set IRQ mask
    pub async fn set_irq_mask(&mut self, irq: Irq) -> Result<(), <Hal as base::HalError>::E> {
        trace!("Setting IRQ mask: {:?}", irq);

        let raw = irq.bits();
        self.hal.write_cmd(
            Commands::SetDioIrqParams as u8,
            &[(raw >> 8) as u8, (raw & 0xff) as u8],
        ).await
    }

    /// Set the IRQ and DIO masks
    pub async fn set_irq_dio_mask(
        &mut self,
        irq: Irq,
        dio1: DioMask,
        dio2: DioMask,
        dio3: DioMask,
    ) -> Result<(), <Hal as base::HalError>::E> {
        trace!(
            "Setting IRQ mask: {:?} DIOs: {:?}, {:?}, {:?}",
            irq,
            dio1,
            dio2,
            dio3
        );

        let raw_irq = irq.bits();
        let raw_dio1 = dio1.bits();
        let raw_dio2 = dio2.bits();
        let raw_dio3 = dio3.bits();

        let data = [
            (raw_irq >> 8) as u8,
            (raw_irq & 0xff) as u8,
            (raw_dio1 >> 8) as u8,
            (raw_dio1 & 0xff) as u8,
            (raw_dio2 >> 8) as u8,
            (raw_dio2 & 0xff) as u8,
            (raw_dio3 >> 8) as u8,
            (raw_dio3 & 0xff) as u8,
        ];

        self.hal.write_cmd(Commands::SetDioIrqParams as u8, &data).await
    }

    pub(crate) async fn configure_modem(
        &mut self,
        config: &Modem,
    ) -> Result<(), <Hal as base::HalError>::E> {
        use Modem::*;

        debug!("Setting modem config: {:?}", config);

        // First update packet type (if required)
        let packet_type = PacketType::from(config);
        if self.packet_type != packet_type {
            trace!("Setting packet type: {:?}", packet_type);
            self.hal
                .write_cmd(Commands::SetPacketType as u8, &[packet_type as u8]).await?;
            self.packet_type = packet_type;
        }

        let data = match config {
            Gfsk(c) => [
                c.preamble_length as u8,
                c.sync_word_length as u8,
                c.sync_word_match as u8,
                c.header_type as u8,
                c.payload_length,
                c.crc_mode as u8,
                c.whitening as u8,
            ],
            LoRa(c) | Ranging(c) => [
                c.preamble_length,
                c.header_type as u8,
                c.payload_length,
                c.crc_mode as u8,
                c.invert_iq as u8,
                0u8,
                0u8,
            ],
            Flrc(c) => [
                c.preamble_length as u8,
                c.sync_word_length as u8,
                c.sync_word_match as u8,
                c.header_type as u8,
                c.payload_length,
                c.crc_mode as u8,
                c.whitening as u8,
            ],
            Ble(c) => [
                c.connection_state as u8,
                c.crc_field as u8,
                c.packet_type as u8,
                c.whitening as u8,
                0u8,
                0u8,
                0u8,
            ],
            None => [0u8; 7],
        };

        self.hal.write_cmd(Commands::SetPacketParams as u8, &data).await?;

        // Apply patches
        match config {
            Flrc(c) if c.patch_syncword => {
                // Apply sync-word patch for FLRC mode
                self.patch_flrc_syncword().await?;
            }
            Gfsk(c) if c.patch_preamble => {
                // Write preamble length for GFSK mode
                self.hal.write_reg(
                    Registers::GfskBlePreambleLength as u16,
                    c.preamble_length as u8,
                ).await?;
            }
            _ => (),
        }

        Ok(())
    }

    pub async fn set_ranging_target(&mut self, target_id: u32) -> Result<(), <Hal as base::HalError>::E>{
        self.hal.write_regs(Registers::LrRequestRangingAddr as u16, &target_id.to_be_bytes()).await?;
        Ok(())
    }

    pub async fn configure_ranging(&mut self,
        conf: &RangingConfig
    ) -> Result<(), <Hal as base::HalError>::E> {
        self.hal.write_reg(Registers::LrRangingIdCheckLength as u16, conf.id_length as u8).await?;
        self.hal.write_regs(Registers::LrDeviceRangingAddr as u16, &conf.device_id.to_be_bytes()).await?;
        self.hal.write_reg(Registers::LrRangingResultConfig as u16, conf.result_type as u8).await?;
        match conf.filter_size {
            Some(size) => self.hal.write_reg(Registers::LrRangingFilterWindowSize as u16, size).await?,
            None => (),
        };
        self.hal.write_regs(Registers::LrRangingReRxTxDelayCal as u16, &conf.calibration_value.to_be_bytes()).await?;

        Ok(())
    }

    pub(crate) async fn get_rx_buffer_status(&mut self) -> Result<(u8, u8), <Hal as base::HalError>::E> {
        use device::lora::LoRaHeader;

        let mut status = [0u8; 2];

        self.hal
            .read_cmd(Commands::GetRxBufferStatus as u8, &mut status).await?;

        let len = match &self.config.modem {
            Modem::LoRa(c) => match c.header_type {
                LoRaHeader::Implicit => self.hal.read_reg(Registers::LrPayloadLength as u16).await?,
                LoRaHeader::Explicit => status[0],
            },
            // BLE status[0] does not include 2-byte PDU header
            Modem::Ble(_) => status[0] + 2,
            _ => status[0],
        };

        let rx_buff_ptr = status[1];

        trace!("RX buffer ptr: {} len: {}", rx_buff_ptr, len);

        Ok((rx_buff_ptr, len))
    }

    pub(crate) async fn get_packet_info(
        &mut self,
        info: &mut PacketInfo,
    ) -> Result<(), <Hal as base::HalError>::E> {
        let mut data = [0u8; 5];
        self.hal
            .read_cmd(Commands::GetPacketStatus as u8, &mut data).await?;

        info.packet_status = PacketStatus::from_bits_truncate(data[2]);
        info.tx_rx_status = TxRxStatus::from_bits_truncate(data[3]);
        info.sync_addr_status = data[4] & 0b0111;

        match self.packet_type {
            PacketType::Gfsk | PacketType::Flrc | PacketType::Ble => {
                info.rssi = -(data[1] as i16) / 2;
                let rssi_avg = -(data[0] as i16) / 2;
                trace!("Raw RSSI: {}", info.rssi);
                trace!("Average RSSI: {}", rssi_avg);
            }
            PacketType::LoRa | PacketType::Ranging => {
                info.rssi = -(data[0] as i16) / 2;
                info.snr = Some(match data[1] < 128 {
                    true => data[1] as i16 / 4,
                    false => (data[1] as i16 - 256) / 4,
                });
            }
            PacketType::None => unimplemented!(),
        }

        debug!("Info: {:?}", info);

        Ok(())
    }

    pub async fn calibrate(&mut self, c: CalibrationParams) -> Result<(), <Hal as base::HalError>::E> {
        trace!("Calibrate {:?}", c);
        self.hal.write_cmd(Commands::Calibrate as u8, &[c.bits()]).await
    }

    pub(crate) async fn set_regulator_mode(
        &mut self,
        r: RegulatorMode,
    ) -> Result<(), <Hal as base::HalError>::E> {
        trace!("Set regulator mode {:?}", r);
        self.hal
            .write_cmd(Commands::SetRegulatorMode as u8, &[r as u8]).await
    }

    // TODO: this could got into a mode config object maybe?
    #[allow(dead_code)]
    pub(crate) async fn set_auto_tx(&mut self, a: AutoTx) -> Result<(), <Hal as base::HalError>::E> {
        let data = match a {
            AutoTx::Enabled(timeout_us) => {
                let compensated = timeout_us - AUTO_RX_TX_OFFSET;
                [(compensated >> 8) as u8, (compensated & 0xff) as u8]
            }
            AutoTx::Disabled => [0u8; 2],
        };
        self.hal.write_cmd(Commands::SetAutoTx as u8, &data).await
    }

    pub(crate) async fn set_buff_base_addr(
        &mut self,
        tx: u8,
        rx: u8,
    ) -> Result<(), <Hal as base::HalError>::E> {
        trace!("Set buff base address (tx: {}, rx: {})", tx, rx);
        self.hal
            .write_cmd(Commands::SetBufferBaseAddress as u8, &[tx, rx]).await
    }

    /// Set the sychronization mode for a given index (1-3).
    /// This is 5-bytes for GFSK mode and 4-bytes for FLRC and BLE modes.
    pub async fn set_syncword(
        &mut self,
        index: u8,
        value: &[u8],
    ) -> Result<(), <Hal as base::HalError>::E> {
        trace!(
            "Attempting to set sync word index: {} to: {:?}",
            index,
            value
        );

        // Check sync words for errata 16.4
        if self.packet_type == PacketType::Flrc {
            match &value[0..2] {
                &[0x8C, 0x32] | &[0x63, 0x0E] => {
                    error!("Invalid sync word selected (see errata 16.4)");
                    return Err(Error::InvalidConfiguration);
                }
                _ => (),
            }
        }

        // Calculate sync word base address and expected length
        let (addr, len) = match (&self.packet_type, index) {
            (PacketType::Gfsk, 1) => (Registers::LrSyncWordBaseAddress1 as u16, 5),
            (PacketType::Gfsk, 2) => (Registers::LrSyncWordBaseAddress2 as u16, 5),
            (PacketType::Gfsk, 3) => (Registers::LrSyncWordBaseAddress3 as u16, 5),
            (PacketType::Flrc, 1) => (Registers::LrSyncWordBaseAddress1 as u16 + 1, 4),
            (PacketType::Flrc, 2) => (Registers::LrSyncWordBaseAddress2 as u16 + 1, 4),
            (PacketType::Flrc, 3) => (Registers::LrSyncWordBaseAddress3 as u16 + 1, 4),
            (PacketType::Ble, _) => (Registers::LrSyncWordBaseAddress1 as u16 + 1, 4),
            _ => {
                warn!(
                    "Invalid sync word configuration (mode: {:?} index: {} value: {:?}",
                    self.config.modem, index, value
                );
                return Err(Error::InvalidConfiguration);
            }
        };

        // Check length is correct
        if value.len() != len {
            warn!(
                "Incorrect sync word length for mode: {:?} (actual: {}, expected: {})",
                self.config.modem,
                value.len(),
                len
            );
            return Err(Error::InvalidConfiguration);
        }

        // Write sync word
        self.hal.write_regs(addr, value).await?;

        Ok(())
    }

    /// Apply patch for sync-word match errata in FLRC mode
    async fn patch_flrc_syncword(&mut self) -> Result<(), <Hal as base::HalError>::E> {
        // If we're in FLRC mode, patch to force 100% match on syncwords
        // because otherwise the 4 bit threshold is too low
        if let PacketType::Flrc = &self.packet_type {
            let r = self.hal.read_reg(Registers::LrSyncWordTolerance as u16).await?;
            self.hal
                .write_reg(Registers::LrSyncWordTolerance as u16, r & 0xF0).await?;
        }

        Ok(())
    }

    /// Fetch device state
    pub async fn get_state(&mut self) -> Result<State, <Hal as base::HalError>::E> {
        let mut d = [0u8; 1];
        self.hal.read_cmd(Commands::GetStatus as u8, &mut d).await?;

        trace!("raw state: {}", d[0]);

        let mode = (d[0] & 0b1110_0000) >> 5;
        let m = State::try_from(mode).map_err(|_| Error::InvalidCircuitState(d[0]))?;

        let status = (d[0] & 0b0001_1100) >> 2;
        let s = CommandStatus::try_from(status).map_err(|_| Error::InvalidCommandStatus(d[0]))?;

        trace!("get state: {:?} status: {:?}", m, s);

        Ok(m)
    }

    /// Set device state
    pub async fn set_state(&mut self, state: State) -> Result<(), <Hal as base::HalError>::E> {
        let command = match state {
            State::Tx => Commands::SetTx,
            State::Rx => Commands::SetRx,
            // State::Cad => Commands::SetCad,
            State::Fs => Commands::SetFs,
            State::StandbyRc | State::StandbyXosc => Commands::SetStandby,
            State::Sleep => Commands::SetSleep,
            #[cfg(feature = "patch-unknown-state")]
            State::Unknown => return Err(Error::InvalidStateCommand),
        };

        trace!("Setting state {:?} ({})", state, command);

        self.hal.write_cmd(command as u8, &[0u8]).await
    }

    /// Fetch device state
    pub async fn is_busy(&mut self) -> Result<bool, <Hal as base::HalError>::E> {
        let irq = self.get_interrupts(false).await?;

        if irq.contains(Irq::SYNCWORD_VALID)
            && !(irq.contains(Irq::RX_DONE) || irq.contains(Irq::CRC_ERROR))
        {
            return Ok(true);
        }

        Ok(false)
    }

    /// Set operating channel
    pub async fn set_channel(&mut self, ch: &Channel) -> Result<(), <Hal as base::HalError>::E> {
        use Channel::*;

        debug!("Setting channel config: {:?}", ch);

        // Set frequency
        let freq = ch.frequency();
        if !(FREQ_MIN..=FREQ_MAX).contains(&freq) {
            return Err(Error::InvalidFrequency);
        }

        self.set_frequency(freq).await?;

        // First update packet type (if required)
        let packet_type = PacketType::from(ch);
        if self.packet_type != packet_type {
            self.hal
                .write_cmd(Commands::SetPacketType as u8, &[packet_type as u8]).await?;
            self.packet_type = packet_type;
        }

        // Then write modulation configuration
        let data = match ch {
            Gfsk(c) => [c.br_bw as u8, c.mi as u8, c.ms as u8],
            LoRa(c) | Ranging(c) => [c.sf as u8, c.bw as u8, c.cr as u8],
            Flrc(c) => [c.br_bw as u8, c.cr as u8, c.ms as u8],
            Ble(c) => [c.br_bw as u8, c.mi as u8, c.ms as u8],
        };

        self.hal
            .write_cmd(Commands::SetModulationParams as u8, &data).await
    }

    pub async fn set_power(&mut self, power: i8) -> Result<(), <Hal as base::HalError>::E> {
        let ramp_time = self.config.pa_config.ramp_time;
        self.set_power_ramp(power, ramp_time).await
    }

    /// Fetch (and optionally clear) current interrupts
    pub async fn get_interrupts(&mut self, clear: bool) -> Result<Irq, <Hal as base::HalError>::E> {
        let mut data = [0u8; 2];

        self.hal.read_cmd(Commands::GetIrqStatus as u8, &mut data).await?;
        let irq = Irq::from_bits((data[0] as u16) << 8 | data[1] as u16).unwrap();

        if clear && !irq.is_empty() {
            self.hal.write_cmd(Commands::ClearIrqStatus as u8, &data).await?;
        }

        if !irq.is_empty() {
            trace!("irq: {:?}", irq);
        }

        Ok(irq)
    }

    /// Start transmitting a packet
    pub async fn start_transmit(&mut self, data: &[u8]) -> Result<(), <Hal as base::HalError>::E> {
        debug!("TX start");

        // Set state to idle before we write configuration
        self.set_state(State::StandbyRc).await?;

        // let s = self.get_state().await?;
        // debug!("TX setup state: {:?}", s);

        // Set packet mode
        let mut modem_config = self.config.modem.clone();
        modem_config.set_payload_len(data.len() as u8);

        if let Err(e) = self.configure_modem(&modem_config).await {
            if let Ok(s) = self.get_state().await {
                error!("TX error setting modem (state: {:?})", s);
            } else {
                error!("TX error setting modem",);
            }
            return Err(e);
        }


        // Reset buffer addr
        if let Err(e) = self.set_buff_base_addr(0, data.len() as u8).await {
            if let Ok(s) = self.get_state().await {
                error!("TX error setting buffer base addr (state: {:?})", s);
            } else {
                error!("TX error setting buffer base addr",);
            }

            return Err(e);
        }

        // Write data to be sent
        debug!("TX data: {:?}", data);
        self.hal.write_buff(0, data).await?;

        // Configure ranging if used
        // handle in separate function instead
        if PacketType::Ranging == self.packet_type {
            self.hal.write_cmd(
                Commands::SetRangingRole as u8,
                &[RangingRole::Initiator as u8],
            ).await?;
        }

        // Setup timout
        let config = [
            self.config.rf_timeout.step() as u8,
            ((self.config.rf_timeout.count() >> 8) & 0x00FF) as u8,
            (self.config.rf_timeout.count() & 0x00FF) as u8,
        ];

        // Enable IRQs
        let irq1 = Irq::TX_DONE
            | Irq::RX_TX_TIMEOUT;

        // Enable IRQs
        let irq2 = Irq::RANGING_MASTER_RESULT_TIMEOUT
            | Irq::RANGING_MASTER_RESULT_VALID;

        let irqs = irq1 | irq2;
        self.set_irq_dio_mask(irqs, irqs, irq2, DioMask::empty()).await?;

        // Enter transmit mode
        self.hal.write_cmd(Commands::SetTx as u8, &config).await?;

        trace!("TX start issued");

        // let state = self.get_state().await?;
        // trace!("State: {:?}", state);

        Ok(())
    }

    pub async fn wait_transmit_done(&mut self, timeout_ms: Option<u64>) -> Result<(), <Hal as base::HalError>::E> {
        self.hal.wait_dio1(timeout_ms).await
    }

    pub async fn wait_ranging_done(&mut self, timeout_ms: Option<u64>) -> Result<(), <Hal as base::HalError>::E> {
        self.hal.wait_dio2(timeout_ms).await
    }

    pub async fn check_ranging(&mut self) -> Result<bool, <Hal as base::HalError>::E> {
        let irq = self.get_interrupts(true).await?;

        if irq.contains(Irq::RANGING_MASTER_RESULT_VALID) {
            Ok(true)
        } else if irq.contains(Irq::RANGING_MASTER_RESULT_TIMEOUT) {
            Err(self::Error::Timeout)
        } else {
            Ok(false)
        }
    }

    pub async fn get_ranging_result(&mut self) -> Result<f32, <Hal as base::HalError>::E> {

        // do this weird maneuver from the datasheet first
        self.set_state(State::StandbyXosc).await?;
        let reg = self.hal.read_reg(Registers::LrRangingResultsFreeze as u16).await?;
        self.hal.write_reg(Registers::LrRangingResultsFreeze as u16, reg | (1 << 1)).await?;

        // maybe set the mux again??

        let mut rx_buf: [u8; 3] = [0; 3];
        self.hal.read_regs(Registers::LrRangingResultBaseAddr as u16, &mut rx_buf).await?;
        let distance_raw: u32 = (rx_buf[0] as u32) + (rx_buf[1] as u32) << 8 + (rx_buf[2] as u32) << 16;

        let bw: f32 = if let Channel::LoRa(channel) = &self.config.channel {
            match channel.bw {
                LoRaBandwidth::Bw400kHz => 0.4,
                LoRaBandwidth::Bw800kHz => 0.8,
                LoRaBandwidth::Bw1600kHz => 1.6,
                _ => {
                    return Err(self::Error::InvalidConfiguration)
                }
            }
        } else {
            return Err(self::Error::InvalidConfiguration);
        };

        let distance_m: f32 = (distance_raw as f32) * 150.0 / (4096.0 * bw);
        Ok(distance_m)
    }

    /// Check for transmit completion
    pub async fn check_transmit(&mut self) -> Result<bool, <Hal as base::HalError>::E> {
        // Poll on DIO and short-circuit if not asserted
        // if self.hal.get_dio1().await? == false {
        //     return Ok(false);
        // }

        let irq = self.get_interrupts(true).await?;
        // let state = self.get_state().await?;

        // trace!("TX poll (irq: {:?}, state: {:?})", irq, state);

        if irq.contains(Irq::TX_DONE) {
            debug!("TX complete");
            Ok(true)
        } else if irq.contains(Irq::RX_TX_TIMEOUT) {
            debug!("TX timeout");
            Err(Error::Timeout)
        } else {
            Ok(false)
        }
    }

    /// Stutort radio in receive mode
    pub async fn start_receive(&mut self) -> Result<(), <Hal as base::HalError>::E> {
        debug!("RX start");

        // Set state to idle before we write configuration
        self.set_state(State::StandbyRc).await?;

        // let s = self.get_state().await?;
        // debug!("RX setup state: {:?}", s);

        // Reset buffer addr
        if let Err(e) = self.set_buff_base_addr(0, 0).await {
            if let Ok(s) = self.get_state().await {
                error!("RX error setting buffer base addr (state: {:?})", s);
            } else {
                error!("RX error setting buffer base addr",);
            }
            return Err(e);
        }

        // Set packet mode
        // TODO: surely this should not bre required _every_ receive?
        let modem_config = self.config.modem.clone();

        if let Err(e) = self.configure_modem(&modem_config).await {
            if let Ok(s) = self.get_state().await {
                error!("RX error setting configuration (state: {:?})", s);
            } else {
                error!("RX error setting configuration",);
            }
            return Err(e);
        }

        // Configure ranging if used
        if PacketType::Ranging == self.packet_type {
            self.hal.write_cmd(
                Commands::SetRangingRole as u8,
                &[RangingRole::Responder as u8],
            ).await?;
        }

        // Setup timout
        let config = [
            self.config.rf_timeout.step() as u8,
            ((self.config.rf_timeout.count() >> 8) & 0x00FF) as u8,
            (self.config.rf_timeout.count() & 0x00FF) as u8,
        ];

        // Enable IRQs
        let irq1 = Irq::RX_DONE
            | Irq::CRC_ERROR
            | Irq::RX_TX_TIMEOUT
            | Irq::SYNCWORD_ERROR
            | Irq::HEADER_ERROR;

        let irq2 = Irq::RANGING_SLAVE_REQUEST_VALID;

        let irq3 = Irq::RANGING_SLAVE_RESPONSE_DONE;

        let irqs = irq1 | irq2 | irq3;

        self.set_irq_dio_mask(irqs, irq1, irq2, irq3).await?;

        // Enter transmit mode
        self.hal.write_cmd(Commands::SetRx as u8, &config).await?;

        Ok(())
    }

    pub async fn is_responding(&mut self) -> Result<bool, <Hal as base::HalError>::E>{
        self.hal.get_dio2().await
    }

    pub async fn wait_respond_done(&mut self, timeout_ms: Option<u64>) -> Result<(), <Hal as base::HalError>::E>{
        self.hal.wait_dio3(timeout_ms).await
    }

    pub async fn wait_receive_done(&mut self, timeout_ms: Option<u64>) -> Result<(), <Hal as base::HalError>::E> {
        self.hal.wait_dio1(timeout_ms).await
    }


    /// Check for a received packet
    pub async fn check_receive(&mut self) -> Result<bool, <Hal as base::HalError>::E> {
        // Poll on DIO and short-circuit if not asserted
        // if self.hal.get_dio1().await? == false {
        //     return Ok(false);
        // }

        let irq = self.get_interrupts(true).await?;
        let mut res = Ok(false);

        trace!("RX poll (irq: {:?})", irq);

        // Process flags
        if irq.contains(Irq::CRC_ERROR) {
            debug!("RX CRC error");
            res = Err(Error::InvalidCrc);
        } else if irq.contains(Irq::RX_TX_TIMEOUT) {
            debug!("RX timeout");
            res = Err(Error::Timeout);
        } else if irq.contains(Irq::SYNCWORD_ERROR) {
            debug!("Invalid syncword");
            res = Err(Error::InvalidSync);
        } else if irq.contains(Irq::RX_DONE) {
            debug!("RX complete");
            res = Ok(true);
        }

        res
    }

    /// Fetch a received packet
    pub async fn get_received(&mut self, data: &mut [u8]) -> Result<(usize, PacketInfo), <Hal as base::HalError>::E> {
        // Fetch RX buffer information
        let (ptr, len) = self.get_rx_buffer_status().await?;

        debug!("RX get received, ptr: {} len: {}", ptr, len);

        if data.len() < len as usize {
            return Err(Error::InvalidLength);
        }

        // TODO: check error packet status byte to ensure CRC is valid
        // as this may not result in a CRC error IRQ.
        // See chip errata for further details

        // Read from the buffer at the provided pointer
        self.hal.read_buff(ptr, &mut data[..len as usize]).await?;

        // Fetch related information
        let mut info = PacketInfo::default();
        self.get_packet_info(&mut info).await?;

        trace!("RX data: {:?} info: {:?}", &data[..len as usize], info);

        // Return read length
        Ok((len as usize, info))
    }

    /// Poll for the current channel RSSI
    /// This should only be called when in receive mode
    pub async fn poll_rssi(&mut self) -> Result<i16, <Hal as base::HalError>::E> {
        let mut raw = [0u8; 1];
        self.hal.read_cmd(Commands::GetRssiInst as u8, &mut raw).await?;
        Ok(-(raw[0] as i16) / 2)
    }
}