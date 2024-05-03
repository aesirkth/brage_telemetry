use defmt::{*};
use defmt_rtt as _;

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::Spi;
use embassy_stm32::peripherals::{GPDMA1_CH0, GPDMA1_CH1, SPI1};
use radio_sx128x::device::PacketInfo;
use radio_sx128x::Sx128x;
use radio_sx128x::base::Hal;
use radio_sx128x::device::lora;

use crate::sx_hal::ChungusHal;

struct Amplifier {
    pa_en: Output<'static>,
    pa_tx_en: Output<'static>,
    pa_rx_en: Output<'static>,
    led_tx: Output<'static>,
    led_rx: Output<'static>
}

impl Amplifier {
    fn new(pa_en: Output<'static>, pa_tx_en: Output<'static>, pa_rx_en: Output<'static>, led_tx: Output<'static>, led_rx: Output<'static>) -> Self {
        let mut ret = Amplifier {
            pa_en: pa_en,
            pa_tx_en: pa_tx_en,
            pa_rx_en: pa_rx_en,
            led_tx: led_tx,
            led_rx: led_rx
        };
        ret.set_idle();
        ret
    }

    fn set_receive(&mut self) {
        self.pa_en.set_high();
        self.pa_tx_en.set_low();
        self.pa_rx_en.set_high();
        self.led_rx.set_high();
        self.led_tx.set_low();
    }

    fn set_transmit(&mut self) {
        self.pa_en.set_high();
        self.pa_rx_en.set_low();
        self.pa_tx_en.set_high();
        self.led_tx.set_high();
        self.led_rx.set_low();
    }

    fn set_idle(&mut self) {
        self.pa_en.set_low();
        self.pa_tx_en.set_low();
        self.pa_rx_en.set_low();
        self.led_tx.set_low();
        self.led_rx.set_low();
    }
}

struct Radio<T> {
    sx128x: Sx128x<T>,
    pa: Amplifier,
}

impl<T> Radio<T> where T: Hal {
    const PA_GAIN: i8 = 30;

    pub fn new(sx128x: Sx128x<T>, mut pa: Amplifier) -> Self {
        pa.set_idle();
        let ret = Radio {
            sx128x: sx128x,
            pa: pa,
        };
        ret
    }

    pub async fn set_transmit_power(&mut self, power: i8) {
        self.sx128x.set_power(power - Self::PA_GAIN).await.unwrap();
    }

    pub async fn transmit_and_range(&mut self, data: &[u8], id: u32, timeout_ms: Option<u64>) -> Option<f32> {
        self.sx128x.set_ranging_target(id).await.unwrap();

        // transmit
        self.pa.set_transmit();
        debug!("starting transmission");
        self.sx128x.start_transmit(&data).await.unwrap();
        if self.sx128x.wait_transmit_done(timeout_ms).await.is_err() {
            return None;
        }

        // receive ranging response
        self.pa.set_receive();
        debug!("waiting for ranging response");
        if self.sx128x.wait_ranging_done(timeout_ms).await.is_err() {
            return None;
        }
        self.pa.set_idle();
        debug!("master ranging done");

        // handle ranging result
        let ret = self.sx128x.check_ranging().await;
        if ret.is_err() {
            return None;
        }
        let got_ranging = ret.unwrap();
        if got_ranging {
            let distance = self.sx128x.get_ranging_result().await.unwrap();
            debug!("got valid distance {}", distance);
            Some(distance)
        } else {
            debug!("got no valid distance");
            None
        }
    }

    pub async fn receive_and_range(&mut self, data: &mut [u8], timeout_ms: Option<u64>) -> Option<(usize, PacketInfo)> {

        // receive
        debug!("starting receive");
        self.pa.set_receive();
        self.sx128x.start_receive().await.unwrap();
        if self.sx128x.wait_receive_done(timeout_ms).await.is_err() {
            return None;
        }

        // responding
        if self.sx128x.is_responding().await.unwrap() {
            self.pa.set_transmit();
            debug!("sending ranging response");
            self.sx128x.wait_respond_done(timeout_ms).await.unwrap();
            debug!("ranging response done");
        }
        self.pa.set_idle();

        // handle received stuff
        let got_message = self.sx128x.check_receive().await.unwrap();
        if got_message {
            let res = self.sx128x.get_received(data).await.unwrap();
            debug!("got message");
            Some(res)
        } else {
            debug!("got no message");
            None
        }
    }
}

#[embassy_executor::task]
pub async fn radio_task(
    spi: Spi<'static, SPI1, GPDMA1_CH0, GPDMA1_CH1>,
    sx_cs: Output<'static>,
    sx_busy: ExtiInput<'static>,
    sx_reset: Output<'static>,
    sx_dio1: ExtiInput<'static>,
    sx_dio2: ExtiInput<'static>,
    sx_dio3: ExtiInput<'static>,
    led_tx: Output<'static>,
    led_rx: Output<'static>,
    pa_en: Output<'static>,
    pa_tx_en: Output<'static>,
    pa_rx_en: Output<'static>,
) -> ! {
    let hal = ChungusHal::new(spi, sx_cs, sx_busy, sx_reset, sx_dio1, sx_dio2, sx_dio3);

    let modem = radio_sx128x::device::Modem::Ranging(
        radio_sx128x::device::lora::LoRaConfig {
            preamble_length: 32,
            header_type: radio_sx128x::device::lora::LoRaHeader::Explicit,
            payload_length: 0,
            crc_mode: lora::LoRaCrc::Enabled,
            invert_iq: lora::LoRaIq::Inverted,
        }
    );

    let channel = radio_sx128x::device::Channel::Ranging(
        radio_sx128x::device::lora::LoRaChannel {
            freq: 2.4e9 as u32,
            sf: lora::LoRaSpreadingFactor::Sf5,
            bw: lora::LoRaBandwidth::Bw1600kHz,
            cr: lora::LoRaCodingRate::Cr4_5,
        }
    );

    let ranging = radio_sx128x::device::ranging::RangingConfig {
        calibration_value: 13308,
        id_length: radio_sx128x::device::ranging::IdLength::IdLength08,
        device_id: 0,
        result_type: radio_sx128x::device::ranging::RangingResultType::Raw,
        filter_size: None,
    };

    let mut config = radio_sx128x::Config::default();

    config.modem = modem;
    config.channel = channel;
    config.ranging = ranging;

    config.skip_version_check = true;
    config.pa_config.power = -18;

    let sx128x = radio_sx128x::Sx128x::new(hal, &config).await.unwrap();
    let pa = Amplifier::new(pa_en, pa_tx_en, pa_rx_en, led_tx, led_rx);

    let mut radio = Radio::new(sx128x, pa);
    radio.set_transmit_power(12).await;
    loop {
        let tx_buf = "hejhej".as_bytes();
        let mut rx_buf: [u8; 255] = [0; 255];
        radio.transmit_and_range(tx_buf, 0, Some(100)).await;
        radio.receive_and_range(&mut rx_buf, Some(1000)).await;
    }
}