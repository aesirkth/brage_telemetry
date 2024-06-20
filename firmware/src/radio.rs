
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::Spi;
use embassy_stm32::peripherals::SPI1;
use embassy_stm32::mode::Async;
use radio_sx128x::device::PacketInfo;
use radio_sx128x::Sx128x;
use radio_sx128x::base::Hal;
use radio_sx128x::device::lora;

use crate::protocol::{consume_msg, verify_and_extract_messages};
use crate::sx_hal::ChungusHal;

use defmt::{*};

use crate::{protocol, CAN_IN, CAN_OUT, UART_IN, UART_OUT};

struct Amplifier {
    pa_en: Output<'static>,
    pa_tx_en: Output<'static>,
    pa_rx_en: Output<'static>,
    led_tx: Output<'static>,
    led_rx: Output<'static>,
    vref_en: Output<'static>
}

impl Amplifier {
    fn new(pa_en: Output<'static>, pa_tx_en: Output<'static>, pa_rx_en: Output<'static>, led_tx: Output<'static>, led_rx: Output<'static>, vref_en: Output<'static>) -> Self {
        let mut ret = Amplifier {
            pa_en: pa_en,
            pa_tx_en: pa_tx_en,
            pa_rx_en: pa_rx_en,
            led_tx: led_tx,
            led_rx: led_rx,
            vref_en: vref_en
        };
        ret.set_idle();
        ret
    }

    fn set_receive(&mut self) {
        self.pa_en.set_high();
        self.vref_en.set_low();
        self.pa_tx_en.set_low();
        self.pa_rx_en.set_high();
        self.led_rx.set_high();
        self.led_tx.set_low();
    }

    fn set_transmit(&mut self) {
        self.pa_en.set_high();
        self.vref_en.set_high();
        self.pa_tx_en.set_high();
        self.pa_rx_en.set_low();
        self.led_tx.set_high();
        self.led_rx.set_low();

    }

    fn set_idle(&mut self) {
        self.pa_en.set_low();
        self.vref_en.set_low();
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
    const PA_GAIN: i8 = 34;

    pub fn new(sx128x: Sx128x<T>, mut pa: Amplifier) -> Self {
        pa.set_idle();
        let ret = Radio {
            sx128x: sx128x,
            pa: pa,
        };
        ret
    }


    // pub async fn set_transmit_power(&mut self, power: i8) {
    //     self.sx128x.set_power(power - Self::PA_GAIN).await.unwrap();
    // }

    pub async fn transmit_and_range(&mut self, data: &[u8], id: u32, timeout_ms: Option<u64>) -> Option<f32> {
        self.sx128x.set_ranging_target(id).await.unwrap();

        // transmit
        self.pa.set_transmit();
        info!("starting ranging transmission");
        let tx_start = embassy_time::Instant::now();
        self.sx128x.start_transmit(&data).await.unwrap();
        if self.sx128x.wait_transmit_done(timeout_ms).await.is_err() {
            warn!("tx timeout");
            return None;
        }
        let tx_done = embassy_time::Instant::now();

        // receive ranging response
        self.pa.set_receive();
        info!("waiting for ranging response");
        if self.sx128x.wait_master_ranging_done(timeout_ms).await.is_err() {
            warn!("ranging rx timeout");
        }

        self.sx128x.check_transmit().await.unwrap();

        let rx_done = embassy_time::Instant::now();
        self.pa.set_idle();
        info!("master ranging done");
        info!("time {} {}", tx_done.as_micros() - tx_start.as_micros(), rx_done.as_micros() - tx_done.as_micros());
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
            self.sx128x.wait_slave_ranging_done(timeout_ms).await.unwrap();
            debug!("ranging response done");
        }
        // self.pa.set_idle();

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

    pub async fn transmit(&mut self, data: &[u8], timeout_ms: Option<u64>) -> Result<(), ()> {
        // transmit
        self.pa.set_transmit();
        embassy_time::Timer::after_millis(100).await;

        info!("starting transmission");
        if let Err(_) = self.sx128x.start_transmit(&data).await {
            warn!("start transmit failed");
            return Err(());
        }
        warn!("wet chungus");
        let state = self.sx128x.get_state().await.unwrap();

        if let Err(_) = self.sx128x.wait_transmit_done(timeout_ms).await {
            warn!("wait transmit done failed");
            return Err(());
        }
        // self.pa.set_idle();
        match self.sx128x.check_transmit().await {
            Err(_) => {
                warn!("check transmit failed");
                return Err(());
            },
            Ok(sent) => {
                if sent == false {
                    warn!("transmit not successful");
                    return Err(());
                }
            }
        };

        Ok(())
    }
    pub async fn receive(&mut self, data: &mut [u8], timeout_ms: Option<u64>) -> Option<(usize, PacketInfo)> {
        self.pa.set_receive();
        embassy_time::Timer::after_millis(100).await;
        if let Err(_) = self.sx128x.start_receive().await {
            warn!("Start receive failed");
            return None
        }

        warn!("big chungus");
        let state = self.sx128x.get_state().await.unwrap();
        embassy_time::Timer::after_millis(10).await;
        warn!("big chungus2");
        let state = self.sx128x.get_state().await.unwrap();

        if let Err(_) = self.sx128x.wait_receive_done(timeout_ms).await {
            warn!("wait receive timed out");
        }

        self.pa.set_idle();
        match self.sx128x.check_receive().await {
            Err(_) => {
                warn!("check receive failed");
                return None;
            },
            Ok(received) => {
                if received == false {
                    warn!("receive not successful");
                    return None;
                }
            }
        };

        match self.sx128x.get_received(data).await {
            Ok(result) => {
                info!("received message");
                Some(result)
            },
            Err(_) => {
                warn!("get received failed");
                None
            }
        }
    }
}

#[embassy_executor::task]
pub async fn radio_task(
    spi: Spi<'static, SPI1, Async>,
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
    vref_en: Output<'static>
) -> ! {
    let hal = ChungusHal::new(spi, sx_cs, sx_busy, sx_reset, sx_dio1, sx_dio2, sx_dio3);

    // let modem = radio_sx128x::device::Modem::LoRa(
    //     lora::LoRaConfig {
    //         preamble_length: 64,
    //         header_type: lora::LoRaHeader::Explicit,
    //         payload_length: 0,
    //         crc_mode: lora::LoRaCrc::Disabled,
    //         invert_iq: lora::LoRaIq::Normal,
    //     }
    // );


    let modem = radio_sx128x::device::Modem::Gfsk(
        radio_sx128x::device::gfsk::GfskConfig::default()
    );

    let channel = radio_sx128x::device::Channel::Gfsk(
        radio_sx128x::device::gfsk::GfskChannel::default()
    );

    let ranging = radio_sx128x::device::ranging::RangingConfig {
        calibration_value: 13308,
        id_length: radio_sx128x::device::ranging::IdLength::IdLength08,
        device_id: 0,
        result_type: radio_sx128x::device::ranging::RangingResultType::Raw,
        filter_size: None,
    };

    let mut config = radio_sx128x::Config::default();
    config.rf_timeout = radio_sx128x::device::Timeout::Configurable { step: radio_sx128x::device::TickSize::TickSize0062us, count: 2*10*100 };
    config.modem = modem;
    config.channel = channel;
    config.ranging = ranging;

    config.skip_version_check = true;
    config.pa_config.power = -14;

    let sx128x = radio_sx128x::Sx128x::new(hal, &config).await.unwrap();
    let pa = Amplifier::new(pa_en, pa_tx_en, pa_rx_en, led_tx, led_rx, vref_en);

    let mut radio = Radio::new(sx128x, pa);
    // radio.set_transmit_power(30).await;
    loop {
        let mut rx_buf: [u8; 255] = [0; 255];
        if let Some((size, _packet_info)) = radio.receive(&mut rx_buf, Some(5000)).await {
            if let Ok(mut msg_data) = verify_and_extract_messages(&rx_buf[0..size]) {
                while let Ok((msg, new_msg_data)) = consume_msg(msg_data) {
                    msg_data = new_msg_data;
                    match msg {
                        protocol::Msg::Can(can_msg) => {
                            if let Err(_) = CAN_OUT.try_send(can_msg) {
                                warn!("Could not push CAN msg");
                            };
                        },
                        protocol::Msg::Uart(uart_msg) => {
                            if let Err(_) = UART_OUT.try_send(uart_msg) {
                                warn!("Could not push UART msg");
                            };
                        },
                    };
                }
            }
        }


        let mut tx_buf: [u8; 255] = [0; 255];
        let mut index = protocol::begin_buffer(&mut tx_buf, 0).unwrap();


        // Collect all messages
        loop {
            let mut got_msg = false;
            if let Ok(uart_msg) = UART_IN.try_receive() {
                match protocol::append_uart_frame(&mut tx_buf, index, uart_msg) {
                    Ok(new_index) => {
                        index = new_index;
                        got_msg = true;
                    },
                    Err(new_index) => {
                        index = new_index;
                    }
                }
            }

            if let Ok(can_msg) = CAN_IN.try_receive() {
                match protocol::append_can_frame(&mut tx_buf, index, can_msg) {
                    Ok(new_index) => {
                        index = new_index;
                        got_msg = true;
                    },
                    Err(new_index) => {
                        index = new_index;
                    }
                }
            }

            if got_msg == false {
                break
            }
        }
        let radio_packet = protocol::finish_buffer(&mut tx_buf, index).unwrap();
        radio.transmit(&radio_packet, Some(3000)).await.unwrap();
    }
}