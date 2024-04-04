#![no_std]
#![no_main]

use core::cell::UnsafeCell;

use defmt::{*};
use defmt_rtt as _;
use embassy_stm32::dma::NoDma;
use embassy_stm32::peripherals::{FDCAN1, SPI1, USB_OTG_FS};
use embassy_stm32::time::Hertz;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::exti::ExtiInput;

use embedded_hal::delay::DelayNs;
use panic_probe as _;

mod sx_hal;

use radio::{Transmit, Receive};
use radio_sx128x::Sx128x;

embassy_stm32::bind_interrupts!(struct Irqs {
    OTG_FS => embassy_stm32::usb::InterruptHandler<USB_OTG_FS>;
    FDCAN1_IT0 => embassy_stm32::can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => embassy_stm32::can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::task]
async fn logger_task(driver: embassy_stm32::usb::Driver<'static, USB_OTG_FS>) -> ! {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
    loop {
        core::panic!("unreachable");
    }
}

#[embassy_executor::task]
async fn can_reader_task(can_rx: embassy_stm32::can::BufferedFdCanReceiver) -> ! {
    loop {

    }
}

#[embassy_executor::task]
async fn can_writer_task(can_tx: embassy_stm32::can::BufferedFdCanSender) -> ! {
    loop {

    }
}

#[embassy_executor::task]
async fn radio_task(
    spi: embassy_stm32::spi::Spi<'static, SPI1, NoDma, NoDma>,
    sx_cs: Output<'static>,
    sx_busy: Input<'static>,
    sx_reset: Output<'static>,
    sx_dio1_exti: ExtiInput<'static>,
    mut led_tx: Output<'static>,
    mut led_rx: Output<'static>,
    mut pa_en: Output<'static>,
    mut pa_tx_en: Output<'static>,
    mut pa_rx_en: Output<'static>,
) -> ! {
    let hal = sx_hal::ChungusHal::new(spi, sx_cs, sx_busy, sx_reset, sx_dio1_exti);
    let config = radio_sx128x::Config::default();
    let mut sx128x = radio_sx128x::Sx128x::new(hal, &config).unwrap();
    let calib = radio_sx128x::device::CalibrationParams::all();
    sx128x.calibrate(calib).unwrap();
    pa_en.set_high();
    pa_tx_en.set_low();
    pa_rx_en.set_low();

    let tx_buf: [u8; 6] = *b"hejsan";
    let mut rx_buf: [u8; 255] = [0; 255];
    loop {
        pa_tx_en.set_high();
        led_tx.set_high();
        sx128x.start_transmit(&tx_buf).unwrap();
        info!("Started transmission");
        embassy_time::Timer::after_millis(100).await;
        sx128x.check_transmit().unwrap();
        pa_tx_en.set_low();
        led_tx.set_low();

        info!("finished transmission");
        pa_rx_en.set_high();
        led_rx.set_high();

        sx128x.start_receive().unwrap();
        embassy_time::Timer::after_millis(1000).await;
        info!("started reception");
        if sx128x.check_receive(false).unwrap() {
            info!("received");
            sx128x.get_received(&mut rx_buf).unwrap();
        } else {
            info!("not received");
        }
        pa_rx_en.set_low();
        led_rx.set_low();

        info!("finished reception");
    }
}


#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI, // 16 MHz
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: None,
            divq: None,
            divr: Some(PllDiv::DIV1), // 160 MHz
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.voltage_range = VoltageScale::RANGE1;
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB
        config.rcc.mux.iclksel = mux::Iclksel::HSI48; // USB uses ICLK
    }

    let p = embassy_stm32::init(config);

    // Create the driver, from the HAL.
    static mut USB_BUF: UnsafeCell<[u8; 1024]> = UnsafeCell::new([0_u8; 1024]);
    let mut config = embassy_stm32::usb::Config::default();
    config.vbus_detection = false;
    let usb_driver = embassy_stm32::usb::Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, unsafe {USB_BUF.get_mut()}, config);
    spawner.spawn(logger_task(usb_driver)).unwrap();

    info!("Hello World!");

    let sx_cs = Output::new(p.PB0, Level::Low, Speed::VeryHigh);
    let sx_reset = Output::new(p.PB6, Level::Low, Speed::VeryHigh);
    let sx_dio1 = ExtiInput::new(p.PB3, p.EXTI3, Pull::Up);
    let _sx_dio2 = Input::new(p.PB4, Pull::Up);
    let _sx_dio3 = Input::new(p.PB5, Pull::Up);
    let sx_busy = Input::new(p.PA15, Pull::Up);

    let _led_g = Output::new(p.PB2, Level::Low, Speed::VeryHigh);
    let _led_b = Output::new(p.PB14, Level::Low, Speed::VeryHigh);
    let led_tx = Output::new(p.PB12, Level::Low, Speed::VeryHigh);
    let led_rx = Output::new(p.PB13, Level::Low, Speed::VeryHigh);

    let pa_en = Output::new(p.PC13, Level::Low, Speed::VeryHigh);
    let pa_tx_en = Output::new(p.PC14, Level::Low, Speed::VeryHigh);
    let pa_rx_en = Output::new(p.PB15, Level::Low, Speed::VeryHigh);


    let mut can_configurator = embassy_stm32::can::CanConfigurator::new(p.FDCAN1, p.PB8, p.PB9, Irqs);

    can_configurator.set_extended_filter(
        embassy_stm32::can::filter::ExtendedFilterSlot::_0,
        embassy_stm32::can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );

    // 250k bps
    can_configurator.set_bitrate(1_000_000);

    let use_fd = true;

    // 1M bps
    if use_fd {
        can_configurator.set_fd_data_bitrate(1_000_000, false);
    }

    let can = can_configurator.start(embassy_stm32::can::OperatingMode::NormalOperationMode);

    static mut TX_BUF: core::cell::UnsafeCell<embassy_stm32::can::TxFdBuf<1024>> = core::cell::UnsafeCell::new(embassy_stm32::can::TxFdBuf::new());
    static mut RX_BUF: core::cell::UnsafeCell<embassy_stm32::can::RxFdBuf<1024>> = core::cell::UnsafeCell::new(embassy_stm32::can::RxFdBuf::new());

    let can = can.buffered_fd(unsafe{TX_BUF.get_mut()}, unsafe {RX_BUF.get_mut()});

    #[allow(unused_variables)]
    let can_rx = can.reader();
    #[allow(unused_variables)]
    let can_tx = can.writer();

    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = Hertz(1_000_000);

    let spi = embassy_stm32::spi::Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, NoDma, NoDma, spi_config);

    spawner.spawn(radio_task(spi, sx_cs, sx_busy, sx_reset, sx_dio1, led_tx, led_rx, pa_en, pa_tx_en, pa_rx_en)).unwrap();
}