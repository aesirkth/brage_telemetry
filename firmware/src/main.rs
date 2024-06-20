#![no_std]
#![no_main]


use defmt::{*};
use defmt_rtt as _;


use embassy_stm32::peripherals::{FDCAN1, USB_OTG_FS, USART2};
use embassy_stm32::time::Hertz;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::exti::ExtiInput;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;


use panic_probe as _;

pub mod sx_hal;
pub mod protocol;

pub static UART_IN: Channel<ThreadModeRawMutex, protocol::UartMsg, 32> = Channel::new();
pub static UART_OUT: Channel<ThreadModeRawMutex, protocol::UartMsg, 32> = Channel::new();

pub static CAN_OUT: Channel<ThreadModeRawMutex, protocol::CanMsg, 32> = Channel::new();
pub static CAN_IN: Channel<ThreadModeRawMutex, protocol::CanMsg, 32> = Channel::new();

mod radio;
mod can;
mod uart;

embassy_stm32::bind_interrupts!(struct Irqs {
    OTG_FS => embassy_stm32::usb::InterruptHandler<USB_OTG_FS>;
    FDCAN1_IT0 => embassy_stm32::can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => embassy_stm32::can::IT1InterruptHandler<FDCAN1>;
    USART2 => embassy_stm32::usart::BufferedInterruptHandler<USART2>;
});


#[embassy_executor::task]
async fn logger_task(driver: embassy_stm32::usb::Driver<'static, USB_OTG_FS>) -> ! {
    embassy_usb_logger::run!(1024, log::LevelFilter::Trace, driver);
    loop {
        core::panic!("unreachable");
    }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = true;
        config.rcc.hse = Some(Hse {
            freq: Hertz(48_000_000),
            mode: HseMode::Oscillator
        });
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE, // 48 MHz
            prediv: PllPreDiv::DIV3, // 16 MHz
            mul: PllMul::MUL10, // 160 MHz
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV2),
            divr: Some(PllDiv::DIV1), // 160 MHz
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.voltage_range = VoltageScale::RANGE1;
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB
        config.rcc.mux.iclksel = mux::Iclksel::HSI48; // USB uses ICLK
    }

    let p = embassy_stm32::init(config);

    // Create the driver, from the HAL.
    static mut USB_BUF: core::cell::UnsafeCell<[u8; 1024]> = core::cell::UnsafeCell::new([0; 1024]);
    let mut config = embassy_stm32::usb::Config::default();
    config.vbus_detection = false;
    let usb_driver = embassy_stm32::usb::Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, unsafe {USB_BUF.get_mut()}, config);
    // spawner.spawn(logger_task(usb_driver)).unwrap();

    info!("Hello World!");

    let sx_cs = Output::new(p.PB0, Level::High, Speed::VeryHigh);
    let sx_reset = Output::new(p.PB6, Level::High, Speed::VeryHigh);
    let sx_dio1 = ExtiInput::new(p.PB3, p.EXTI3, Pull::Down);
    let sx_dio2 = ExtiInput::new(p.PB4, p.EXTI4, Pull::Down);
    let sx_dio3 = ExtiInput::new(p.PB5, p.EXTI5, Pull::Down);
    let sx_busy = ExtiInput::new(p.PA15, p.EXTI15, Pull::Down);

    let led_tx = Output::new(p.PB12, Level::Low, Speed::VeryHigh);
    let led_rx = Output::new(p.PB13, Level::Low, Speed::VeryHigh);

    let pa_en = Output::new(p.PC13, Level::Low, Speed::VeryHigh);
    let pa_tx_en = Output::new(p.PC14, Level::Low, Speed::VeryHigh);
    let pa_rx_en = Output::new(p.PC15, Level::Low, Speed::VeryHigh);
    let vref_en = Output::new(p.PB2, Level::Low, Speed::VeryHigh); // soldered to LED G with a jumper

    let mut can_configurator = embassy_stm32::can::CanConfigurator::new(p.FDCAN1, p.PB8, p.PB9, Irqs);

    can_configurator.set_bitrate(500_000);

    can_configurator.set_fd_data_bitrate(1_000_000, true);

    let can = can_configurator.start(embassy_stm32::can::OperatingMode::NormalOperationMode);

    static mut CAN_TX_BUF: core::cell::UnsafeCell<embassy_stm32::can::TxFdBuf<1024>> = core::cell::UnsafeCell::new(embassy_stm32::can::TxFdBuf::new());
    static mut CAN_RX_BUF: core::cell::UnsafeCell<embassy_stm32::can::RxFdBuf<1024>> = core::cell::UnsafeCell::new(embassy_stm32::can::RxFdBuf::new());

    let can = can.buffered_fd(unsafe{CAN_TX_BUF.get_mut()}, unsafe {CAN_RX_BUF.get_mut()});

    let can_rx = can.reader();
    let can_tx = can.writer();

    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = Hertz(10_000_000);

    let spi = embassy_stm32::spi::Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.GPDMA1_CH0, p.GPDMA1_CH1, spi_config);

    static mut UART_TX_BUF: core::cell::UnsafeCell<[u8; 512]> = core::cell::UnsafeCell::new([0; 512]);
    static mut UART_RX_BUF: core::cell::UnsafeCell<[u8; 512]> = core::cell::UnsafeCell::new([0; 512]);
    let config = embassy_stm32::usart::Config::default();
    let uart = embassy_stm32::usart::BufferedUart::new_with_rtscts(p.USART2, Irqs, p.PA3, p.PA2, p.PA1, p.PA0, unsafe {UART_TX_BUF.get_mut()}, unsafe {UART_RX_BUF.get_mut()}, config).unwrap();

    let (uart_tx, uart_rx) = uart.split();
    spawner.spawn(radio::radio_task(spi, sx_cs, sx_busy, sx_reset, sx_dio1, sx_dio2, sx_dio3, led_tx, led_rx, pa_en, pa_tx_en, pa_rx_en, vref_en)).unwrap();
    spawner.spawn(can::can_reader_task(can_rx)).unwrap();
    spawner.spawn(can::can_writer_task(can_tx)).unwrap();
    spawner.spawn(uart::uart_writer_task(uart_tx)).unwrap();
    spawner.spawn(uart::uart_reader_task(uart_rx)).unwrap();
}