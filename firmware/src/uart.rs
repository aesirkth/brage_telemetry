use embassy_stm32::peripherals::USART2;

use crate::UartMsg;

#[embassy_executor::task]
pub async fn uart_reader_task(uart_rx: embassy_stm32::usart::BufferedUartRx<'static, USART2>) -> ! {
    loop {
    }
}

#[embassy_executor::task]
pub async fn uart_writer_task(uart_tx: embassy_stm32::usart::BufferedUartTx<'static, USART2>) -> ! {
    loop {

    }
}