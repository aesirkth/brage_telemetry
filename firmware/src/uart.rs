use embassy_stm32::peripherals::USART2;
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx};
use embedded_io_async::{Read, Write};

use crate::{UART_IN, UART_OUT, protocol};

#[embassy_executor::task]
pub async fn uart_reader_task(mut uart_rx: BufferedUartRx<'static, USART2>) -> ! {
    let mut rx_buf: [u8; 100] = [0; 100];
    loop {
        let count = uart_rx.read(&mut rx_buf).await.unwrap();

        let msg = protocol::UartMsg {
            length: count as u8,
            data: rx_buf,
        };

        UART_IN.send(msg).await;
    }
}

#[embassy_executor::task]
pub async fn uart_writer_task(mut uart_tx: BufferedUartTx<'static, USART2>) -> ! {
    loop {
        let msg = UART_OUT.receive().await;
        uart_tx.write_all(&msg.data[0..msg.length as usize]).await.unwrap();
    }
}