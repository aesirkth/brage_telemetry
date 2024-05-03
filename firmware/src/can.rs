use crate::CanMsg;

#[embassy_executor::task]
pub async fn can_reader_task(can_rx: embassy_stm32::can::BufferedFdCanReceiver) -> ! {
    loop {
        let res = can_rx.receive().await.unwrap();
        // let frame = CanMsg {
        //     id: res.frame.id(). as u16,
        //     length: res.frame.data().len(),
        //     data: [0; 64]
        // };
    }
}

#[embassy_executor::task]
pub async fn can_writer_task(_can_tx: embassy_stm32::can::BufferedFdCanSender) -> ! {
    loop {

    }
}