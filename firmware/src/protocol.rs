use arrayvec::ArrayVec;


const UART_FRAME_OVERHEAD: usize = 1;
const CAN_FRAME_OVERHEAD: usize = 2;
const HEADER_SIZE: usize = 2;
const X25: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_IBM_SDLC);

pub fn append_header(data: &mut ArrayVec<u8, 256>) -> Result<(), ()> {
    data.try_extend_from_slice("bra".as_bytes()).unwrap();
    Ok(())
}

pub fn append_can_frame(data: &mut ArrayVec<u8, 256>, msg: crate::CanMsg) -> Result<(), ()> {
    if data.remaining_capacity() < msg.length as usize + CAN_FRAME_OVERHEAD + HEADER_SIZE {
        return Err(());
    }
    data.push('c' as u8);
    data.push(msg.id as u8);
    data.push(msg.length as u8);
    data.try_extend_from_slice(&msg.data[0..msg.length as usize]).unwrap();
    Ok(())
}

pub fn append_uart_frame(data: &mut ArrayVec<u8, 256>, msg: crate::UartMsg) -> Result<(), ()> {
    if data.remaining_capacity() < msg.length as usize + UART_FRAME_OVERHEAD + HEADER_SIZE {
        return Err(());
    }
    data.push('u' as u8);
    data.push(msg.length as u8);
    data.try_extend_from_slice(&msg.data[0..msg.length as usize]).unwrap();
    Ok(())
}

pub fn append_footer(data: &mut ArrayVec<u8, 256>) -> Result<(), ()> {
    data[2] = data.len() as u8;
    let checksum: u16 = X25.checksum(data.as_slice());
    data.try_extend_from_slice(&checksum.to_le_bytes()).unwrap();
    Ok(())
}