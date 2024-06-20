pub struct CanMsg {
    pub id: u16,
    pub length: u8,
    pub data: [u8; 64],
}


pub struct UartMsg {
    pub length: u8,
    pub data: [u8; 100]
}

pub struct RangingMsg {
    pub lon: u32,
    pub lat: u32,
    pub distance_cm: u32,
}

pub enum Msg {
    Uart(UartMsg),
    Can(CanMsg)
}

const UART_FRAME_OVERHEAD: usize = 2;
const CAN_FRAME_OVERHEAD: usize = 4;
const HEADER_SIZE: usize = 3;
const FOOTER_SIZE: usize = 2;

const X25: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_IBM_SDLC);

pub fn begin_buffer(data: &mut [u8], id: u8) -> Result<usize, usize> {
    if data.len() < HEADER_SIZE + FOOTER_SIZE {
        return Err(0)
    }
    data[0] = b'b';
    data[1] = id;
    Ok(HEADER_SIZE)
    // Ok(100)
}

pub fn append_can_frame(data: &mut [u8], index: usize, msg: CanMsg) -> Result<usize, usize> {
    if data.len() - index < msg.length as usize + CAN_FRAME_OVERHEAD + FOOTER_SIZE {
        return Err(index);
    }
    data[index] = 'c' as u8;
    data[index + 1 .. index + 3 as usize].copy_from_slice(&msg.id.to_le_bytes());
    data[index + 3] = msg.length as u8;
    data[index + 4 .. index + 3 + msg.length as usize].copy_from_slice(&msg.data);
    Ok(index + 5 + msg.length as usize)
}

pub fn append_uart_frame(data: &mut [u8], index: usize, msg: UartMsg) -> Result<usize, usize> {
    if data.len() - index < msg.length as usize + UART_FRAME_OVERHEAD + FOOTER_SIZE {
        return Err(index);
    }
    data[index] = 'u' as u8;
    data[index + 1] = msg.length as u8;
    data[index + 2 .. index + 2 + msg.length as usize].copy_from_slice(&msg.data[0 .. msg.length as usize]);
    Ok(index + 2 + msg.length as usize)
}

pub fn finish_buffer<'a>(data: &'a mut [u8], index: usize) -> Result<&'a [u8], ()> {
    if data.len() - index < FOOTER_SIZE {
        panic!("remaing bytes are too few to encode footer");
    }
    let payload_len = index - HEADER_SIZE;
    data[2] = payload_len as u8;
    let checksum: u16 = X25.checksum(data);
    data[index..index + FOOTER_SIZE].copy_from_slice(&checksum.to_le_bytes());
    Ok(&data[..index + FOOTER_SIZE])
    // Ok(&data[0..200])
}

pub enum MsgError {
    InvalidHeader,
    InvalidCrc,
    InvalidLen,
}

pub fn verify_and_extract_messages(data: &[u8]) -> Result<&[u8], MsgError>{
    if &data[..2] != "br".as_bytes() {
        return Err(MsgError::InvalidHeader);
    };
    let len = data[3];
    if data.len() != len as usize + HEADER_SIZE + FOOTER_SIZE {
        return Err(MsgError::InvalidLen);
    };
    let calculated_checksum: u16 = X25.checksum(&data[..len as usize + HEADER_SIZE]);
    let received_checksum: u16 = u16::from_le_bytes(data[FOOTER_SIZE + len as usize..].try_into().unwrap());
    if calculated_checksum != received_checksum {
        return Err(MsgError::InvalidCrc);
    };

    Ok(&data[HEADER_SIZE .. data.len() - FOOTER_SIZE])
}

pub fn consume_msg(data: &[u8]) -> Result<(Msg, &[u8]), MsgError> {
    let ret;
    let index: usize;
    match data[0] {
        b'c' => {
            if data.len() < CAN_FRAME_OVERHEAD {
                return Err(MsgError::InvalidLen)
            }
            let id = u16::from_le_bytes(data[1 .. 3].try_into().unwrap());
            let length = data[3];
            if data.len() < CAN_FRAME_OVERHEAD + length as usize {
                return Err(MsgError::InvalidLen)
            }
            let mut can_msg = CanMsg {
                id: id,
                length: length,
                data: [0; 64]
            };
            can_msg.data[..length as usize].copy_from_slice(&data[4 .. 4 + length as usize]);
            index = 4 + length as usize;
            ret = Msg::Can(can_msg);
        },

        b'u' => {
            if data.len() < UART_FRAME_OVERHEAD {
                return Err(MsgError::InvalidLen)
            }
            let length = data[1];
            if data.len() < UART_FRAME_OVERHEAD + length as usize {
                return Err(MsgError::InvalidLen)
            }
            let mut uart_msg = UartMsg {
                length: length,
                data: [0; 100]
            };
            uart_msg.data[..length as usize].copy_from_slice(&data[2 .. 2 + length as usize]);
            index = 2 + length as usize;
            ret = Msg::Uart(uart_msg);
        }

        _ => return Err(MsgError::InvalidHeader),
    };

    Ok((ret, &data[index..]))
}