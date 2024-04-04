
#[repr(u8)]
pub enum RangingResultType {
    Raw = 0b00000,
    Average {filter_size: u8} = 0b010000,
}

pub enum IdLength {
    IdLength08 = 0 << 6,
    IdLength16 = 1 << 6,
    IdLength24 = 2 << 6,
    IdLength32 = 3 << 6,
}

pub struct RangingConfig {
    pub id_length: IdLength,
    pub id: u32,
    pub result_type: RangingResultType,
    pub calibration_value: u16
}