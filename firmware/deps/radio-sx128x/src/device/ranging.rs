#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RangingResultType {
    Raw = 0b00000,
    Average = 0b010000,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum IdLength {
    IdLength08 = 0 << 6,
    IdLength16 = 1 << 6,
    IdLength24 = 2 << 6,
    IdLength32 = 3 << 6,
}


// |    | BW
// | SF |      |   5   |   6   |   7   |   8   |   9   |   10  |
// |    | 400  | 10299 | 10271 | 10244 | 10242 | 10230 | 10246 |
// |    | 800  | 11486 | 11474 | 11453 | 11426 | 11417 | 11401 |
// |    | 1600 | 13308 | 13493 | 13528 | 13515 | 13430 | 13376 |
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RangingConfig {
    pub id_length: IdLength,
    pub device_id: u32,
    pub result_type: RangingResultType,
    pub filter_size: Option<u8>,
    pub calibration_value: u16
}

impl Default for RangingConfig {
    fn default() -> Self {
        RangingConfig {
            id_length: IdLength::IdLength08,
            device_id: 0,
            result_type: RangingResultType::Raw,
            filter_size: None,
            calibration_value: 10299
        }
    }
}