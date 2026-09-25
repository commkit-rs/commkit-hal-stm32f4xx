#![cfg_attr(not(test), no_std)]

mod bit_timing;
mod can;
mod uart;

pub use bit_timing::{DEFAULT_SAMPLE_POINT, bit_timing};
pub use can::CanLink;
pub use uart::UartLink;

pub use bxcan;
