pub const DEFAULT_SAMPLE_POINT: u8 = 87;

const TQ_MIN: u32 = 8;
const TQ_MAX: u32 = 25;
const TS1_MAX: u32 = 16;
const TS2_MAX: u32 = 8;
const BRP_MAX: u32 = 1024;

/// Computes a bxCAN `CAN_BTR` value (SJW = 1) for the given peripheral clock, bit rate and sample point
pub fn bit_timing(pclk_hz: u32, baud_rate: u32, sample_point_percent: u8) -> Option<u32> {
    if baud_rate == 0 || !(50..=95).contains(&sample_point_percent) {
        return None;
    }
    let target_permille = sample_point_percent as u32 * 10;

    let mut best: Option<(u32, u32, u32, u32)> = None;
    for tq in (TQ_MIN..=TQ_MAX).rev() {
        let Some(tq_rate) = baud_rate.checked_mul(tq) else { continue };
        if !pclk_hz.is_multiple_of(tq_rate) {
            continue;
        }
        let brp = pclk_hz / tq_rate;
        if brp == 0 || brp > BRP_MAX {
            continue;
        }

        let ts2 = ((tq * (100 - sample_point_percent as u32) + 50) / 100).clamp(1, TS2_MAX);
        let ts1 = tq - 1 - ts2;
        if !(1..=TS1_MAX).contains(&ts1) {
            continue;
        }

        let error = ((1 + ts1) * 1000 / tq).abs_diff(target_permille);
        if best.is_none_or(|(best_error, ..)| error < best_error) {
            best = Some((error, brp, ts1, ts2));
        }
    }

    best.map(|(_, brp, ts1, ts2)| ((ts2 - 1) << 20) | ((ts1 - 1) << 16) | (brp - 1))
}