use bxcan::filter::Mask32;
use bxcan::{CanConfig, ExtendedId, FilterOwner, Fifo, Frame, Id, Instance, StandardId};
use commkit::DataLink;
use commkit_can::{
    CanBusConfig, CanBusState, CanDataLink, CanError, CanErrorCounters, CanFrame, CanId, LoopbackMode, OneShotMode,
    SamplePointControl,
};

use crate::bit_timing::{DEFAULT_SAMPLE_POINT, bit_timing};

/// bxCAN supports Classic CAN only
const MAX_CLASSIC_LEN: usize = 8;

/// CAN_ESR offset from the peripheral base. `bxcan` keeps its register fields private
const ESR_OFFSET: usize = 0x18;
const ESR_EPVF: u32 = 1 << 1;
const ESR_BOFF: u32 = 1 << 2;

pub struct CanLink<I: Instance> {
    can: bxcan::Can<I>,
    displaced: Option<Frame>,
    pclk_hz: u32,
    open_baud_rate: Option<u32>,
    sample_point: u8,
    loopback: bool,
    one_shot: bool,
}

impl<I: Instance> CanLink<I> {
    /// Takes the peripheral with one filter bank accepting every frame into FIFO 0
    pub fn new(instance: I, pclk_hz: u32) -> Self
    where
        I: FilterOwner,
    {
        let mut link = Self::new_without_filters(instance, pclk_hz);
        link.can.modify_filters().enable_bank(0, Fifo::Fifo0, Mask32::accept_all());
        link
    }

    pub fn new_without_filters(instance: I, pclk_hz: u32) -> Self {
        let can = bxcan::Can::builder(instance)
            .set_loopback(false)
            .set_silent(false)
            .set_automatic_retransmit(true)
            .leave_disabled();

        Self {
            can,
            displaced: None,
            pclk_hz,
            open_baud_rate: None,
            sample_point: DEFAULT_SAMPLE_POINT,
            loopback: false,
            one_shot: false,
        }
    }

    /// The underlying `bxcan` driver, for filters, interrupts and sleep control
    pub fn inner(&mut self) -> &mut bxcan::Can<I> {
        &mut self.can
    }

    pub fn release(self) -> bxcan::Can<I> {
        self.can
    }

    fn read_esr(&self) -> u32 {
        // Safety: aligned, read-only access to a status register of a peripheral this link owns
        unsafe { core::ptr::read_volatile((I::REGISTERS as *const u8).add(ESR_OFFSET) as *const u32) }
    }

    /// Applies a configuration change, leaving the peripheral in whichever open/closed state it was in
    fn reconfigure(&mut self, change: impl FnOnce(CanConfig<'_, I>) -> CanConfig<'_, I>) {
        let open = self.open_baud_rate.is_some();
        let config = change(self.can.modify_config());
        if open {
            config.enable();
        } else {
            config.leave_disabled();
        }
    }

    /// Re-queues a displaced frame, if any. Returns `false` if it is still waiting for a mailbox.
    fn requeue_displaced(&mut self) -> bool {
        // Each displacement trades a frame for a strictly lower-priority one, so this settles within the 3 mailboxes
        while let Some(frame) = self.displaced.take() {
            match self.can.transmit(&frame) {
                Ok(status) => self.displaced = status.dequeued_frame().cloned(),
                Err(nb::Error::WouldBlock) => {
                    self.displaced = Some(frame);
                    return false;
                }
                Err(nb::Error::Other(never)) => match never {},
            }
        }
        true
    }
}

impl<I: Instance> DataLink for CanLink<I> {
    type Packet = CanFrame;
    type PhysicalConfig = CanBusConfig;
    type Error = CanError;

    fn send(&mut self, packet: Self::Packet) -> Result<(), Self::Error> {
        match self.state() {
            CanBusState::Closed => return Err(CanError::NotOpen),
            CanBusState::BusOff => return Err(CanError::BusOff),
            CanBusState::ErrorActive | CanBusState::ErrorPassive => {}
        }

        let frame = to_bxcan(&packet)?;
        if !self.requeue_displaced() {
            return Err(CanError::TxBusy);
        }

        match self.can.transmit(&frame) {
            Ok(status) => {
                self.displaced = status.dequeued_frame().cloned();
                Ok(())
            }
            Err(nb::Error::WouldBlock) => Err(CanError::TxBusy),
            Err(nb::Error::Other(never)) => match never {},
        }
    }

    fn recv(&mut self) -> Result<Option<Self::Packet>, Self::Error> {
        if !self.is_open() {
            return Err(CanError::NotOpen);
        }
        self.requeue_displaced();

        match self.can.receive() {
            Ok(frame) => Ok(Some(from_bxcan(&frame))),
            Err(nb::Error::WouldBlock) => Ok(None),
            Err(nb::Error::Other(_)) => Err(CanError::Overrun),
        }
    }
}

impl<I: Instance> CanDataLink for CanLink<I> {
    /// Blocks until the peripheral has synchronized with the bus (11 recessive bits)
    fn open(&mut self, config: &CanBusConfig) -> Result<(), CanError> {
        if config.fd_enabled {
            return Err(CanError::FdNotSupported);
        }
        let btr = bit_timing(self.pclk_hz, config.baud_rate, self.sample_point).ok_or(CanError::UnsupportedBitTiming)?;

        self.can.modify_config().set_bit_timing(btr).set_silent(config.listen_only).enable();
        self.open_baud_rate = Some(config.baud_rate);
        Ok(())
    }

    fn close(&mut self) -> Result<(), CanError> {
        self.can.modify_config().leave_disabled();
        self.open_baud_rate = None;
        self.displaced = None;
        Ok(())
    }

    fn state(&self) -> CanBusState {
        if self.open_baud_rate.is_none() {
            return CanBusState::Closed;
        }

        let esr = self.read_esr();
        if esr & ESR_BOFF != 0 {
            CanBusState::BusOff
        } else if esr & ESR_EPVF != 0 {
            CanBusState::ErrorPassive
        } else {
            CanBusState::ErrorActive
        }
    }

    fn error_counters(&self) -> CanErrorCounters {
        let esr = self.read_esr();
        CanErrorCounters { tx: (esr >> 16) as u8, rx: (esr >> 24) as u8 }
    }
}

impl<I: Instance> LoopbackMode for CanLink<I> {
    fn set_loopback(&mut self, enable: bool) -> Result<(), CanError> {
        self.reconfigure(|config| config.set_loopback(enable));
        self.loopback = enable;
        Ok(())
    }

    fn loopback(&self) -> bool {
        self.loopback
    }
}

impl<I: Instance> OneShotMode for CanLink<I> {
    fn set_one_shot(&mut self, enable: bool) -> Result<(), CanError> {
        self.reconfigure(|config| config.set_automatic_retransmit(!enable));
        self.one_shot = enable;
        Ok(())
    }

    fn one_shot(&self) -> bool {
        self.one_shot
    }
}

impl<I: Instance> SamplePointControl for CanLink<I> {
    /// While closed, the sample point is validated and applied on the next `open`
    fn set_sample_point(&mut self, percent: u8) -> Result<(), CanError> {
        if let Some(baud_rate) = self.open_baud_rate {
            let btr = bit_timing(self.pclk_hz, baud_rate, percent).ok_or(CanError::UnsupportedBitTiming)?;
            self.reconfigure(|config| config.set_bit_timing(btr));
        } else if !(50..=95).contains(&percent) {
            return Err(CanError::UnsupportedBitTiming);
        }
        self.sample_point = percent;
        Ok(())
    }

    fn sample_point(&self) -> u8 {
        self.sample_point
    }
}

fn to_bxcan(packet: &CanFrame) -> Result<Frame, CanError> {
    if packet.fd || packet.brs {
        return Err(CanError::FdNotSupported);
    }
    let data = packet.data();
    if data.len() > MAX_CLASSIC_LEN {
        return Err(CanError::DataTooLong);
    }

    let id: Id = match packet.id {
        CanId::Standard(raw) => StandardId::new(raw).ok_or(CanError::InvalidId)?.into(),
        CanId::Extended(raw) => ExtendedId::new(raw).ok_or(CanError::InvalidId)?.into(),
    };

    // Remote frames carry no data, so the packet's length stands in for the requested DLC
    if packet.rtr {
        return Ok(Frame::new_remote(id, data.len() as u8));
    }
    let data = bxcan::Data::new(data).ok_or(CanError::DataTooLong)?;
    Ok(Frame::new_data(id, data))
}

fn from_bxcan(frame: &Frame) -> CanFrame {
    let id = match frame.id() {
        Id::Standard(id) => CanId::Standard(id.as_raw()),
        Id::Extended(id) => CanId::Extended(id.as_raw()),
    };

    match frame.data() {
        Some(data) => CanFrame::new(id, false, false, false, data),
        None => CanFrame::new(id, false, false, true, &[0u8; MAX_CLASSIC_LEN][..frame.dlc() as usize]),
    }
}