use commkit::{ByteTransfer, DataLink};
use commkit_uart::{UartConfig, UartDataLink, UartError, UartPacket};
use embedded_hal_nb::serial::{Error as _, ErrorKind, Read, Write};

/// UART data link over any `embedded-hal-nb` serial port, e.g. `stm32f4xx_hal::serial::Serial`
///
/// The port's baud rate and framing are set when the HAL builds it. `recv` drains whatever bytes
/// the peripheral has buffered, up to `N` per call. `send` blocks until every byte has been handed
/// to the transmit register.
pub struct UartLink<S, const N: usize> {
    serial: S,
    /// Error hit mid-read after some bytes were already collected, reported on the next `recv`
    pending_error: Option<UartError>,
}

impl<S, const N: usize> UartLink<S, N>
where
    S: Read<u8> + Write<u8>,
{
    pub fn new(serial: S) -> Self {
        Self { serial, pending_error: None }
    }

    pub fn inner(&mut self) -> &mut S {
        &mut self.serial
    }

    pub fn release(self) -> S {
        self.serial
    }
}

impl<S, const N: usize> DataLink for UartLink<S, N>
where
    S: Read<u8> + Write<u8>,
{
    type Packet = UartPacket<N>;
    type PhysicalConfig = UartConfig;
    type Error = UartError;

    fn send(&mut self, packet: Self::Packet) -> Result<(), Self::Error> {
        for &byte in packet.as_bytes() {
            nb::block!(self.serial.write(byte)).map_err(|e| to_uart_error(e.kind()))?;
        }
        Ok(())
    }

    fn recv(&mut self) -> Result<Option<Self::Packet>, Self::Error> {
        if let Some(error) = self.pending_error.take() {
            return Err(error);
        }

        let mut packet = UartPacket::empty();
        while !packet.is_full() {
            match self.serial.read() {
                Ok(byte) => {
                    packet.push(byte);
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(e)) if packet.is_empty() => return Err(to_uart_error(e.kind())),
                Err(nb::Error::Other(e)) => {
                    self.pending_error = Some(to_uart_error(e.kind()));
                    break;
                }
            }
        }

        Ok((!packet.is_empty()).then_some(packet))
    }
}

impl<S, const N: usize> UartDataLink for UartLink<S, N> where S: Read<u8> + Write<u8> {}

fn to_uart_error(kind: ErrorKind) -> UartError {
    match kind {
        ErrorKind::Overrun => UartError::Overrun,
        ErrorKind::FrameFormat => UartError::Framing,
        ErrorKind::Parity => UartError::Parity,
        ErrorKind::Noise => UartError::Noise,
        _ => UartError::Other,
    }
}