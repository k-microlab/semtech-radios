//! SPI interface commands for the radio.

use defmt::println;

use crate::{
    shared,
    shared::{MAX_ITERS, OpCode, RadioError},
    status,
};


// Note: Should be 256.
// pub const RADIO_BUF_SIZE: usize = 256;
pub const MAX_OTA_SIZE: usize = 256;
pub const RADIO_BUF_SIZE: usize = 256 + 3; // For the 3 header bytes?

// // todo: - 3 for opcode, start buffer pointer, and the third byte?
// pub const MAX_OTA_PAYLOAD: usize = RADIO_BUF_SIZE - 3;

use crate::shared::{RadioPins, Register};

pub struct Interface<SPI, CS, BUSY, RESET> {
    pub spi: SPI,
    pub pins: RadioPins<CS, BUSY, RESET>,
    pub read_buf: [u8; RADIO_BUF_SIZE],
    pub write_buf: [u8; RADIO_BUF_SIZE],
    pub rx_payload_len: u8,
    pub rx_payload_start: u8,
    /// Otherwise, 6x.
    pub r8x: bool,
}

impl<SPI, CS, BUSY, RST> Interface<SPI, CS, BUSY, RST>
where
    SPI: embedded_hal::spi::SpiDevice,
    CS: embedded_hal::digital::OutputPin,
    BUSY: embedded_hal::digital::InputPin,
    RST: embedded_hal::digital::OutputPin,
{
    pub fn reset<DELAY: embedded_hal::delay::DelayNs>(&mut self, delay: &mut DELAY) {
        // Should only need 100us.
        self.pins.reset.set_low();
        delay.delay_ms(500);
        self.pins.reset.set_high();
    }

    /// Wait for the radio to be ready to accept commands, using the busy pin. If the busy pin is high,
    /// the radio is not ready for commands.
    pub fn wait_on_busy(&mut self) -> Result<(), RadioError> {
        let mut i = 0;

        while self.pins.busy.is_high().unwrap() {
            i += 1;
            if i >= MAX_ITERS {
                println!("Exceeded max iters on wait on busy.");
                return Err(RadioError::BusyTimeout);
            }
        }
        Ok(())
    }

    /// Perform a write to an opcode, with 1 byte of data.
    pub fn write_op_word(&mut self, code: OpCode, word: u8) -> Result<(), RadioError> {
        self.wait_on_busy()?;

        let c = if self.r8x { code.val_8x() } else { code as u8 };

        self.pins.cs.set_low();

        // if self.spi.write(&[c, word]).is_err() {
        //     self.pins.cs.set_high();
        //     return Err(RadioError::Spi);
        // }

        let mut buf = [c, word];
        if self.spi.transfer(&mut buf, &[c, word]).is_err() {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        }
        self.pins.cs.set_high();

        let status = status::status_from_byte(buf[0], self.r8x);
        // println!("STATUS OP WORD WRITE: {:x}, {:?}", code as u8, status);

        Ok(())
    }

    /// Perform a read of an opcode, with 1 byte of data.
    pub fn read_op_word(&mut self, code: OpCode) -> Result<u8, RadioError> {
        let c = if self.r8x { code.val_8x() } else { code as u8 };

        let mut buf = [c, 0, 0, 0, 0];

        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self.spi.transfer(&mut buf, &[]).is_err() {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.pins.cs.set_high();

        // todo: Status is buf[1]. Use it? How do we interpret it?
        Ok(buf[2])
    }

    /// Write a single word to a register.
    pub fn write_reg_word(&mut self, reg: Register, word: u8) -> Result<(), RadioError> {
        let r = match reg {
            Register::Reg6x(reg) => reg as u16,
            Register::Reg8x(reg) => reg as u16,
        };

        let c = if self.r8x {
            OpCode::WriteRegister.val_8x()
        } else {
            OpCode::WriteRegister as u8
        };

        let addr_split = shared::split_addr(r);

        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self
            .spi
            .write(&[c, addr_split.0, addr_split.1, word])
            .is_err()
        {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.pins.cs.set_high();

        Ok(())
    }

    /// Common to 8-bit and 16-bit reads.
    fn read_reg_common(&mut self, reg: Register) -> Result<[u8; 6], RadioError> {
        let r = match reg {
            Register::Reg6x(reg) => reg as u16,
            Register::Reg8x(reg) => reg as u16,
        };

        let c = if self.r8x {
            OpCode::ReadRegister.val_8x()
        } else {
            OpCode::ReadRegister as u8
        };

        let addr_split = shared::split_addr(r);

        let mut read_buf = [0; 6];

        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self
            .spi
            .transfer(&mut read_buf, &[c, addr_split.0, addr_split.1, 0, 0, 0])
            .is_err()
        {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.pins.cs.set_high();

        Ok(read_buf)
    }

    /// Read a single 8-bit word from a register.
    pub fn read_reg_word(&mut self, reg: Register) -> Result<u8, RadioError> {
        Ok(self.read_reg_common(reg)?[4])
    }

    /// Read a single 16-bit word from a register.
    pub fn read_reg_word_16(&mut self, reg: Register) -> Result<u16, RadioError> {
        let buf = self.read_reg_common(reg)?;
        Ok(u16::from_be_bytes([buf[4], buf[5]]))
    }

    /// Write a buffer to the radio.
    pub fn write(&mut self, write_buffer: &[u8]) -> Result<(), RadioError> {
        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self.spi.write(write_buffer).is_err() {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        }
        self.pins.cs.set_high();

        Ok(())
    }

    /// Write with a payload; uses DMA. Must clean up the transaction in an ISR. See note on offsets in
    /// `read_with_payload`.
    pub fn write_with_payload(&mut self, payload: &[u8], offset: u8) -> Result<(), RadioError> {
        let c = if self.r8x {
            OpCode::WriteBuffer.val_8x()
        } else {
            OpCode::WriteBuffer as u8
        };
        self.write_buf[0] = c;
        self.write_buf[1] = offset;

        for (i, word) in payload.iter().enumerate() {
            self.write_buf[i + 2] = *word;
        }

        self.wait_on_busy()?;

        println!("Writing DMA..."); // todo: Temp
        self.pins.cs.set_low().unwrap();
        if self.spi.write(&self.write_buf[..2 + payload.len()]).is_err() {
            return Err(RadioError::Spi);
        }

        Ok(())
    }

    /// Read a payload; uses DMA.
    /// "Before any read or write operation it is hence necessary to initialize this offset to the corresponding beginning of the buffer.
    /// Upon reading or writing to the data buffer the address pointer will then increment automatically."
    pub fn read_with_payload(&mut self, payload_len: u8, offset: u8) -> Result<(), RadioError> {
        let c = if self.r8x {
            OpCode::ReadBuffer.val_8x()
        } else {
            OpCode::ReadBuffer as u8
        };

        self.write_buf[0] = c;
        self.write_buf[1] = offset;
        // "Note that the NOP must be sent after sending the offset."
        self.write_buf[2] = 0;

        // DS, Table 13-27: ReadBuffer SPI Transaction: Payload starts at byte 3.
        let buf_end = payload_len as usize + 3;
        // let buf_end = 4; // todo  t

        self.wait_on_busy()?;

        println!("Reading DMA..."); // todo: temp
        self.pins.cs.set_low().unwrap();
        if self.spi.transfer(
            &mut self.read_buf[0..buf_end],
            &self.write_buf[0..buf_end],
        ).is_err() {
            return Err(RadioError::Spi);
        }

        Ok(())
    }

    /// Request a read, filling the provided buffer.
    pub fn read(&mut self, buffer: &mut [u8]) -> Result<(), RadioError> {
        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self.spi.read(buffer).is_err() {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        }
        self.pins.cs.set_high();

        Ok(())
    }

    /// // DS, Table 13-27: ReadBuffer SPI Transaction: Payload starts at byte 3, using the radio's API.
    /// Mutable for use with encryption.
    /// TODO: move to radio mod.
    // fn rx_payload_from_buf(&mut self) -> &'static mut [u8] {
    pub fn rx_payload_from_buf(&mut self) -> &mut [u8] {
        // fn rx_payload_from_buf() -> &'static [u8] {
        // Note: This is the payload length as reported by the radio.
        // let payload_len = self.rx_payload_len as usize;
        // let payload_start = self.rx_payload_start as usize;
        // let payload_start_i = 3 + payload_start;

        // todo: This may not be accurate: We may have handled payload start i when creating read_buf.
        // &mut self.read_buf[payload_start_i..payload_start_i + payload_len]

        &mut self.read_buf[..self.rx_payload_len as usize]
    }

    //
    // /// Request a read, filling the provided buffer.
    // pub fn read(&mut self, write_buffer: &[u8], read_buffer: &mut [u8]) -> Result<(), RadioError> {
    //     self.wait_on_busy()?;
    //
    //     self.pins.cs.set_low();
    //     if self.spi.transfer_type2(write_buffer, read_buffer).is_err() {
    //         self.pins.cs.set_high();
    //         return Err(RadioError::SPI);
    //     }
    //     self.pins.cs.set_high();
    //
    //     Ok(())
    // }
}
