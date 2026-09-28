use crate::mode::{Async, Mode};
use crate::pio_programs::clock_divider::calculate_pio_clock_divider;
use crate::{Peri, dma, interrupt};
use crate::gpio::Pull;
use crate::pio::{Common, Instance, Pin, PioPin, ShiftDirection, StateMachine};
use crate::pio::LoadedProgram;

/// Error type for I2C operations 
pub enum Error {
    /// Address NACK received from slave
    AddressNack,
    /// Data NACK received from slavePio
    DataNack,
    /// Timeout waiting for slave to respond
    Timeout,
}

/// This struct represents an i2c program loaded into pio instruction memory.
pub struct PioI2cProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioI2cProgram<'d, PIO> {
    /// Load the i2c program into the given pio
    pub fn new(common: &mut Common<'d, PIO>, sda: &Pin<'d, PIO>, scl: &Pin<'d,PIO>) -> Self {
        let mut prg = pio::pio_asm!(
            r#"
                .side_set 1 opt pindirs

                byte_nack:
                    jmp  y--     byte_end  ; continue if NAK was expected
                    irq  wait    0    rel  ; otherwise stop, ask for help
                    jmp          byte_end  ; resumed, finalize the current byte

                byte_send:
                    out  y       1         ; Unpack FINAL
                    set  x       7         ; loop 8 times

                bitloop:
                    out  pindirs 1                [7] ; Serialize write data
                    nop                    side 1 [2] ; SCL rising edge
                    wait 1       gpio 0           [4] ; Allow clock to be stretched
                    in   pins 1                   [7] ; Sample read data
                    jmp  x--     bitloop   side 0 [7] ; SCL falling edge

                ; Handle ACK pulse
                    out  pindirs 1                [7] ; Provide ACK on reads
                    nop                    side 1 [7] ; SCL rising edge
                    wait 1       gpio 0           [7] ; Allow clock to be stretched
                    jmp  pin     byte_nack side 0 [2] ; Test SDA for ACK/NACK

                byte_end:
                    push block             ; flush ISR to RX FIFO

                .wrap_target
                    out  x       6         ; Unpack Instr count
                    jmp  !x      byte_send ; Instr == 0, data record
                    out  null    10        ; Instr > 0, remainder of OSR invalid

                do_exec:
                    out  exec    16        ; Execute dynamic command words
                    jmp  x--     do_exec
                .wrap
                "#
        );

    let scl_pin_num = scl.pin() as u16;
    
    prg.program.code[7] = scl_pin_num;
    prg.program.code[12] = scl_pin_num;

    let prg = common.load_program(&prg.program);
    Self { prg }
    }
}

/// PIO backed I2C driver
pub struct I2c<'d, PIO: Instance, const SM: usize, M: Mode> {
    sm: StateMachine<'d, PIO, SM>,
    dma_tx: Option<dma::Channel<'d, Async>>,
    dma_rx: Option<dma::Channel<'d, Async>>,
    _mode: core::marker::PhantomData<M>,
}

impl <'d, PIO: Instance, const SM:usize, M:Mode> I2c<'d, PIO, SM, M> {
    fn new_inner(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        sda: Peri<'d, impl PioPin>,
        scl: Peri<'d, impl PioPin>,
        dma_tx:  Option<dma::Channel<'d, Async>>,
        dma_rx:  Option<dma::Channel<'d, Async>>,
        freq: u32
    ) -> Self {
        // Pin configuration with internal pull-ups enabled
        let mut sda_pin = common.make_pio_pin(sda);
        let mut scl_pin = common.make_pio_pin(scl);
        sda_pin.set_pull(Pull::Up);
        scl_pin.set_pull(Pull::Up);

        let program = PioI2cProgram::new(common, &sda_pin, &scl_pin);

        let mut cfg = crate::pio::Config::default();
        cfg.use_program(&program.prg, &[&scl_pin]); // Side-set base pin = SCL
        cfg.set_out_pins(&[&sda_pin]); 
        cfg.set_in_pins(&[&sda_pin]);
        cfg.set_set_pins(&[&sda_pin]); 

        cfg.shift_out = crate::pio::ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Left,
            threshold: 8,
        };
        cfg.shift_in = crate::pio::ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Left,
            threshold: 8,
        };
        cfg.clock_divider = calculate_pio_clock_divider(32*freq);
        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            sm,
            dma_tx: dma_tx,
            dma_rx: dma_rx,
            _mode: core::marker::PhantomData,
        }
    }
    /// Set i2c frequency on runtime. This will stop the state machine, reconfigure it and restart it.
    pub fn set_frequency(&mut self, freq: u32) {
        let clock_divider = calculate_pio_clock_divider(32*freq);
        self.sm.set_enable(false);
        self.sm.clear_fifos();
        self.sm.restart();
        self.sm.set_clock_divider(clock_divider);
        self.sm.set_enable(true);
    }
}
    


impl<'d, PIO: Instance, const SM: usize> I2c<'d, PIO, SM, Async> {
    /// Create an I2c driver in async mode supporting DMA operations.
    #[allow(clippy::too_many_arguments)]
    pub fn new<TxDma: dma::ChannelInstance, RxDma: dma::ChannelInstance>(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        freq:u32,
        sda: Peri<'d, impl PioPin>,
        scl: Peri<'d, impl PioPin>,
        tx_dma: Peri<'d, TxDma>,
        rx_dma: Peri<'d, RxDma>,
        irq: impl interrupt::typelevel::Binding<TxDma::Interrupt, dma::InterruptHandler<TxDma>>
        + interrupt::typelevel::Binding<RxDma::Interrupt, dma::InterruptHandler<RxDma>>
        + 'd,
    ) -> Self {
        let tx_dma_ch = dma::Channel::new(tx_dma, irq);
        let rx_dma_ch = dma::Channel::new(rx_dma, irq);
        Self::new_inner(common, sm, sda, scl, Some(tx_dma_ch), Some(rx_dma_ch), freq)
    }
    /// Reads bytes from a target device into `buffer`.
    pub async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.send_start().await?;
        self.send_byte((address << 1) | 1, false, false).await?; // Read address frame
        
        let len = buffer.len();
        for (i, byte) in buffer.iter_mut().enumerate() {
            let is_last = i == len - 1;
            // Send dummy byte 0xFF to trigger clock cycles for receiving
            *byte = self.send_byte(0xFF, is_last, is_last).await?;
        }

        self.send_stop().await?;
        Ok(())
    }

    /// Writes `bytes` to a target device.
    pub async fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        self.send_start().await?;
        self.send_byte(address << 1, false, false).await?; // Write address frame

        let len = bytes.len();
        for (i, &byte) in bytes.iter().enumerate() {
            let is_last = i == len - 1;
            self.send_byte(byte, false, is_last).await?;
        }

        self.send_stop().await?;
        Ok(())
    }

    /// Writes `bytes` and then reads into `buffer` with a repeated START condition.
    pub async fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        // Write phase (without sending STOP at the end)
        self.send_start().await?;
        self.send_byte(address << 1, false, false).await?;
        for &byte in bytes {
            self.send_byte(byte, false, false).await?;
        }

        // Repeated START + Read phase
        self.send_start().await?;
        self.send_byte((address << 1) | 1, false, false).await?;
        
        let len = buffer.len();
        for (i, byte) in buffer.iter_mut().enumerate() {
            let is_last = i == len - 1;
            *byte = self.send_byte(0xFF, is_last, is_last).await?;
        }

        self.send_stop().await?;
        Ok(())
    }

    /// Reads bytes from a specific 8-bit register on the target device.
    pub async fn read_reg(
        &mut self,
        address: u8,
        reg: u8,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.write_read(address, &[reg], buffer).await
    }

    /// Writes a single byte payload to a specific 8-bit register on the target device.
    pub async fn write_reg(&mut self, address: u8, reg: u8, byte: u8) -> Result<(), Error> {
        self.write(address, &[reg, byte]).await
    }

    // --- Private Low-Level Helpers to drive the PIO Assembly ---

    /// Helper to encode dynamic PIO instructions into the FIFO
    async fn push_instr(&mut self, instrs: &[u16]) {
        let count = instrs.len() as u16;
        let header = (count << 10) as u16; 
        
        let mut tx_ch = self.dma_tx.as_mut().unwrap().reborrow();
        self.sm.tx().dma_push(&mut tx_ch, &[header], false).await;
        self.sm.tx().dma_push(&mut tx_ch, instrs, false).await;
        
    }
    

    /// Issue I2C START condition via dynamic EXEC instructions
    async fn send_start(&mut self) -> Result<(), Error> {
        // Dynamic instructions: drive SDA low while SCL is high
        let instrs = [
            0x0000, // set pindirs, 0 (SDA high)
            0x0001, // set pindirs, 1 (SDA low -> START)
        ];
        self.push_instr(&instrs).await;
        Ok(())
    }

    /// Issue I2C STOP condition via dynamic EXEC instructions
    async fn send_stop(&mut self) -> Result<(), Error> {
        let instrs = [
            0x0001, // set pindirs, 1 (SDA low)
            0x0000, // set pindirs, 0 (SDA high -> STOP)
        ];
        self.push_instr(&instrs).await;
        Ok(())
    }

    /// Sends a single byte and pulls the sampled ACK/data byte back from RX FIFO
    async fn send_byte(&mut self, byte: u8, nak_expected: bool, final_byte: bool) -> Result<u8, Error> {
        // Frame encoding for OSR unpacking in assembly:
        // [6 bits count (0 = data)] [1 bit NAK] [1 bit FINAL] [8 bits DATA]
        let nak_bit = if nak_expected { 1u16 } else { 0u16 };
        let final_bit = if final_byte { 1u16 } else { 0u16 };
        
        let word: u16 = (0 << 10) | (nak_bit << 9) | (final_bit << 8) | (byte as u16);

        // Push data command word to TX FIFO
        
        let mut tx_ch = self.dma_tx.as_mut().unwrap().reborrow();
        self.sm.tx().dma_push(&mut tx_ch, &[word], false).await;

        // Read sampled byte back from RX FIFO
        let mut rx_ch = self.dma_rx.as_mut().unwrap().reborrow();
        let mut rx_buf = [0u8; 1];
        self.sm.rx().dma_pull(&mut rx_ch, &mut rx_buf, false).await;

        Ok(rx_buf[0])
    }
}
