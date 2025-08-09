#![no_std]

use cortex_m::singleton;
use pio::pio_asm;
use rp2040_hal as hal;
use rp2040_hal::dma::double_buffer::{Transfer, WriteNext};
use rp2040_hal::dma::{double_buffer, SingleChannel};
use rp2040_hal::pio::{
    PIOExt, Rx, ShiftDirection, StateMachineIndex, UninitStateMachine, PIO,
};

pub struct INMP441<P: PIOExt, S: StateMachineIndex, CH1: SingleChannel, CH2: SingleChannel>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
{
    dma_transfer: Option<
        Transfer<
            CH1,
            CH2,
            Rx<(P, S)>,
            &'static mut [u32; 1024],
            WriteNext<&'static mut [u32; 1024]>,
        >,
    >,
}

impl<P: PIOExt, S: StateMachineIndex, CH1: SingleChannel, CH2: SingleChannel>
    INMP441<P, S, CH1, CH2>
{
    pub fn new(
        clck_pin: u8,
        ws_pin: u8,
        data_pin: u8,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, S)>,
        dma_channel_1: CH1,
        dma_channel_2: CH2,
    ) -> INMP441<P, S, CH1, CH2> {
        let program = pio_asm!(
            // 0 | 00: ws 0 clock 0
            // 1 | 01: ws 1 clock 0
            // 2 | 10: ws 0 clock 1
            // 3 | 11: ws 1 clock 1
            ".side_set 2", // each instruction must set 1 bit
            ".wrap_target",
            "set y 30 side 0" // set bit counter clock and ws 0
            "push noblock side 2" // pushing right channel data
            "lp1:"
            "  in pins 1 side 0"
            "  jmp y-- lp1 side 2"
            "set y 30 side 1" // 31 minus 2 cycles (1 for pushing data, one for clock sync)
            "push noblock side 3" // pushing left channel data
            "lp2:"
            "  in pins 1 side 1"
            "  jmp y-- lp2 side 3"
            ".wrap",
        );
        let installed = pio.install(&program.program).unwrap();
        let (int, frac) = (20, 88); // 125_000_000 / ((20 + 88/256) * 64 * 2) = 48000 (well almost)
        let (mut sm, rx, _) = hal::pio::PIOBuilder::from_installed_program(installed)
            .side_set_pin_base(ws_pin)
            .in_pin_base(data_pin)
            .clock_divisor_fixed_point(int, frac)
            .in_shift_direction(ShiftDirection::Left)
            .build(sm);
        sm.set_pindirs([
            (clck_pin, hal::pio::PinDir::Output),
            (ws_pin, hal::pio::PinDir::Output),
            (data_pin, hal::pio::PinDir::Input),
        ]);
        let rx_buf_1 = singleton!(: [u32; 1024] = [0; 1024]).unwrap();
        let rx_buf_2 = singleton!(: [u32; 1024] = [0; 1024]).unwrap();
        let rx_transfer =
            double_buffer::Config::new((dma_channel_1, dma_channel_2), rx, rx_buf_1).start();
        let rx_transfer = rx_transfer.write_next(rx_buf_2);
        sm.start();
        INMP441 {
            dma_transfer: Some(rx_transfer),
        }
    }

    pub fn read(&mut self, buffer: &mut [i32; 1024]) {
        let transfer = self.dma_transfer.take().unwrap();
        if transfer.is_done() {
            let (rx_buf, next_rx_transfer) = transfer.wait();
            for i in 0..rx_buf.len() {
                let sample = Self::convert_i24_to_i32(rx_buf[i]);
                buffer[i] = sample;
            }
            let rx_transfer = next_rx_transfer.write_next(rx_buf);
            self.dma_transfer = Some(rx_transfer);
        } else {
            self.dma_transfer = Some(transfer);
        }
    }

    pub fn ready(&mut self) -> bool {
        let mut result = false;
        let transfer = self.dma_transfer.take().unwrap();
        if transfer.is_done() {
            result = true;
        };
        self.dma_transfer = Some(transfer);
        result
    }

    fn convert_i24_to_i32(reading: u32) -> i32 {
        // every reading from queue retrieves u32 value with only 24 msb's containing actual data
        // function translates it to i32 integer
        // removing unused bits
        let bits = reading >> 6;
        // checking if value is negative by comparing msb to 1
        match (bits & (1 << 23)) == 1 << 23 {
            true => (0b11111111 << 24 | bits) as i32,
            false => bits as i32,
        }
    }
}
