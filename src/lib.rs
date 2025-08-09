#![no_std]

use pio::pio_asm;
use rp2040_hal as hal;
use rp2040_hal::dma::double_buffer::{Transfer, WriteNext};
use rp2040_hal::dma::{double_buffer, SingleChannel};
use rp2040_hal::pio::{PIOExt, Rx, ShiftDirection, StateMachineIndex, UninitStateMachine, PIO};

pub enum DriverMode {
    MonoLeft,
    MonoRight,
    Stereo,
}

pub enum DriverError {
    NotInitialized,
}

pub struct INMP441<
    P: PIOExt,
    S: StateMachineIndex,
    CH1: SingleChannel,
    CH2: SingleChannel,
    const N: usize,
> {
    dma_transfer: Option<
        Transfer<CH1, CH2, Rx<(P, S)>, &'static mut [u32; N], WriteNext<&'static mut [u32; N]>>,
    >,
    i2s_clock_pin: u8,
    i2s_ws_pin: u8,
    i2s_data_pin: u8,
    system_clock_freq: u32,
    sampling_rate: u32,
}

impl<P: PIOExt, S: StateMachineIndex, CH1: SingleChannel, CH2: SingleChannel, const N: usize>
    INMP441<P, S, CH1, CH2, N>
{
    pub fn new(
        clck_pin: u8,
        ws_pin: u8,
        data_pin: u8,
        system_clock_freq: u32,
        sampling_rate: u32,
    ) -> INMP441<P, S, CH1, CH2, N> {
        INMP441 {
            dma_transfer: None,
            i2s_clock_pin: clck_pin,
            i2s_ws_pin: ws_pin,
            i2s_data_pin: data_pin,
            system_clock_freq,
            sampling_rate,
        }
    }

    pub fn init(
        &mut self,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, S)>,
        dma_channel_1: CH1,
        dma_channel_2: CH2,
        rx_buf_1: &'static mut [u32; N],
        rx_buf_2: &'static mut [u32; N],
    ) {
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
        let (int, frac) =
            Self::calculate_pio_clock_dividers(self.system_clock_freq, self.sampling_rate);
        let (mut sm, rx, _) = hal::pio::PIOBuilder::from_installed_program(installed)
            .side_set_pin_base(self.i2s_ws_pin)
            .in_pin_base(self.i2s_data_pin)
            .clock_divisor_fixed_point(int, frac)
            .in_shift_direction(ShiftDirection::Left)
            .build(sm);
        sm.set_pindirs([
            (self.i2s_clock_pin, hal::pio::PinDir::Output),
            (self.i2s_ws_pin, hal::pio::PinDir::Output),
            (self.i2s_data_pin, hal::pio::PinDir::Input),
        ]);
        let rx_transfer =
            double_buffer::Config::new((dma_channel_1, dma_channel_2), rx, rx_buf_1).start();
        let rx_transfer = rx_transfer.write_next(rx_buf_2);
        sm.start();
        self.dma_transfer = Some(rx_transfer);
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

    fn calculate_pio_clock_dividers(system_clock_hz: u32, sample_rate: u32) -> (u16, u8) {
        // formula for calculating int and frac:
        // system_clock_freq / ((int + frac/256) * 64 * 2) = sample_rate
        // every pio program cycle needs 64 i2s clock cycle (32 for left channel + 32 for right channel)
        // every i2s clock cycle - 2 pio program instructions (1 clock pin low + 1 clock pin high)
        let combined = system_clock_hz as f32 / (sample_rate as f32 * 64. * 2.);
        let int = combined as u16;
        let frac = ((combined - int as f32) * 256.) as u8;
        (int, frac)
    }

    pub fn ready(&mut self) -> bool {
        if let Some(transfer) = &self.dma_transfer {
            if transfer.is_done() {
                return true;
            }
        }
        false
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
