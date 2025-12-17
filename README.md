# RP2040-INMP441

INMP441 mems microphone driver for RP2040 series microcontrollers.

## Description

Driver for receiving data from INMP441 mems microphone. Supports both connecting
single mic (left or right) or both microphone in stereo mode.
Should also work with any other 24-bit I2S microphone, but was tested 
only with INMP441. Driver uses RP2040 PIO to implement I2S protocol and
DMA to transfer data from PIO.

## Usage
```rust
// configuring i2s mic pins
let i2s_clock_pin: Pin<_, FunctionPio0, _> = pins.gpio28.into_function();
let i2s_ws_pin: Pin<_, FunctionPio0, _> = pins.gpio27.into_function();
let i2s_sd_pin: Pin<_, FunctionPio0, _> = pins.gpio26.into_function();
// PIN id for use inside of PIO
let i2s_clock_pin_id = i2s_clock_pin.id().num;
let i2s_ws_pin_id = i2s_ws_pin.id().num;
let i2s_sd_pin_id = i2s_sd_pin.id().num;

// Initialize and start PIO
let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

// configuring dma
let dma = pac.DMA.split(&mut pac.RESETS);
let rx_buf_1 = singleton!(: [u32; 1024] = [0; 1024]).unwrap();
let rx_buf_2 = singleton!(: [u32; 1024] = [0; 1024]).unwrap();

let mut inmp441 = INMP441::new(
    i2s_clock_pin_id,
    i2s_ws_pin_id,
    i2s_sd_pin_id,
    125_000_000,
    48_000,
    DriverMode::Stereo,
).unwrap();
inmp441.init(&mut pio, sm0, dma.ch0, dma.ch1, rx_buf_1, rx_buf_2);

// buffers for audio samples
let mut samples_buffer: [i32; 1024] = [0; 1024];
loop {
    if inmp441.ready().unwrap() {
        inmp441.read(&mut samples_buffer).unwrap();
    }
}
```