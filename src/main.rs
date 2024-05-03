//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;
use rp2040_hal::pio::PIOExt;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionPio0, Pin},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let mut led_pin = pins.led.into_push_pull_output();
    let pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio3,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio3.into_function();

    let program_with_defines = pio_proc::pio_file!(
        "eloy.pio",
        select_program("uart_rx"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let (mut sm, mut rx, _) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .buffers(rp_pico::hal::pio::Buffers::OnlyRx)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .autopush(false)
            .push_threshold(32)
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .in_pin_base(pin.id().num)
            .jmp_pin(pin.id().num)
            .set_pins(0, 0)
            .out_pins(0, 0)
            .build(sm0);

    // Not sure yet what is this divisor doing.
    sm.set_clock_divisor(125_000_000f32 / (8f32 * 115200f32));
    sm.set_pindirs([(pin.id().num, rp_pico::hal::pio::PinDir::Input)]);

    sm.start();

    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        if let Some(channel) = rx.read() {
            //@TODO: I think that it should be to_ne_bytes()
            channel.to_le_bytes().iter().for_each(|channel| {
                if *channel != 0 {
                    info!("new info: {}", &channel.to_le_bytes());
                }
            });
        }
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
