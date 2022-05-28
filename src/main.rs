//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_RAM_MEMCPY;

// #[entry]
// fn main() -> ! {
//     info!("Program start");
//     let mut pac = pac::Peripherals::take().unwrap();
//     let core = pac::CorePeripherals::take().unwrap();
//     let mut watchdog = Watchdog::new(pac.WATCHDOG);
//     let sio = Sio::new(pac.SIO);

//     // External high-speed crystal on the pico board is 12Mhz
//     let external_xtal_freq_hz = 12_000_000u32;
//     let clocks = init_clocks_and_plls(
//         external_xtal_freq_hz,
//         pac.XOSC,
//         pac.CLOCKS,
//         pac.PLL_SYS,
//         pac.PLL_USB,
//         &mut pac.RESETS,
//         &mut watchdog,
//     )
//     .ok()
//     .unwrap();

//     let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

//     let pins = bsp::Pins::new(
//         pac.IO_BANK0,
//         pac.PADS_BANK0,
//         sio.gpio_bank0,
//         &mut pac.RESETS,
//     );

//     let mut led_pin = pins.led.into_push_pull_output();

//     loop {
//         info!("on!");
//         led_pin.set_high().unwrap();
//         delay.delay_ms(500);
//         info!("off!");
//         led_pin.set_low().unwrap();
//         delay.delay_ms(500);
//     }
// }

// https://crates.io/crates/switch-hal

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1, PIO1_IRQ_0])]
mod app {
    use crate::hal;

    use cortex_m::prelude::{
        _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogEnable,
    };

    use embedded_hal::digital::v2::{InputPin, OutputPin};

    use embedded_time::duration::units::*;

    use hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        sio::Sio,
        watchdog::Watchdog,
    };

    const TIMER_INTERVAL: u32 = 1000;

    #[shared]
    struct Shared {
        timer: hal::timer::Timer,
        alarm: hal::timer::Alarm0,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = ctx.device.RESETS;
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        let clocks = init_clocks_and_plls(
            12_000_000u32,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let mut timer = hal::Timer::new(ctx.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        alarm.schedule(TIMER_INTERVAL.microseconds()).ok();
        alarm.enable_interrupt();

        let sio = Sio::new(ctx.device.SIO);
        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut led = pins.gpio25.into_readable_output();

        // Debug LEDs
        // let green = pins.gpio16.into_readable_output();
        // let red = pins.gpio17.into_readable_output();
        // let blue = pins.gpio25.into_readable_output();

        // neopixel
        let pixel_power = pins.gpio11;
        let pixel_data = pins.gpio12;

        // watchdog with low priority of 1kHz
        watchdog.start(10_000.microseconds());

        (Shared { timer, alarm }, Local {}, init::Monotonics())
    }

    #[idle()]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfe();
        }
    }

    #[task(shared = [], local=[counter: u8 = 0, prev: u8 = 0], priority = 1)]
    fn blink(mut ctx: blink::Context) {}
}
