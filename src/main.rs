//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use defmt_rtt as _;
// use panic_halt as _;
use panic_probe as _;

use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// https://crates.io/crates/switch-hal

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])] // extra PIO0_IRQ_1, PIO1_IRQ_0
mod app {
    use crate::hal;
    // use cortex_m::prelude::*;
    // use hal::prelude::*;
    use defmt::*;
    // use hal::{gpio::bank0::Gpio12, timer::CountDown};

    use embedded_hal::digital::v2::{InputPin, OutputPin};

    use embedded_time::{duration::units::*, fixed_point::FixedPoint};

    use hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::*,
        pac,
        sio::Sio,
        timer::Timer,
        watchdog::Watchdog,
    };
    use rp2040_monotonic::*;

    // use smart_leds::{brightness, SmartLedsWrite, RGB8};
    // use ws2812_pio::Ws2812;

    // const TIMER_INTERVAL: u32 = 1000;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Rp2040Monotonic;

    #[shared]
    struct Shared {
        // alarm: hal::timer::Alarm0,
        // ws: Ws2812<hal::pac::PIO0, hal::pio::SM0, CountDown<'static>, Gpio12>,
    }

    #[local]
    struct Local {
        debug_led: Pin<bank0::Gpio25, Output<PushPull>>,
        debug_delay: cortex_m::delay::Delay,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // info!("init start");
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

        let monotonic = Rp2040Monotonic::new(ctx.device.TIMER);
        // let mut alarm = timer.alarm_0().unwrap();
        // alarm.schedule(TIMER_INTERVAL.microseconds()).ok();
        // TODO
        // alarm.enable_interrupt();

        // move timer into static lifetime early so Ws2812 can use it
        // TODO reput this in

        let debug_delay =
            cortex_m::delay::Delay::new(ctx.core.SYST, clocks.system_clock.freq().integer());

        let sio = Sio::new(ctx.device.SIO);
        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Debug LEDs
        // let green = pins.gpio16.into_readable_output();
        // let red = pins.gpio17.into_readable_output();
        let blue = pins.gpio25.into_push_pull_output();

        // // neopixel
        // let pixel_power = pins.gpio11;
        // let pixel_data = pins.gpio12;

        // let (mut pio, sm0, _, _, _) = hal::pio::PIOExt::split(ctx.device.PIO0, &mut resets);

        // let mut ws = Ws2812::new(
        //     pixel_data.into_mode(),
        //     &mut pio,
        //     sm0,
        //     clocks.peripheral_clock.freq(),
        //     timer.count_down(),
        // );

        tick::spawn().ok();
        // // watchdog with low priority of 1kHz
        // cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable::start(
        //     &mut watchdog,
        //     1_000.microseconds(),
        // );

        info!("init finished");
        (
            Shared {},
            Local {
                debug_led: blue,
                debug_delay,
            },
            init::Monotonics(monotonic),
        )
    }

    #[idle(shared = [], local = [debug_led, debug_delay])]
    fn idle(ctx: idle::Context) -> ! {
        let delay = ctx.local.debug_delay;
        let debug_led = ctx.local.debug_led;
        loop {
            info!("on!");
            debug_led.set_high().unwrap();
            delay.delay_ms(500);
            info!("off!");
            debug_led.set_low().unwrap();
            delay.delay_ms(500);
        }
    }

    #[task]
    fn tick(_: tick::Context) {
        info!("Tick");
        tick::spawn_after(1_000_u64.millis()).ok();
    }

    // #[task(shared = [], local=[counter: u8 = 0, prev: u8 = 0], priority = 1)]
    // fn blink(ctx: blink::Context) {}

    // /// Convert a number from `0..=255` to an RGB color triplet.
    // ///
    // /// The colours are a transition from red, to green, to blue and back to red.
    // fn wheel(mut wheel_pos: u8) -> RGB8 {
    //     wheel_pos = 255 - wheel_pos;
    //     if wheel_pos < 85 {
    //         // No green in this sector - red and blue only
    //         (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    //     } else if wheel_pos < 170 {
    //         // No red in this sector - green and blue only
    //         wheel_pos -= 85;
    //         (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    //     } else {
    //         // No blue in this sector - red and green only
    //         wheel_pos -= 170;
    //         (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    //     }
    // }
}
