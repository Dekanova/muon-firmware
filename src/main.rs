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

// https://crates.io/crates/switch-hal

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1, PIO1_IRQ_0])]
mod app {
    use crate::hal;
    use cortex_m::prelude::*;
    use hal::prelude::*;
    use hal::{gpio::bank0::Gpio12, timer::CountDown};

    use embedded_hal::digital::v2::{InputPin, OutputPin};

    use embedded_time::duration::units::*;

    use hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        sio::Sio,
        timer::Timer,
        watchdog::Watchdog,
    };

    use rtic::RacyCell;
    use smart_leds::{brightness, SmartLedsWrite, RGB8};
    use ws2812_pio::Ws2812;

    const TIMER_INTERVAL: u32 = 1000;

    #[shared]
    struct Shared {
        timer: &'static Timer,
        alarm: hal::timer::Alarm0,
        ws: Ws2812<hal::pac::PIO0, hal::pio::SM0, CountDown<'static>, Gpio12>,
    }

    #[local]
    struct Local {}

    #[init(local = [flash: Option<Timer> = None])]
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
        // move timer into static lifetime early so Ws2812 can use it
        let timer = ctx.local.flash.insert(timer);

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

        let (mut pio, sm0, _, _, _) = hal::pio::PIOExt::split(ctx.device.PIO0, &mut resets);

        let mut ws = Ws2812::new(
            pixel_data.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
            timer.count_down(),
        );

        // watchdog with low priority of 1kHz
        cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable::start(
            &mut watchdog,
            10_000.microseconds(),
        );

        defmt::info!("init finished");
        (Shared { timer, alarm, ws }, Local {}, init::Monotonics())
    }

    #[idle(shared = [ws, timer])]
    fn idle(mut ctx: idle::Context) -> ! {
        let mut delay = ctx.shared.timer.lock(|t| t.count_down());

        let mut n = 128;
        defmt::info!("idle entry");

        loop {
            // cortex_m::asm::wfe();

            ctx.shared.ws.lock(|ws| {
                ws.write(brightness(core::iter::once(wheel(n)), 32))
                    .unwrap();
            });
            n = n.wrapping_add(1);
            delay.start(25.milliseconds());
            let _ = nb::block!(delay.wait());
        }
    }

    #[task(shared = [], local=[counter: u8 = 0, prev: u8 = 0], priority = 1)]
    fn blink(ctx: blink::Context) {}

    /// Convert a number from `0..=255` to an RGB color triplet.
    ///
    /// The colours are a transition from red, to green, to blue and back to red.
    fn wheel(mut wheel_pos: u8) -> RGB8 {
        wheel_pos = 255 - wheel_pos;
        if wheel_pos < 85 {
            // No green in this sector - red and blue only
            (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
        } else if wheel_pos < 170 {
            // No red in this sector - green and blue only
            wheel_pos -= 85;
            (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
        } else {
            // No blue in this sector - red and green only
            wheel_pos -= 170;
            (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
        }
    }
}
