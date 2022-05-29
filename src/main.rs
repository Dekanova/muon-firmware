//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use defmt_rtt as _;
use panic_probe as _;
// use panic_halt as _;

use rp2040_hal as hal;

mod kb;

// TODO https://crates.io/crates/switch-hal
#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1])] // extra PIO0_IRQ_1, PIO1_IRQ_0
mod app {
    use crate::hal;
    use cortex_m::prelude::*;
    // use hal::prelude::*;
    use defmt::*;
    use hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::*,
        pac,
        sio::Sio,
        timer::Timer,
        usb::UsbBus,
        watchdog::Watchdog,
    };
    use hal::{gpio::bank0::Gpio12, timer::CountDown};

    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use embedded_time::{duration::units::*, fixed_point::FixedPoint};

    use usb_device::class_prelude::*;

    use smart_leds::{brightness, SmartLedsWrite, RGB8};
    use ws2812_pio::Ws2812Direct as Ws2812;

    use keyberon::{debounce::Debouncer, hid::HidClass, matrix::Matrix};

    const TIMER_INTERVAL: u32 = 1000;

    // use rp2040_monotonic::*;
    use systick_monotonic::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>;

    #[shared]
    struct Shared {
        // alarm: hal::timer::Alarm0,
        #[lock_free]
        timer: &'static hal::timer::Timer,
        ws: Ws2812<hal::pac::PIO0, hal::pio::SM0, Gpio12>,
        usb_dev: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_class: keyberon::Class<'static, UsbBus, crate::kb::Leds>,
        // #[lock_free]
        // debouncer: Debouncer<PressedKeys<16, 5>>,
    }

    #[local]
    struct Local {
        debug_led: Pin<bank0::Gpio25, Output<PushPull>>,
    }

    #[init(local = [TIMER: Option<hal::timer::Timer> = None, USB: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        info!("init start");
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

        // move timer into static lifetime early so Ws2812 can use it
        let mut timer = hal::Timer::new(ctx.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(TIMER_INTERVAL.microseconds());
        // alarm.enable_interrupt();

        // TODO reput this in
        let timer = ctx.local.TIMER.insert(timer);

        let sio = Sio::new(ctx.device.SIO);
        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Matric whoOhhoH -------------
        // Matrix::new();

        // gpio2.into_pull_up_input().into();

        // Debug LEDs
        // let green = pins.gpio16.into_readable_output();
        // let red = pins.gpio17.into_readable_output();
        let mut blue = pins.gpio25.into_push_pull_output();

        // // neopixel
        let mut pixel_power = pins.gpio11.into_push_pull_output();

        pixel_power.set_high();

        let pixel_data = pins.gpio12;

        let (mut pio, sm0, _, _, _) = hal::pio::PIOExt::split(ctx.device.PIO0, &mut resets);

        // LED
        let mut ws = Ws2812::new(
            pixel_data.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            ctx.device.USBCTRL_REGS,
            ctx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        let usb_bus = ctx.local.USB.insert(usb_bus);

        let usb_class = keyberon::new_class(usb_bus, crate::kb::Leds);
        let usb_dev = keyberon::new_device(usb_bus);

        tick::spawn().ok(); // TODO for debug, remove
        led_color_wheel::spawn().ok();

        let mono = Systick::new(ctx.core.SYST, clocks.system_clock.freq().0);
        // let mono = Rp2040Monotonic::new(ctx.device.TIMER);

        // TODO this is causing issues and IDK why yet
        // // watchdog with low priority of 1kHz
        // cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable::start(
        //     &mut watchdog,
        //     10_000.microseconds(),
        // );

        info!("init finished");
        (
            Shared {
                usb_dev,
                usb_class,
                timer,
                ws,
            },
            Local { debug_led: blue },
            init::Monotonics(mono),
        )
    }

    #[idle(shared = [timer], local = [debug_led])]
    fn idle(ctx: idle::Context) -> ! {
        let mut delay = ctx.shared.timer.count_down();
        let debug_led = ctx.local.debug_led;
        loop {
            info!("on!");
            debug_led.set_high().unwrap();
            delay.start(1.seconds());
            nb::block!(delay.wait()).ok();

            info!("off!");
            delay.start(1.seconds());
            debug_led.set_low().unwrap();
            nb::block!(delay.wait()).ok();
        }
    }

    #[task(binds = USBCTRL_IRQ, priority = 4, shared = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let usb = c.shared.usb_dev;
        let kb = c.shared.usb_class;
        (usb, kb).lock(|usb, kb| {
            if usb.poll(&mut [kb]) {
                kb.poll();
            }
        });
    }
    // #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class])]
    // fn handle_event(mut c: handle_event::Context, event: Option<Event>) {
    //     let mut layout = c.shared.layout;
    //     match event {
    //         None => match layout.lock(|l| l.tick()) {
    //             CustomEvent::Press(event) => match event {
    //                 kb_layout::CustomActions::Underglow => {
    //                     handle_underglow::spawn().unwrap();
    //                 }
    //                 kb_layout::CustomActions::Bootloader => {
    //                     rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
    //                 }
    //                 kb_layout::CustomActions::Display => {
    //                     handle_display::spawn().unwrap();
    //                 }
    //             },
    //             _ => (),
    //         },
    //         Some(e) => {
    //             layout.lock(|l| l.event(e));
    //             return;
    //         }
    //     }

    //     let report: key_code::KbHidReport = layout.lock(|l| l.keycodes().collect());
    //     if !c
    //         .shared
    //         .usb_class
    //         .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
    //     {
    //         return;
    //     }
    //     if c.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
    //         return;
    //     }
    //     while let Ok(0) = c.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
    // }

    // --------------------------------------------------------------
    /// DEBUG
    #[task]
    fn tick(_: tick::Context) {
        info!("Tick");
        tick::spawn_after(1_0000.millis()).ok();
    }

    #[task(shared = [ws], local = [n: u8 = 0])]
    fn led_color_wheel(mut ctx: led_color_wheel::Context) {
        let n = ctx.local.n;

        ctx.shared.ws.lock(|ws| {
            ws.write(brightness(core::iter::once(crate::wheel_color(*n)), 32))
                .unwrap();
            *n = n.wrapping_add(1);
        });

        led_color_wheel::spawn_after(20.millis()).ok();
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel_color(mut wheel_pos: u8) -> smart_leds::RGB8 {
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
