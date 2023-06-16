//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use defmt_rtt as _;
// use panic_probe as _;
use panic_halt as _;

use rp2040_hal as hal;

mod kb;
mod led;

// TODO https://crates.io/crates/switch-hal
#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1])] // extra PIO0_IRQ_1, PIO1_IRQ_0
mod app {
    use crate::hal;
    use cortex_m::prelude::*;
    // use hal::prelude::*;
    use defmt::*;
    use hal::gpio::DynPin;
    use hal::rom_data::reset_to_usb_boot;
    use hal::Timer;
    use hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::*,
        sio::Sio,
        timer::{monotonic::Monotonic, Alarm0},
        usb::UsbBus,
        watchdog::Watchdog,
    };

    use fugit::{ExtU32, ExtU64};

    use keyberon::layout::CustomEvent;
    use usb_device::class_prelude::*;

    use crate::led::Ws2812;

    use keyberon::{
        action::Action,
        debounce::Debouncer,
        key_code::*,
        layout::{Event, *},
        matrix::DirectPinMatrix,
    };
    use switch_hal::ToggleableOutputSwitch;

    use crate::kb::Leds;
    use crate::led::*;

    const TIMER_INTERVAL: u64 = 1_000;
    const SW_COUNT: usize = 3;

    #[derive(Debug, Copy, Clone)]
    pub enum FnAction {
        LED_Up,
        LED_Down,
        LED_Under_Toggle,
        LED_Key_Toggle,
    }
    const LAYERS: Layers<SW_COUNT, 2, 2, FnAction> = layout! {{
            [Z X C]
            [ Grave Escape (1) ]
        }
        {
            [n n n]
            [ {Action::Custom(FnAction::LED_Key_Toggle)} {Action::Custom(FnAction::LED_Under_Toggle)} (0) ]
        }
    };

    // use systick_monotonic::*;

    // #[monotonic(binds = SysTick, default = true)]
    // type MyMono = Systick<100>;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_class: keyberon::Class<'static, UsbBus, crate::kb::Leds>,
        #[lock_free]
        matrix: DirectPinMatrix<DynPin, SW_COUNT, 2>,
        layout: Layout<SW_COUNT, 2, 2, FnAction>,
        #[lock_free]
        debouncer: Debouncer<[[bool; SW_COUNT]; 2]>,
        #[lock_free]
        watchdog: Watchdog,
        #[lock_free]
        underglow: KeypadLEDs<hal::pac::PIO0, hal::pio::SM0, CountDownMonotonic, bank0::Gpio22, 2>,
        #[lock_free]
        keyglow:
            KeypadLEDs<hal::pac::PIO0, hal::pio::SM1, CountDownMonotonic, bank0::Gpio29, SW_COUNT>,
    }

    #[local]
    struct Local {
        status_led: crate::led::LED<bank0::Gpio11, crate::led::LEDOnType>,
    }

    #[init(local = [USB: Option<UsbBusAllocator<UsbBus>> = None,])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        debug!("init start");
        let mut resets = ctx.device.RESETS;
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        watchdog.pause_on_debug(false);

        // System Init
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
        .unwrap(); // panic if this fails, please.

        let sio = Sio::new(ctx.device.SIO);
        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Status LED
        let mut status_led = LED::new(pins.gpio11.into_readable_output(), LEDOnType::High);
        use switch_hal::OutputSwitch;
        // leave on to signal halt during init
        status_led.on().ok();

        // ------ KEYBOARD MATRIX -------
        // TODO add `DynPin` to keyberon docs
        let mut matrix = DirectPinMatrix::new([
            [
                Some(pins.gpio26.into_pull_up_input().into()),
                Some(pins.gpio27.into_pull_up_input().into()),
                Some(pins.gpio28.into_pull_up_input().into()),
            ],
            [
                Some(pins.gpio25.into_pull_up_input().into()),
                Some(pins.gpio24.into_pull_up_input().into()),
                Some(pins.gpio23.into_pull_up_input().into()),
            ],
        ])
        .unwrap(); // should't panic unless something is horribly wrong

        // ------ REBOOT SELECT -------
        // reboot into bootselect if left key is held down under reset
        if matrix.get().unwrap()[0][0] {
            reset_to_usb_boot(0, 0);
        }

        // ------ KEYBOARD LAYOUT -------
        let layout = Layout::new(&LAYERS);
        let debouncer = Debouncer::new([[false; SW_COUNT]; 2], [[false; SW_COUNT]; 2], 2);

        // ------ LEDs -------

        // ws2812 data pins
        let underglow_pin = pins.gpio22;
        let keyglow_pin = pins.gpio29;

        let (mut pio, sm0, sm1, _, _) = hal::pio::PIOExt::split(ctx.device.PIO0, &mut resets);

        let mut underglow = KeypadLEDs::new(
            Ws2812::new(
                underglow_pin.into_mode(),
                &mut pio,
                sm0,
                clocks.peripheral_clock.freq(),
                CountDownMonotonic::new(),
            ),
            30,
        );
        let mut keyglow = KeypadLEDs::new(
            Ws2812::new(
                keyglow_pin.into_mode(),
                &mut pio,
                sm1,
                clocks.peripheral_clock.freq(),
                CountDownMonotonic::new(),
            ),
            10,
        );

        // ---- USB ----

        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            ctx.device.USBCTRL_REGS,
            ctx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        let usb_bus = ctx.local.USB.insert(usb_bus);

        let usb_class = keyberon::hid::HidClass::new_with_polling_interval(
            keyberon::keyboard::Keyboard::new(Leds),
            usb_bus,
            1,
        );
        let usb_dev = usb_device::device::UsbDeviceBuilder::new(
            usb_bus,
            // https://github.com/obdev/v-usb/blob/7a28fdc685952412dad2b8842429127bc1cf9fa7/usbdrv/USB-IDs-for-free.txt#L128
            // TODO https://github.com/pidcodes/pidcodes.github.com/
            usb_device::device::UsbVidPid(0x1209, 0x0001),
        )
        .manufacturer("Dekanova")
        .product("Muon")
        .serial_number(env!("CARGO_PKG_VERSION"))
        .max_packet_size_0(64)
        .build();

        scan_timer_irq::spawn().ok();
        // clear any previous state of the LEDs
        // spawning this directly after kills things, hence delay (just microcontroller IO things)
        flush_led::spawn_after(10u64.millis()).ok();
        status_blinky::spawn().ok();
        // led_color_wheel::spawn().ok();

        // let mono = Systick::new(ctx.core.SYST, clocks.system_clock.freq().0);
        let mut t = Timer::new(ctx.device.TIMER, &mut resets);
        let a0 = t.alarm_0().unwrap();
        let mono = Monotonic::new(t, a0);

        debug!("starting watchdog");
        watchdog.start(1_500u32.micros());
        watchdog.feed();

        // should be 125 MHz
        info!(
            "init finished running at {} hz",
            &clocks.system_clock.freq().raw()
        );
        (
            Shared {
                usb_dev,
                usb_class,
                matrix,
                underglow,
                keyglow,
                layout,
                debouncer, // nb * update Hz?
                watchdog,
            },
            Local { status_led },
            init::Monotonics(mono),
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let usb = c.shared.usb_dev;
        let kb = c.shared.usb_class;
        (usb, kb).lock(|usb, kb| {
            if usb.poll(&mut [kb]) {
                kb.poll();
            }
        });
    }

    /// funnily enough, this has a priority higher than the USB IRQ
    /// might need to change to avoid latentcy spikes (LED io takes a hot minute), but I'm lazy and dont want to lock things
    #[task(priority = 2, shared = [underglow, keyglow])]
    fn flush_led(c: flush_led::Context) {
        c.shared.underglow.flush();
        c.shared.keyglow.flush();
    }

    #[task(priority = 2, shared = [layout, underglow, keyglow])]
    fn handle_function_event(c: handle_function_event::Context, action: FnAction) {
        let handle_function_event::SharedResources {
            layout,
            underglow,
            keyglow,
        } = c.shared;

        match action {
            FnAction::LED_Down => keyglow.step_brightness(false),
            FnAction::LED_Up => keyglow.step_brightness(true),
            FnAction::LED_Key_Toggle => keyglow.toggle().unwrap(),
            FnAction::LED_Under_Toggle => underglow.toggle().unwrap(),
        }
    }

    #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class, layout, underglow, keyglow], local = [counter: usize = 0])]
    fn handle_event(c: handle_event::Context, event: Option<Event>) {
        let handle_event::Context { shared, local } = c;
        let handle_event::SharedResources {
            mut layout,
            mut usb_dev,
            mut usb_class,
            underglow,
            keyglow,
        } = shared;

        if let Some(e) = event {
            *local.counter += 1;

            layout.lock(|l| l.event(e));

            // underglow combine red and blue together
            // keyglow is per key color

            debug!(" = Event: {}", local.counter);

            // led things
            match &e {
                Event::Press(_, k) => {
                    debug!("pressed key {}", k);
                    match k {
                        0 => {
                            underglow.write_all(|c| c.r = 255);
                            keyglow.write_nth(0, |c| c.r = 255);
                        }
                        1 => {
                            underglow.write_all(|c| c.g = 255);
                            keyglow.write_nth(1, |c| c.g = 255);
                        }
                        2 => {
                            underglow.write_all(|c| c.b = 255);
                            keyglow.write_nth(2, |c| c.b = 255);
                        }
                        _ => (),
                    }
                }
                Event::Release(_, k) => {
                    debug!("released key {}", k);
                    match k {
                        0 => {
                            underglow.write_all(|c| c.r = 0);
                            keyglow.write_nth(0, |c| c.r = 0);
                        }
                        1 => {
                            underglow.write_all(|c| c.g = 0);
                            keyglow.write_nth(1, |c| c.g = 0);
                        }
                        2 => {
                            underglow.write_all(|c| c.b = 0);
                            keyglow.write_nth(2, |c| c.b = 0);
                        }
                        _ => (),
                    }
                }
                _ => (),
            }
            // write to LEDs after USB things are finished
            // same priority as this function
            flush_led::spawn().unwrap_or_else(|_| error!("unable to spawn flush LED task"));
        }

        let report: KbHidReport = layout.lock(|l| l.keycodes().collect());
        if !usb_class.lock(|k| k.device_mut().set_keyboard_report(report.clone())) {
            return;
        }
        if usb_dev.lock(|d| d.state()) != usb_device::device::UsbDeviceState::Configured {
            return;
        }
        while let Ok(0) = usb_class.lock(|k| k.write(report.as_bytes())) {}
    }

    #[task(priority = 1, shared = [matrix, layout, debouncer, watchdog])]
    fn scan_timer_irq(ctx: scan_timer_irq::Context) {
        let scan_timer_irq::SharedResources {
            matrix,
            mut layout,
            debouncer,
            watchdog,
        } = ctx.shared;

        scan_timer_irq::spawn_after(TIMER_INTERVAL.micros()).ok();

        watchdog.feed();

        match layout.lock(|l| l.tick()) {
            CustomEvent::Press(&p) => handle_function_event::spawn(p).unwrap(),
            CustomEvent::Release(_) => (),
            _ => (),
        }

        for event in debouncer.events(matrix.get().unwrap()) {
            handle_event::spawn(Some(event)).unwrap();
        }
        // nothing new, but still feed update
        handle_event::spawn(None).unwrap();
    }

    // --------------------------------------------------------------
    // DEBUG
    // #[task(shared = [ws], local = [n: u8 = 0])]
    // fn led_color_wheel(mut ctx: led_color_wheel::Context) {
    //     let n = ctx.local.n;
    //     ctx.shared.ws.lock(|ws| {
    //         ws.write(brightness(core::iter::once(crate::wheel_color(*n)), 5))
    //             .unwrap();
    //         *n = n.wrapping_add(1);
    //     });
    //     led_color_wheel::spawn_after(20.millis()).ok();
    // }

    #[task(priority = 1, local = [status_led])]
    fn status_blinky(ctx: status_blinky::Context) {
        use switch_hal::ToggleableOutputSwitch;
        let led = ctx.local.status_led;
        led.toggle().ok();

        status_blinky::spawn_after(1000u64.millis()).ok();
    }
}

// /// Convert a number from `0..=255` to an RGB color triplet.
// ///
// /// The colours are a transition from red, to green, to blue and back to red.
// fn wheel_color(mut wheel_pos: u8) -> smart_leds::RGB8 {
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
