use core::marker::PhantomData;

use crate::app::monotonics;

use defmt::*;
use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};
use rp2040_hal::{
    gpio::{
        Function, FunctionConfig, OutputConfig, Pin, PinId, PinMode, ReadableOutput, ValidPinMode,
    },
    pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO},
};
use rp2040_monotonic::fugit::ExtU64;
use smart_leds::*;
use switch_hal::{OutputSwitch, ToggleableOutputSwitch};
use ws2812_pio::Ws2812Direct;

pub struct CountDownMonotonic {
    period: rp2040_monotonic::fugit::TimerDurationU64<1_000_000>,
    next_end: Option<rp2040_monotonic::fugit::TimerInstantU64<1_000_000>>,
}

impl CountDownMonotonic {
    pub fn new() -> Self {
        Self {
            period: 0u64.millis(),
            next_end: None,
        }
    }
}

impl embedded_hal::timer::CountDown for CountDownMonotonic {
    /// The unit of time used by this timer
    type Time = rp2040_monotonic::fugit::TimerDurationU64<1_000_000>;

    /// Starts a new count down
    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        self.period = count.into();
        self.next_end = monotonics::now().checked_add_duration(self.period.into())
    }

    /// Non-blockingly "waits" until the count down finishes
    ///
    /// # Contract
    ///
    /// - If `Self: Periodic`, the timer will start a new count down right after the last one
    /// finishes.
    /// - Otherwise the behavior of calling `wait` after the last call returned `Ok` is UNSPECIFIED.
    /// Implementers are suggested to panic on this scenario to signal a programmer error.
    fn wait(&mut self) -> nb::Result<(), void::Void> {
        if let Some(end) = self.next_end {
            let ts = monotonics::now();
            if ts >= end {
                self.next_end = end.checked_add_duration(self.period);
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        } else {
            defmt::panic!("Tried to wait on CountDown that wasnt started!");
        }
    }
}

pub struct Ws2812<P, SM, C, I>
where
    I: PinId,
    C: embedded_hal::timer::CountDown,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    driver: Ws2812Direct<P, SM, I>,
    cd: C,
}

impl<P, SM, C, I> Ws2812<P, SM, C, I>
where
    I: PinId,
    C: embedded_hal::timer::CountDown,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    /// Creates a new instance of this driver.
    pub fn new(
        pin: Pin<I, Function<P>>,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        clock_freq: embedded_time::rate::Hertz,
        cd: C,
    ) -> Ws2812<P, SM, C, I> {
        let driver = Ws2812Direct::new(pin, pio, sm, clock_freq);

        Self { driver, cd }
    }
}

impl<P, SM, C, I> SmartLedsWrite for Ws2812<P, SM, C, I>
where
    I: PinId,
    C: embedded_hal::timer::CountDown,
    C::Time: From<rp2040_monotonic::fugit::TimerDurationU64<1_000_000>>,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    type Color = smart_leds_trait::RGB8;
    type Error = ();
    fn write<T, J>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = J>,
        J: Into<Self::Color>,
    {
        self.cd.start(60.micros());
        let _ = nb::block!(self.cd.wait());

        self.driver.write(iterator)
    }
}

impl<const L: usize, P, SM, C, I> defmt::Format for KeypadLEDs<P, SM, C, I, L>
where
    I: PinId,
    C: embedded_hal::timer::CountDown,
    C::Time: From<rp2040_monotonic::fugit::TimerDurationU64<1_000_000>>,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    fn format(&self, f: defmt::Formatter) {
        if let Some(col) = self.colors {
            defmt::write!(
                f,
                "LED_STRIP {{ PIN: {=?} LEDS: {=?} ON: {}, BRIGHTNESS: {} }} ",
                I::DYN.num, // TODO: doesnt include group
                col.map(|c| (c.r, c.g, c.b)),
                self.on,
                self.brightness,
            )
        }
    }
}

pub struct KeypadLEDs<P, SM, C, I, const LENGTH: usize>
where
    I: PinId,
    C: embedded_hal::timer::CountDown,
    C::Time: From<rp2040_monotonic::fugit::TimerDurationU64<1_000_000>>,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    pub colors: Option<[RGB8; LENGTH]>,
    pub driver: Ws2812<P, SM, C, I>,
    pub brightness: u8,
    pub step_size: u8,
    on: bool,
}

impl<P, SM, C, I, const L: usize> KeypadLEDs<P, SM, C, I, L>
where
    I: PinId,
    C: embedded_hal::timer::CountDown,
    C::Time: From<rp2040_monotonic::fugit::TimerDurationU64<1_000_000>>,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    // constructs LED string, not clearing any previous state (even when inserting a color)
    pub fn new_explicit(
        colors: [RGB8; L],
        driver: Ws2812<P, SM, C, I>,
        brightness: u8,
        step_size: u8,
        on: bool,
    ) -> Self {
        Self {
            colors: Some(colors),
            driver,
            brightness,
            step_size,
            on,
        }
    }
    pub fn new(driver: Ws2812<P, SM, C, I>, brightness: u8) -> Self {
        Self {
            colors: None,
            driver,
            brightness,
            step_size: 10,
            on: true,
        }
    }

    /// write to led driver, respects on/off
    pub fn write<T>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = RGB8>,
    {
        if self.on {
            self.write_raw(brightness(iterator, self.brightness))
        } else {
            Err(())
        }
    }

    pub fn step_brightness(&mut self, step_up: bool) {
        if step_up {
            self.brightness = self.brightness.saturating_add(self.step_size)
        } else {
            self.brightness = self.brightness.saturating_sub(self.step_size)
        }
    }

    /// write directly to led driver, not applying brightness or respecting on/off state
    pub fn write_raw<T>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = RGB8>,
    {
        self.driver.write(iterator)
    }

    /// write color to nth LED buffer
    pub fn write_nth(&mut self, n: usize, mut f: impl FnMut(&mut RGB8)) {
        if let Some(c) = self.colors().iter_mut().rev().nth(n) {
            f(c);
        } else {
            error!(
                "tried to write to nth LED {}, on a strip {} LEDs long",
                n, L
            );
        }
    }

    /// get nth buffered color
    pub fn nth(&mut self, n: usize) -> Option<RGB8> {
        self.colors().iter().nth(n).copied()
    }

    /// write a color to each LED in strip
    pub fn write_all(&mut self, f: impl FnMut(&mut RGB8)) {
        self.colors().iter_mut().for_each(f)
    }

    /// flush buffered colors to the strip, logging errors to defmt
    pub fn flush(&mut self) {
        let colors = *self.colors();
        if self.on {
            trace!("{}", self);
            self.write(colors.iter().copied())
                .unwrap_or_else(|_| error!("failed to write colors to LED strip"))
        }
    }

    /// gets currently set colors
    /// if no colors were previously modified, colors will be reset
    pub fn colors(&mut self) -> &mut [RGB8; L] {
        self.colors.get_or_insert_with(|| {
            let colors = [RGB8::new(0, 0, 0); L];
            self.driver
                .write(colors.iter().copied())
                .unwrap_or_else(|_| {
                    error!("failed writing colors to LED strip");
                });
            colors
        })
    }
}
impl<P, SM, C, I, const L: usize> OutputSwitch for KeypadLEDs<P, SM, C, I, L>
where
    I: PinId,
    C: embedded_hal::timer::CountDown,
    C::Time: From<rp2040_monotonic::fugit::TimerDurationU64<1_000_000>>,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    type Error = ();

    fn on(&mut self) -> Result<(), Self::Error> {
        self.on = true;
        let colors = *self.colors();
        self.write_raw(brightness(colors.iter().copied(), self.brightness))
    }

    fn off(&mut self) -> Result<(), Self::Error> {
        self.on = false;
        self.write_raw([RGB8::new(0, 0, 0); L].iter().copied())
    }
}

/// what type of LED, either
pub enum LEDOnType {
    High,
    Low,
}

pub struct LED<I: PinId, LEDOnType> {
    led: Pin<I, ReadableOutput>,
    on_type: LEDOnType,
}

impl<I: PinId> LED<I, LEDOnType> {
    pub fn new(pin: Pin<I, ReadableOutput>, on_type: LEDOnType) -> Self {
        let mut out = Self { led: pin, on_type };
        out.off().ok();
        out
    }
}

impl<I: PinId> ToggleableOutputSwitch for LED<I, LEDOnType> {
    type Error = ();

    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.led.toggle().unwrap();
        Ok(())
    }
}

impl<I: PinId> OutputSwitch for LED<I, LEDOnType> {
    type Error = ();

    fn on(&mut self) -> Result<(), Self::Error> {
        match self.on_type {
            LEDOnType::High => self.led.set_high().unwrap(),
            LEDOnType::Low => self.led.set_low().unwrap(),
        }
        Ok(())
    }

    fn off(&mut self) -> Result<(), Self::Error> {
        match self.on_type {
            LEDOnType::High => self.led.set_low().unwrap(),
            LEDOnType::Low => self.led.set_high().unwrap(),
        }
        Ok(())
    }
}
