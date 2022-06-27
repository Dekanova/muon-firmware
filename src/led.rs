use crate::app::monotonics;

use rp2040_hal::{
    gpio::{Function, FunctionConfig, Pin, PinId, ValidPinMode},
    pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO},
};
use rp2040_monotonic::fugit::ExtU64;
use smart_leds::{SmartLedsWrite, RGB8};
use switch_hal::OutputSwitch;
use ws2812_pio::Ws2812Direct;

pub struct CountDownMonotonic {
    period: rp2040_monotonic::fugit::TimerDurationU64<1_000_000>,
    next_end: Option<rp2040_monotonic::fugit::TimerInstantU64<1_000_000>>,
}

impl CountDownMonotonic {
    pub fn new(duration: rp2040_monotonic::fugit::TimerDurationU64<1_000_000>) -> Self {
        Self {
            period: duration,
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
            panic!("CountDown is not running!");
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

pub struct KeypadLEDs<P, SM, C, I, const LENGTH: usize>
where
    I: PinId,
    C: embedded_hal::timer::CountDown,
    C::Time: From<rp2040_monotonic::fugit::TimerDurationU64<1_000_000>>,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    colors: [RGB8; LENGTH],
    driver: Ws2812<P, SM, C, I>,
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
    pub fn new(colors: [RGB8; L], driver: Ws2812<P, SM, C, I>, on: bool) -> Self {
        Self { colors, driver, on }
    }
    pub fn new_default(driver: Ws2812<P, SM, C, I>) -> Self {
        Self::new([RGB8::new(0, 0, 0); L], driver, true)
    }

    pub fn write<T, J>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = J>,
        J: Into<<Ws2812<P, SM, C, I> as SmartLedsWrite>::Color>,
    {
        if self.on {
            self.driver.write(iterator)
        } else {
            Err(())
        }
    }

    pub fn write_nth(&mut self, n: usize, mut f: impl FnMut(&mut RGB8)) -> Result<(), ()> {
        if let Some(c) = self.colors.iter_mut().nth(n) {
            f(c);
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn inner_mut(&mut self) -> (&mut [RGB8; L], &mut Ws2812<P, SM, C, I>) {
        let Self { driver, colors, .. } = self;
        (colors, driver)
    }

    pub fn nth(&mut self, n: usize) -> Option<RGB8> {
        self.colors.iter().nth(n).copied()
    }

    pub fn write_all(&mut self, f: impl FnMut(&mut RGB8)) {
        self.colors.iter_mut().for_each(f)
    }

    pub fn flush(&mut self, brightness: u8) -> Result<(), ()> {
        self.driver.write(smart_leds::brightness(
            self.colors.iter().copied(),
            brightness,
        ))
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
        self.driver.write(self.colors.iter().copied())
    }

    fn off(&mut self) -> Result<(), Self::Error> {
        self.on = false;
        self.driver.write([RGB8::new(0, 0, 0); L].iter().copied())
    }
}
