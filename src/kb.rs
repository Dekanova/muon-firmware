pub struct Leds;
impl keyberon::keyboard::Leds for Leds {
    fn caps_lock(&mut self, status: bool) {}
}

use embedded_hal::digital::v2::{InputPin, OutputPin};
use keyberon::matrix::Matrix;

/// Dummy output pin that does nothing since we dont have a matrix
pub struct DummyPin;

impl OutputPin for DummyPin {
    type Error = ();

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
