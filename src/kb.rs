use keyberon::{
    hid::{HidDevice, ReportType},
    key_code::KbHidReport,
};

pub struct Leds;
impl keyberon::keyboard::Leds for Leds {
    fn caps_lock(&mut self, _status: bool) {}
}
