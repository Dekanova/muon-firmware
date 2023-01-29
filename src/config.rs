use keyberon::{
    action::Action,
    key_code::KeyCode,
    layout::{Layers, Layout},
};
use minicbor::{Decode, Encode};
use usb_device::{
    class_prelude::{InterfaceNumber, UsbBus},
    endpoint::EndpointOut,
};

#[derive(Debug, Default, Encode, Decode)]
#[cbor(map)]
pub struct MuonConfig {
    #[n(0)]
    buttons: Btns,
    #[n(1)]
    underglow: [u32; 2],
    #[n(2)]
    keyglow: [u32; 3],
}

#[derive(Debug, Encode, Decode, Default)]
#[cbor(array)]
pub struct Btns(#[n(0)] [u8; 3], #[n(1)] [i32; 3]);

const SET_TO: &[u8] = b"hello world!";

pub struct HidConfigClass<'a, B: UsbBus> {
    reader: (),
    interface: InterfaceNumber,
    enpoint_interrupt_out: EndpointOut<'a, B>,
    expect_interrupt_out_complete: bool,
}
