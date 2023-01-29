use core::hash::{Hash, Hasher};

use defmt::{debug, info, warn};
use minicbor::{encode::write::Cursor, Decode, Encode};
use rp2040_tickv;
use siphasher::sip::SipHasher;
use tickv::{success_codes::SuccessCode, ErrorCode, TicKV, MAIN_KEY};

// marker trait for typestate
pub trait StorageState {}

pub struct Ready;

pub struct Uninitialized;

impl StorageState for Ready {}
impl StorageState for Uninitialized {}

/// KV store of configs usig tickv
/// uses SipHash for keys
pub struct ConfigStorage<'a> {
    pub kv: St<'a>,
}

pub enum St<'a> {
    Uninit(&'a mut [u8; rp2040_tickv::SECTOR_SIZE]),
    Init(TicKV<'a, rp2040_tickv::RP2040FlashCtrl, { rp2040_tickv::SECTOR_SIZE }>),
}

impl<'a> St<'a> {
    fn unwrap_init(
        self,
    ) -> TicKV<'a, rp2040_tickv::RP2040FlashCtrl, { rp2040_tickv::SECTOR_SIZE }> {
        match self {
            St::Init(i) => i,
            _ => unreachable!(),
        }
    }

    fn int_mut(
        &mut self,
    ) -> &mut TicKV<'a, rp2040_tickv::RP2040FlashCtrl, { rp2040_tickv::SECTOR_SIZE }> {
        match self {
            St::Init(i) => i,
            _ => unreachable!(),
        }
    }

    fn int(&self) -> &TicKV<'a, rp2040_tickv::RP2040FlashCtrl, { rp2040_tickv::SECTOR_SIZE }> {
        match self {
            St::Init(i) => i,
            _ => unreachable!(),
        }
    }
}

impl<'a> ConfigStorage<'a> {
    /// 1MiB is the only size installed at the moment
    const _TOTAL_FLASH_SIZE: u32 = 1 * 1024 * 1024;
    /// amount of storage sectors reserved for config
    const SECTORS_RESERVED: u32 = 4;

    // TicKv storage values
    const _STORAGE_SIZE: u32 = Self::SECTORS_RESERVED * rp2040_tickv::SECTOR_SIZE as u32;

    // this is all sorts of wrong and weird and dumb
    pub fn blank(buf: &'a mut [u8; rp2040_tickv::SECTOR_SIZE]) -> Self {
        Self {
            kv: St::Uninit(buf),
        }
    }

    pub fn setup(self) -> Self {
        match self.kv {
            St::Init(_) => self,
            St::Uninit(buf) => Self::new(buf),
        }
    }

    pub fn new(buf: &'a mut [u8; rp2040_tickv::SECTOR_SIZE]) -> Self {
        let ctl = rp2040_tickv::RP2040FlashCtrl::new(Self::_TOTAL_FLASH_SIZE, Self::_STORAGE_SIZE)
            .map_err(|_| defmt::error!("could not build flash controller"))
            .unwrap();

        let kv = St::Init(TicKV::new(ctl, buf, Self::_STORAGE_SIZE as usize));

        Self { kv }
    }

    pub fn init(&mut self) -> Result<(), ErrorCode> {
        debug!("Start init of config storage.");
        let mut h = SipHasher::new();
        MAIN_KEY.hash(&mut h);

        let kv = self.kv.int_mut();
        match kv.initialise(h.finish())? {
            SuccessCode::Complete => info!("Init flash storage with no changes"),
            SuccessCode::Written => warn!("Init and setup flash storage"),
            SuccessCode::Queued => warn!("Unexpected Queuing of Flash Init"),
        };
        Ok(())
    }
    pub fn add(&mut self, k: impl AsRef<[u8]>, v: &[u8]) -> Result<SuccessCode, ErrorCode> {
        let mut h = SipHasher::new();
        h.write(k.as_ref());
        self.kv.int_mut().append_key(h.finish(), v)
    }

    pub fn remove(&mut self, k: impl AsRef<[u8]>) -> Result<SuccessCode, ErrorCode> {
        let mut h = SipHasher::new();
        h.write(k.as_ref());
        self.kv.int_mut().invalidate_key(h.finish())
    }

    pub fn get<const N: usize>(&self, k: impl AsRef<[u8]>) -> Result<[u8; N], ErrorCode> {
        let mut buf = [0u8; N];
        let mut h = SipHasher::new();
        h.write(k.as_ref());

        self.kv.int().get_key(h.finish(), &mut buf)?;
        Ok(buf)
    }

    pub fn add_enc<const S: usize>(
        &mut self,
        k: impl AsRef<[u8]>,
        v: impl Encode<()>,
    ) -> Result<SuccessCode, ErrorCode> {
        let mut buf = Cursor::new([0; S]);
        minicbor::encode(v, &mut buf).map_err(|_| ErrorCode::BufferTooSmall(S))?;
        let v = &buf.get_ref()[..buf.position()];
        self.add(k, v)
    }

    pub fn get_dec<'t, T: Decode<'t, ()>>(
        &mut self,
        k: impl AsRef<[u8]>,
        buf: &'t mut [u8],
    ) -> Result<T, ReadError> {
        let mut h = SipHasher::new();
        h.write(k.as_ref());

        self.kv.int().get_key(h.finish(), buf)?;

        Ok(minicbor::decode::<T>(buf)?)
    }
}

#[derive(Debug)]
pub enum ReadError {
    Deserialization(minicbor::decode::Error),
    FlashRead(ErrorCode),
}

impl From<minicbor::decode::Error> for ReadError {
    fn from(src: minicbor::decode::Error) -> Self {
        ReadError::Deserialization(src)
    }
}
impl From<ErrorCode> for ReadError {
    fn from(src: ErrorCode) -> Self {
        ReadError::FlashRead(src)
    }
}
