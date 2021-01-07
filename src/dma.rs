//! # Direct Memory Access

use core::{convert::TryInto, task::Poll};

use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};
use futures::{future::poll_fn, Future};

use crate::pac;
use crate::rcc;

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    Overrun,
    TransferError,
    LengthMismatch,
    BufferTooLong,
}

bitflags::bitflags! {
    pub struct Flags: u32 {
        const GLOBAL = 0b0001;
        const TRANSFER_COMPLETE = 0b0010;
        const HALF_TRANSFER = 0b0100;
        const TRANSFER_ERROR = 0b1000;
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Half {
    First,
    Second,
}

type Result<T> = core::result::Result<T, Error>;

pub trait DmaExt {
    type Channels;

    fn split(self, ahb: &mut rcc::AHB) -> Self::Channels;
}

macro_rules! dma {
    ($DMAX:ident: {
        $($CX:ident: (
            $chX:ident,
            $x:expr
        ),)+
    }) => {
        use crate::dma;
        use crate::pac;
        use crate::rcc;
        use crate::rcc::Enable;

        pub struct Channels{$(pub $chX: $CX),+}

        $(
            /// A singleton that represents a single DMAx channel (channel X in this case)
            ///
            /// This singleton has exclusive access to the registers of the DMAx channel X
            pub struct $CX { ch: &'static pac::dma1::CH }

            impl dma::private::Sealed for $CX {}

            impl dma::ChannelLowLevel for $CX {
                unsafe fn set_par(&mut self, address: u32) {
                    self.ch.par.write(|w| w.pa().bits(address) );
                }

                unsafe fn set_mar(&mut self, address: u32) {
                    self.ch.mar.write(|w| w.ma().bits(address) );
                }

                unsafe fn set_ndt(&mut self, len: u16) {
                    self.ch.ndtr.write(|w| w.ndt().bits(len));
                }

                fn get_ndt(&self) -> u16 {
                    self.ch.ndtr.read().bits() as u16
                }

                unsafe fn cr(&mut self) -> &pac::dma1::ch::CR {
                    &self.ch.cr
                }

                fn get_flags(&self) -> dma::Flags {
                    dma::Flags::from_bits_truncate(unsafe { &(*pac::$DMAX::ptr()) }.isr.read().bits() >> ($x - 1))
                }

                fn clear_flags(&self, flags: dma::Flags) {
                    unsafe { &(*pac::$DMAX::ptr()) }.ifcr.write(|w| unsafe { w.bits(flags.bits() << ($x - 1)) });
                }
            }
        )+

        impl dma::DmaExt for pac::$DMAX {
            type Channels = Channels;

            fn split(self, ahb: &mut rcc::AHB) -> Channels {
                pac::$DMAX::enable(ahb);

                let pac::dma1::RegisterBlock {$($chX),+, ..} = unsafe {&*Self::ptr()};

                // reset the DMA control registers (stops all on-going transfers)
                $(
                    $chX.cr.reset();
                )+

                Channels{ $($chX: $CX { ch: $chX }),+ }
            }
        }
    }
}

pub mod dma1 {
    dma! {
        DMA1: {
            C1: (ch1, 1),
            C2: (ch2, 2),
            C3: (ch3, 3),
            C4: (ch4, 4),
            C5: (ch5, 5),
            C6: (ch6, 6),
            C7: (ch7, 7),
        }
    }
}
pub mod dma2 {
    dma! {
        DMA2: {
            C1: (ch1, 1),
            C2: (ch2, 2),
            C3: (ch3, 3),
            C4: (ch4, 4),
            C5: (ch5, 5),
        }
    }
}

pub trait ChannelLowLevel: Sized + private::Sealed {
    /// Associated peripheral `address`
    unsafe fn set_par(&mut self, address: u32);

    /// `address` where from/to data will be read/write
    unsafe fn set_mar(&mut self, address: u32);

    /// Number of bytes to transfer
    unsafe fn set_ndt(&mut self, len: u16);

    fn get_ndt(&self) -> u16;

    unsafe fn cr(&mut self) -> &pac::dma1::ch::CR;

    fn get_flags(&self) -> Flags;

    fn clear_flags(&self, flags: Flags);
}

fn poll<CX>(channel: &CX) -> impl Future<Output = ()> + '_
where
    CX: ChannelLowLevel,
{
    poll_fn(move |cx| {
        if channel.get_ndt() == 0 {
            Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    })
}

fn take_periph<CX, PERIPH>(mut channel: CX, periph: PERIPH) -> PeriphChannel<CX, PERIPH>
where
    CX: ChannelLowLevel,
    PERIPH: DmaPeriph<CX = CX>,
{
    unsafe {
        channel.cr().reset();
        channel.set_par(periph.address());
        channel.cr().write(|w| {
            w.mem2mem()
                .disabled()
                .pl()
                .medium()
                .msize()
                .variant(PERIPH::MemWord::SIZE)
                .psize()
                .variant(PERIPH::PeriphWord::SIZE)
                .circ()
                .disabled()
                .minc()
                .enabled()
                .pinc()
                .enabled()
                .dir()
                .variant(PERIPH::Direction::DIRECTION)
                .teie()
                .disabled()
                .htie()
                .disabled()
                .tcie()
                .disabled()
                .en()
                .disabled()
        });
    }
    PeriphChannel { channel, periph }
}

pub async fn mem_to_mem<CX, SOURCE, DEST>(
    channel: &mut CX,
    source: SOURCE,
    mut dest: DEST,
) -> (SOURCE, DEST)
where
    CX: ChannelLowLevel,
    SOURCE: StaticReadBuffer,
    <SOURCE as StaticReadBuffer>::Word: DmaWord,
    DEST: StaticWriteBuffer,
    <DEST as StaticWriteBuffer>::Word: DmaWord,
{
    let (source_ptr, source_len) = unsafe { source.static_read_buffer() };
    let (dest_ptr, dest_len) = unsafe { dest.static_write_buffer() };

    assert_eq!(source_len, dest_len);

    let len: u16 = source_len.try_into().unwrap();

    unsafe {
        channel.cr().reset();
        channel.set_par(source_ptr as u32);
        channel.set_mar(dest_ptr as u32);
        channel.set_ndt(len);
        channel.cr().write(|w| {
            w.mem2mem()
                .enabled()
                .msize()
                .variant(<DEST as StaticWriteBuffer>::Word::SIZE)
                .psize()
                .variant(<SOURCE as StaticReadBuffer>::Word::SIZE)
                .minc()
                .enabled()
                .pinc()
                .enabled()
                .dir()
                .from_peripheral()
                .en()
                .enabled()
        })
    }

    poll(channel).await;

    unsafe { channel.cr().reset() };

    (source, dest)
}

mod private {
    pub trait Sealed {}
}

pub trait DmaWord {
    const SIZE: pac::dma1::ch::cr::PSIZE_A;
}

impl DmaWord for u8 {
    const SIZE: pac::dma1::ch::cr::PSIZE_A = pac::dma1::ch::cr::PSIZE_A::BITS8;
}

impl DmaWord for u16 {
    const SIZE: pac::dma1::ch::cr::PSIZE_A = pac::dma1::ch::cr::PSIZE_A::BITS16;
}

impl DmaWord for u32 {
    const SIZE: pac::dma1::ch::cr::PSIZE_A = pac::dma1::ch::cr::PSIZE_A::BITS32;
}

/// Read transfer
pub struct Rx;

/// Write transfer
pub struct Tx;

pub trait Direction {
    const DIRECTION: pac::dma1::ch::cr::DIR_A;
}

impl Direction for Rx {
    const DIRECTION: pac::dma1::ch::cr::DIR_A = pac::dma1::ch::cr::DIR_A::FROMPERIPHERAL;
}

impl Direction for Tx {
    const DIRECTION: pac::dma1::ch::cr::DIR_A = pac::dma1::ch::cr::DIR_A::FROMMEMORY;
}

pub unsafe trait DmaPeriph {
    type Direction: Direction;
    type PeriphWord: DmaWord;
    type MemWord: DmaWord;
    type CX: ChannelLowLevel;
    fn address(&self) -> u32;

    fn configure_channel(self, channel: Self::CX) -> PeriphChannel<Self::CX, Self>
    where
        Self: Sized,
    {
        take_periph(channel, self)
    }
}

pub struct PeriphChannel<CX, PERIPH>
where
    CX: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    channel: CX,
    periph: PERIPH,
}

impl<CX, PERIPH> PeriphChannel<CX, PERIPH>
where
    CX: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    pub fn split(self) -> (CX, PERIPH) {
        (self.channel, self.periph)
    }

    unsafe fn start(&mut self, address: u32, len: usize, circular: bool) {
        self.channel.set_mar(address);
        self.channel.set_ndt(len.try_into().unwrap());
        self.channel
            .cr()
            .modify(|_, w| w.en().enabled().circ().bit(circular));
    }

    fn poll(&self) -> impl Future<Output = ()> + '_ {
        poll_fn(move |cx| {
            if self.channel.get_ndt() == 0 {
                Poll::Ready(())
            } else {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        })
    }

    fn stop(&mut self) {
        unsafe { self.channel.cr().modify(|_, w| w.en().clear_bit()) };
        self.channel.clear_flags(Flags::all());
    }
}

impl<CHANNEL, PERIPH> PeriphChannel<CHANNEL, PERIPH>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph<Direction = Rx>,
{
    pub async fn recieve_async<BUFFER>(&mut self, buffer: BUFFER) -> BUFFER
    where
        BUFFER: StaticReadBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, len) = unsafe { buffer.static_read_buffer() };

        unsafe { self.start(ptr as u32, len, false) };

        self.poll().await;

        self.stop();

        buffer
    }

    pub fn recieve_circular<HALFBUFFER>(
        mut self,
        buffer: [HALFBUFFER; 2],
    ) -> CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
    where
        HALFBUFFER: StaticReadBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, half_len) = unsafe { buffer[0].static_read_buffer() };

        unsafe { self.start(ptr as u32, half_len * 2, true) };

        CircularTransfer {
            periph_channel: self,
            buffer,
        }
    }
}

impl<CHANNEL, PERIPH> PeriphChannel<CHANNEL, PERIPH>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph<Direction = Tx>,
{
    pub async fn send_async<BUFFER>(&mut self, mut buffer: BUFFER) -> BUFFER
    where
        BUFFER: StaticWriteBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, len) = unsafe { buffer.static_write_buffer() };

        unsafe { self.start(ptr as u32, len, false) };

        self.poll().await;

        self.stop();

        buffer
    }

    pub fn send_circular<HALFBUFFER>(
        mut self,
        mut buffer: [HALFBUFFER; 2],
    ) -> CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
    where
        HALFBUFFER: StaticWriteBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, half_len) = unsafe { buffer[0].static_write_buffer() };

        unsafe { self.start(ptr as u32, half_len * 2, true) };

        CircularTransfer {
            periph_channel: self,
            buffer,
        }
    }
}

pub struct CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    periph_channel: PeriphChannel<CHANNEL, PERIPH>,
    buffer: [HALFBUFFER; 2],
}

impl<CHANNEL, PERIPH, HALFBUFFER> CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    pub fn abort(mut self) -> (PeriphChannel<CHANNEL, PERIPH>, [HALFBUFFER; 2]) {
        self.periph_channel.stop();
        (self.periph_channel, self.buffer)
    }

    fn accessable_half(&self) -> Result<Option<Half>> {
        let flags = self.periph_channel.channel.get_flags();
        if flags.contains(Flags::TRANSFER_ERROR) {
            Err(Error::TransferError)
        } else if flags.contains(Flags::HALF_TRANSFER & Flags::TRANSFER_COMPLETE) {
            Err(Error::Overrun)
        } else if flags.contains(Flags::HALF_TRANSFER) {
            Ok(Some(Half::First))
        } else if flags.contains(Flags::TRANSFER_COMPLETE) {
            Ok(Some(Half::Second))
        } else {
            Ok(None)
        }
    }

    fn poll(&self) -> impl Future<Output = Result<Half>> + '_ {
        poll_fn(move |cx| {
            self.periph_channel.channel.clear_flags(Flags::GLOBAL);
            match self.accessable_half() {
                Ok(Some(half)) => Poll::Ready(Ok(half)),
                Ok(None) => {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
                Err(e) => Poll::Ready(Err(e)),
            }
        })
    }

    fn mark_half_done(&mut self) -> Result<()> {
        // Panics if there is no accessable half
        let half = self.accessable_half()?.unwrap();

        match half {
            Half::First => {
                self.periph_channel
                    .channel
                    .clear_flags(Flags::HALF_TRANSFER);
            }
            Half::Second => {
                self.periph_channel
                    .channel
                    .clear_flags(Flags::TRANSFER_COMPLETE);
            }
        }
        Ok(())
    }

    pub async fn peek<R, F>(&mut self, f: F) -> Result<R>
    where
        F: FnOnce(&mut HALFBUFFER) -> R,
    {
        let buf = match self.poll().await? {
            Half::First => &mut self.buffer[0],
            Half::Second => &mut self.buffer[1],
        };

        // XXX does this need a compiler barrier?
        let ret = f(buf);

        self.mark_half_done()?;

        Ok(ret)
    }
}
