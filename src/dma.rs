//! # Direct Memory Access

use core::{convert::TryInto, task::Poll};

use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};
use futures::{Future, future::poll_fn};

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
            pub struct $CX {}

            impl dma::private::Sealed for $CX {}

            impl dma::ChannelLowLevel for $CX {
                unsafe fn set_par(&mut self, address: u32) {
                    &(*pac::$DMAX::ptr()).$chX.par.write(|w| w.pa().bits(address) );
                }

                unsafe fn set_mar(&mut self, address: u32) {
                    &(*pac::$DMAX::ptr()).$chX.mar.write(|w| w.ma().bits(address) );
                }

                unsafe fn set_ndt(&mut self, len: u16) {
                    &(*pac::$DMAX::ptr()).$chX.ndtr.write(|w| w.ndt().bits(len));
                }

                fn get_ndt(&self) -> u16 {
                    unsafe { &(*pac::$DMAX::ptr()) }.$chX.ndtr.read().bits() as u16
                }

                unsafe fn cr(&mut self) -> &pac::dma1::ch::CR {
                    &&(*pac::$DMAX::ptr()).$chX.cr
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

                Channels{ $($chX: $CX {}),+ }
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

fn poll<CHANNEL>(channel: &CHANNEL) -> impl Future<Output = ()> + '_
where
    CHANNEL: ChannelLowLevel,
{
    poll_fn(move |cx| {
        channel.clear_flags(Flags::GLOBAL);
        if channel.get_ndt() == 0 {
            Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    })
}

pub fn take_periph<CHANNEL, PERIPH>(
    mut channel: CHANNEL,
    periph: PERIPH,
) -> PeriphChannel<CHANNEL, PERIPH>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph<Channel = CHANNEL>,
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

pub async fn mem_to_mem<CHANNEL, SOURCE, DEST>(
    channel: &mut CHANNEL,
    source: SOURCE,
    mut dest: DEST,
) -> (SOURCE, DEST)
where
    CHANNEL: ChannelLowLevel,
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

type DmaWordSize = pac::dma1::ch::cr::PSIZE_A;

pub unsafe trait DmaWord: Copy {
    const SIZE: DmaWordSize;
}

unsafe impl DmaWord for u8 {
    const SIZE: DmaWordSize = DmaWordSize::BITS8;
}

unsafe impl DmaWord for [u8; 2] {
    const SIZE: DmaWordSize = DmaWordSize::BITS16;
}

unsafe impl DmaWord for [u8; 4] {
    const SIZE: DmaWordSize = DmaWordSize::BITS32;
}

unsafe impl DmaWord for u16 {
    const SIZE: DmaWordSize = DmaWordSize::BITS16;
}

unsafe impl DmaWord for [u16; 2] {
    const SIZE: DmaWordSize = DmaWordSize::BITS32;
}

unsafe impl DmaWord for u32 {
    const SIZE: DmaWordSize = DmaWordSize::BITS32;
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
    type Channel: ChannelLowLevel;
    fn address(&self) -> u32;

    fn configure_channel(self, channel: Self::Channel) -> PeriphChannel<Self::Channel, Self>
    where
        Self: Sized,
    {
        take_periph(channel, self)
    }
}

pub struct PeriphChannel<CHANNEL, PERIPH>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    channel: CHANNEL,
    periph: PERIPH,
}

impl<CHANNEL, PERIPH> PeriphChannel<CHANNEL, PERIPH>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    pub fn split(self) -> (CHANNEL, PERIPH) {
        (self.channel, self.periph)
    }

    unsafe fn start(&mut self, address: u32, len: usize, circular: bool) {
        self.channel.set_mar(address);
        self.channel.set_ndt(len.try_into().unwrap());
        self.channel
            .cr()
            .modify(|_, w| w.en().enabled().circ().bit(circular));
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

        poll(&self.channel).await;

        self.stop();

        buffer
    }

    // pub fn into_stream<BUFFER>(self, buffer: BUFFER) -> DmaStream<CHANNEL, PERIPH, BUFFER>
    // where
    //     BUFFER: StaticWriteBuffer<Word = PERIPH::MemWord>,
    //     BUFFER: AsRef<[PERIPH::MemWord]>,
    // {
    //     todo!()
    // }

    pub fn recieve_circular<HALFBUFFER>(
        mut self,
        buffer: &'static mut [HALFBUFFER; 2],
    ) -> CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
    where
        &'static mut [HALFBUFFER; 2]: StaticReadBuffer<Word = PERIPH::MemWord>,
        HALFBUFFER: 'static,
    {
        let (ptr, half_len) = unsafe { buffer.static_read_buffer() };

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

        poll(&self.channel).await;

        self.stop();

        buffer
    }

    // pub fn send_circular<HALFBUFFER>(
    //     mut self,
    //     mut buffer: [HALFBUFFER; 2],
    // ) -> CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
    // where
    //     HALFBUFFER: StaticWriteBuffer<Word = PERIPH::MemWord>,
    // {
    //     let (ptr, half_len) = unsafe { buffer[0].static_write_buffer() };

    //     unsafe { self.start(ptr as u32, half_len * 2, true) };

    //     CircularTransfer {
    //         periph_channel: self,
    //         buffer,
    //     }
    // }
}

pub struct CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
    &'static mut [HALFBUFFER; 2]: StaticReadBuffer<Word = PERIPH::MemWord>,
    HALFBUFFER: 'static,
{
    periph_channel: PeriphChannel<CHANNEL, PERIPH>,
    buffer: &'static mut [HALFBUFFER; 2],
}

impl<CHANNEL, PERIPH, HALFBUFFER> CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
    &'static mut [HALFBUFFER; 2]: StaticReadBuffer<Word = PERIPH::MemWord>,
    HALFBUFFER: 'static,
{
    pub fn abort(mut self) -> (PeriphChannel<CHANNEL, PERIPH>, &'static mut [HALFBUFFER; 2]) {
        self.periph_channel.stop();
        (self.periph_channel, self.buffer)
    }

    fn accessable_half(&self) -> Result<Option<Half>> {
        let flags = self.periph_channel.channel.get_flags();
        if flags.bits != 0 {
            defmt::info!("{:u32}", flags.bits);
        }
        if flags.contains(Flags::TRANSFER_ERROR) {
            Err(Error::TransferError)
        } else if flags.contains(Flags::HALF_TRANSFER | Flags::TRANSFER_COMPLETE) {
            self.periph_channel.channel.clear_flags(Flags::HALF_TRANSFER & Flags::TRANSFER_COMPLETE);
            Err(Error::Overrun)
        } else if flags.contains(Flags::HALF_TRANSFER) {
            self.periph_channel.channel.clear_flags(Flags::HALF_TRANSFER);
            Ok(Some(Half::First))
        } else if flags.contains(Flags::TRANSFER_COMPLETE) {
            self.periph_channel.channel.clear_flags(Flags::TRANSFER_COMPLETE);
            Ok(Some(Half::Second))
        } else {
            Ok(None)
        }
    }

    fn poll(&self) -> impl Future<Output = Result<Half>> + '_ {
        poll_fn(move |cx| {
            // self.periph_channel.channel.clear_flags(Flags::GLOBAL);
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

    // fn mark_half_done(&mut self) -> Result<()> {
    //     // Panics if there is no accessable half
    //     let half = self.accessable_half()?.unwrap();

    //     match half {
    //         Half::First => {
    //             self.periph_channel
    //                 .channel
    //                 .clear_flags(Flags::HALF_TRANSFER);
    //         }
    //         Half::Second => {
    //             self.periph_channel
    //                 .channel
    //                 .clear_flags(Flags::TRANSFER_COMPLETE);
    //         }
    //     }
    //     Ok(())
    // }

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

        if self.accessable_half()? != None {
            Err(Error::Overrun)
        } else {
            Ok(ret)
        }
    }
}

// pub struct DmaStream<CHANNEL, PERIPH, BUFFER>
// where
//     CHANNEL: ChannelLowLevel,
//     PERIPH: DmaPeriph,
//     BUFFER: AsRef<[PERIPH::MemWord]>,
// {
//     periph_channel: PeriphChannel<CHANNEL, PERIPH>,
//     buffer: BUFFER,
//     i: u16,
// }

// impl<CHANNEL, PERIPH, BUFFER> Stream for DmaStream<CHANNEL, PERIPH, BUFFER>
// where
//     CHANNEL: ChannelLowLevel,
//     PERIPH: DmaPeriph,
//     BUFFER: AsRef<[PERIPH::MemWord]>,
// {
//     type Item = PERIPH::MemWord;

//     fn poll_next(
//         self: core::pin::Pin<&mut Self>,
//         cx: &mut core::task::Context<'_>,
//     ) -> Poll<Option<Self::Item>> {
//         if self.periph_channel.channel.get_ndt() < self.i {
//             let val = self.buffer.as_ref()[self.i as usize];
//             self.i += 1;
//             if self.i as usize == self.buffer.as_ref().len() {
//                 unsafe {
//                     self.periph_channel.channel.cr().modify(|_, w| w.en().disabled());
//                     self.periph_channel.channel.cr().modify(|_, w| w.en().enabled());
//                 }
//                 self.i = 0;
//             }
//             Poll::Ready(Some(val))
//         } else {
//             cx.waker().wake_by_ref();
//             Poll::Pending
//         }
//     }
// }