//! # Direct Memory Access
#![allow(dead_code)]

use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

use crate::rcc::AHB;

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    Overrun,
    TransferError,
}

pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Half {
    First,
    Second,
}

bitflags::bitflags! {
    pub struct Flags: u32 {
        const GLOBAL = 0b0001;
        const TRANSFER_COMPLETE = 0b0010;
        const HALF_TRANSFER = 0b0100;
        const TRANSFER_ERROR = 0b1000;
    }
}

type Result<T> = core::result::Result<T, Error>;
type ResultNonBlocking<T> = nb::Result<T, Error>;

pub trait DmaExt {
    type Channels;

    fn split(self, ahb: &mut AHB) -> Self::Channels;
}

macro_rules! dma {
    ($($DMAX:ident: ($dmaX:ident, {
        $($CX:ident: (
            $chX:ident,
            $x:expr,
        ),)+
    }),)+) => {
        $(
            pub mod $dmaX {
                use crate::pac::{$DMAX, dma1};

                use crate::dma::{self, ChannelLowLevel, DmaExt};
                use crate::rcc::{AHB, Enable};

                pub struct Channels{$(pub $chX: $CX),+}

                $(
                    /// A singleton that represents a single DMAx channel (channel X in this case)
                    ///
                    /// This singleton has exclusive access to the registers of the DMAx channel X
                    pub struct $CX { ch: &'static dma1::CH }

                    impl ChannelLowLevel for $CX {
                        /// Associated peripheral `address`
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        fn set_peripheral_address(&mut self, address: u32) {
                            self.ch.par.write(|w| w.pa().bits(address) );
                        }

                        /// `address` where from/to data will be read/write
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        fn set_memory_address(&mut self, address: u32) {
                            self.ch.mar.write(|w| w.ma().bits(address) );
                        }

                        /// Number of bytes to transfer
                        fn set_transfer_length(&mut self, len: usize) {
                            self.ch.ndtr.write(|w| w.ndt().bits(cast::u16(len).unwrap()));
                        }

                        fn cr(&mut self) -> &dma1::ch::CR {
                            &self.ch.cr
                        }

                        fn get_ndtr(&self) -> u32 {
                            // NOTE(unsafe) atomic read with no side effects
                            self.ch.ndtr.read().bits()
                        }

                        fn get_flags(&self) -> dma::Flags {
                            dma::Flags::from_bits_truncate(unsafe { &(*$DMAX::ptr()) }.isr.read().bits() >> ($x - 1))
                        }

                        fn clear_flags(&self, flags: dma::Flags) {
                            unsafe { &(*$DMAX::ptr()) }.ifcr.write(|w| unsafe { w.bits(flags.bits() << ($x - 1)) });
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Channels = Channels;

                    fn split(self, ahb: &mut AHB) -> Channels {
                        $DMAX::enable(ahb);

                        let dma1::RegisterBlock {$($chX),+, ..} = unsafe {&*Self::ptr()};

                        // reset the DMA control registers (stops all on-going transfers)
                        $(
                            $chX.cr.reset();
                        )+

                        Channels{ $($chX: $CX { ch: $chX }),+ }
                    }
                }
            }
        )+
    }
}

dma! {
    DMA1: (dma1, {
        C1: (
            ch1, 1,
        ),
        C2: (
            ch2, 2,
        ),
        C3: (
            ch3, 3,
        ),
        C4: (
            ch4, 4,
        ),
        C5: (
            ch5, 5,
        ),
        C6: (
            ch6, 6,
        ),
        C7: (
            ch7, 7,
        ),
    }),

    DMA2: (dma2, {
        C1: (
            ch1, 1,
        ),
        C2: (
            ch2, 2,
        ),
        C3: (
            ch3, 3,
        ),
        C4: (
            ch4, 4,
        ),
        C5: (
            ch5, 5,
        ),
    }),
}

pub trait ChannelLowLevel: Sized {
    /// Associated peripheral `address`
    fn set_peripheral_address(&mut self, address: u32);

    /// `address` where from/to data will be read/write
    fn set_memory_address(&mut self, address: u32);

    /// Number of bytes to transfer
    fn set_transfer_length(&mut self, len: usize);

    /// Starts the DMA transfer
    fn start(&mut self, circular: Option<bool>) {
        match circular {
            Some(circ) => {
                self.cr().modify(|_, w| w.en().set_bit().circ().bit(circ));
            }
            None => {
                self.cr().modify(|_, w| w.en().set_bit());
            }
        }
    }

    /// Stops the DMA transfer
    fn stop(&mut self) {
        self.cr().modify(|_, w| w.en().clear_bit());
        self.clear_flags(Flags::all());
    }

    /// Returns `true` if there's a transfer in progress
    fn in_progress(&self) -> bool {
        self.get_flags().contains(Flags::TRANSFER_COMPLETE)
    }

    fn listen(&mut self, event: Event) {
        match event {
            Event::HalfTransfer => self.cr().modify(|_, w| w.htie().set_bit()),
            Event::TransferComplete => self.cr().modify(|_, w| w.tcie().set_bit()),
        }
    }

    fn unlisten(&mut self, event: Event) {
        match event {
            Event::HalfTransfer => self.cr().modify(|_, w| w.htie().clear_bit()),
            Event::TransferComplete => self.cr().modify(|_, w| w.tcie().clear_bit()),
        }
    }

    fn cr(&mut self) -> &crate::pac::dma1::ch::CR;

    fn get_ndtr(&self) -> u32;

    fn get_flags(&self) -> Flags;
    fn clear_flags(&self, flags: Flags);

    fn take_periph<PERIPH>(mut self, periph: PERIPH) -> PeriphChannel<Self, PERIPH>
    where
        PERIPH: DmaPeriph<Channel = Self>,
    {
        self.cr().reset();
        self.set_peripheral_address(periph.address());
        self.cr().write(|w| {
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
        PeriphChannel {
            channel: self,
            periph,
        }
    }
}

pub trait DmaWord {
    const SIZE: crate::pac::dma1::ch::cr::PSIZE_A;
}

impl DmaWord for u8 {
    const SIZE: crate::pac::dma1::ch::cr::PSIZE_A = crate::pac::dma1::ch::cr::PSIZE_A::BITS8;
}

impl DmaWord for u16 {
    const SIZE: crate::pac::dma1::ch::cr::PSIZE_A = crate::pac::dma1::ch::cr::PSIZE_A::BITS16;
}

impl DmaWord for u32 {
    const SIZE: crate::pac::dma1::ch::cr::PSIZE_A = crate::pac::dma1::ch::cr::PSIZE_A::BITS32;
}

/// Read transfer
pub struct Rx;

/// Write transfer
pub struct Tx;

pub trait Direction {
    const DIRECTION: stm32f1::stm32f103::dma1::ch::cr::DIR_A;
}

impl Direction for Rx {
    const DIRECTION: stm32f1::stm32f103::dma1::ch::cr::DIR_A =
        stm32f1::stm32f103::dma1::ch::cr::DIR_A::FROMPERIPHERAL;
}

impl Direction for Tx {
    const DIRECTION: stm32f1::stm32f103::dma1::ch::cr::DIR_A =
        stm32f1::stm32f103::dma1::ch::cr::DIR_A::FROMMEMORY;
}

pub trait DmaPeriph {
    type Direction: Direction;
    type PeriphWord: DmaWord;
    type MemWord: DmaWord;
    type Channel: ChannelLowLevel;
    fn address(&self) -> u32;
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

    fn start(&mut self, address: u32, len: usize, circular: bool) {
        self.channel.set_memory_address(address);
        self.channel.set_transfer_length(len);
        self.channel.start(Some(circular));
    }
}

impl<CHANNEL, PERIPH> PeriphChannel<CHANNEL, PERIPH>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph<Direction = Rx>,
{
    pub fn recieve_linear<BUFFER>(
        mut self,
        buffer: BUFFER,
    ) -> LinearTransfer<CHANNEL, PERIPH, BUFFER>
    where
        BUFFER: StaticReadBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, len) = unsafe { buffer.static_read_buffer() };

        self.start(ptr as u32, len, false);

        LinearTransfer {
            periph_channel: self,
            buffer,
        }
    }

    pub fn recieve_circular<HALFBUFFER>(
        mut self,
        buffer: [HALFBUFFER; 2],
    ) -> CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
    where
        HALFBUFFER: StaticReadBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, half_len) = unsafe { buffer[0].static_read_buffer() };

        self.start(ptr as u32, half_len * 2, true);

        CircularTransfer {
            periph_channel: self,
            buffer,
            next_half: Half::First,
        }
    }
}

impl<CHANNEL, PERIPH> PeriphChannel<CHANNEL, PERIPH>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph<Direction = Tx>,
{
    pub fn send_linear<B>(mut self, mut buffer: B) -> LinearTransfer<CHANNEL, PERIPH, B>
    where
        B: StaticWriteBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, len) = unsafe { buffer.static_write_buffer() };

        self.start(ptr as u32, len, false);

        LinearTransfer {
            periph_channel: self,
            buffer,
        }
    }

    pub fn send_circular<HALFBUFFER>(
        mut self,
        mut buffer: [HALFBUFFER; 2],
    ) -> CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
    where
        HALFBUFFER: StaticWriteBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, half_len) = unsafe { buffer[0].static_write_buffer() };

        self.start(ptr as u32, half_len * 2, true);

        CircularTransfer {
            periph_channel: self,
            buffer,
            next_half: Half::First,
        }
    }
}

pub struct LinearTransfer<CHANNEL, PERIPH, BUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    periph_channel: PeriphChannel<CHANNEL, PERIPH>,
    buffer: BUFFER,
}

impl<CHANNEL, PERIPH, BUFFER> LinearTransfer<CHANNEL, PERIPH, BUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    pub fn get_remaining_transfers(&self) -> Result<u16> {
        if self
            .periph_channel
            .channel
            .get_flags()
            .contains(Flags::TRANSFER_ERROR)
        {
            Err(Error::TransferError)
        } else {
            Ok(self.periph_channel.channel.get_ndtr() as u16)
        }
    }

    pub fn poll(&self) -> ResultNonBlocking<()> {
        let remaining = self.get_remaining_transfers()?;
        if remaining == 0 {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    pub fn abort(mut self) -> (PeriphChannel<CHANNEL, PERIPH>, BUFFER) {
        self.periph_channel.channel.stop();
        (self.periph_channel, self.buffer)
    }

    pub fn wait(self) -> ResultNonBlocking<(PeriphChannel<CHANNEL, PERIPH>, BUFFER)> {
        self.poll()?;
        Ok(self.abort())
    }

    pub fn restart(&mut self) {
        self.periph_channel.channel.stop();
        self.periph_channel.channel.start(Some(false));
    }

    pub fn peek<T>(&self) -> Result<&[T]>
    where
        BUFFER: AsRef<[T]>,
    {
        let pending = self.get_remaining_transfers()? as usize;

        let slice = self.buffer.as_ref();
        let capacity = slice.len();

        Ok(&slice[..(capacity - pending)])
    }

    pub fn peek_mut<T>(&mut self) -> Result<&mut [T]>
    where
        BUFFER: AsMut<[T]>,
    {
        let pending = self.get_remaining_transfers()? as usize;

        let slice = self.buffer.as_mut();
        let capacity = slice.len();

        Ok(&mut slice[..(capacity - pending)])
    }
}

pub struct CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    periph_channel: PeriphChannel<CHANNEL, PERIPH>,
    buffer: [HALFBUFFER; 2],
    next_half: Half,
}

impl<CHANNEL, PERIPH, HALFBUFFER> CircularTransfer<CHANNEL, PERIPH, HALFBUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    fn abort(mut self) -> (PeriphChannel<CHANNEL, PERIPH>, [HALFBUFFER; 2]) {
        self.periph_channel.channel.stop();
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

    fn poll(&self) -> ResultNonBlocking<()> {
        self.periph_channel.channel.clear_flags(Flags::GLOBAL);
        if self.accessable_half()? == Some(self.next_half) {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn mark_half_done(&mut self) -> Result<()> {
        if self.accessable_half()? != Some(self.next_half) {
            Err(Error::Overrun)
        } else {
            self.next_half = match self.next_half {
                Half::First => {
                    self.periph_channel
                        .channel
                        .clear_flags(Flags::HALF_TRANSFER);
                    Half::Second
                }
                Half::Second => {
                    self.periph_channel
                        .channel
                        .clear_flags(Flags::TRANSFER_COMPLETE);
                    Half::First
                }
            };
            Ok(())
        }
    }

    pub fn peek<R, F>(&mut self, f: F) -> ResultNonBlocking<R>
    where
        F: FnOnce(&mut HALFBUFFER) -> R,
    {
        self.poll()?;

        let buf = match self.next_half {
            Half::First => &mut self.buffer[0],
            Half::Second => &mut self.buffer[1],
        };

        // XXX does this need a compiler barrier?
        let ret = f(buf);

        self.mark_half_done()?;

        Ok(ret)
    }
}
