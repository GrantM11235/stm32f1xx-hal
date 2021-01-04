//! # Direct Memory Access
#![allow(dead_code)]

use core::{
    marker::PhantomData,
    mem, ptr,
    sync::atomic::{self, compiler_fence, Ordering},
};
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

use crate::rcc::AHB;

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    Overrun,
    TransferError,
}

type Result<T> = core::result::Result<T, Error>;
type ResultNonBlocking<T> = nb::Result<T, Error>;

pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Half {
    First,
    Second,
}

pub struct CircBuffer<BUFFER, PAYLOAD>
where
    BUFFER: 'static,
{
    buffer: &'static mut [BUFFER; 2],
    payload: PAYLOAD,
    readable_half: Half,
}

impl<BUFFER, PAYLOAD> CircBuffer<BUFFER, PAYLOAD>
where
    &'static mut [BUFFER; 2]: StaticWriteBuffer,
    BUFFER: 'static,
{
    pub(crate) fn new(buf: &'static mut [BUFFER; 2], payload: PAYLOAD) -> Self {
        CircBuffer {
            buffer: buf,
            payload,
            readable_half: Half::Second,
        }
    }
}

pub trait DmaExt {
    type Channels;

    fn split(self, ahb: &mut AHB) -> Self::Channels;
}

pub trait TransferPayload {
    fn start(&mut self);
    fn stop(&mut self);
}

pub struct Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    _mode: PhantomData<MODE>,
    buffer: BUFFER,
    payload: PAYLOAD,
}

impl<BUFFER, PAYLOAD> Transfer<R, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn r(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn w(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<MODE, BUFFER, PAYLOAD> Drop for Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    fn drop(&mut self) {
        self.payload.stop();
        compiler_fence(Ordering::SeqCst);
    }
}

/// Read transfer
pub struct R;

/// Write transfer
pub struct W;

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

/// DMA Receiver
pub struct RxDma<PAYLOAD, RXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: RXCH,
}

/// DMA Transmitter
pub struct TxDma<PAYLOAD, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: TXCH,
}

/// DMA Receiver/Transmitter
pub struct RxTxDma<PAYLOAD, RXCH, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub rxchannel: RXCH,
    pub txchannel: TXCH,
}

pub trait Receive {
    type RxChannel;
    type TransmittedWord;
}

pub trait Transmit {
    type TxChannel;
    type ReceivedWord;
}

/// Trait for circular DMA readings from peripheral to memory.
pub trait CircReadDma<B, RS>: Receive
where
    &'static mut [B; 2]: StaticWriteBuffer<Word = RS>,
    B: 'static,
    Self: core::marker::Sized,
{
    fn circ_read(self, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self>;
}

/// Trait for DMA readings from peripheral to memory.
pub trait ReadDma<B, RS>: Receive
where
    B: StaticWriteBuffer<Word = RS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn read(self, buffer: B) -> Transfer<W, B, Self>;
}

/// Trait for DMA writing from memory to peripheral.
pub trait WriteDma<B, TS>: Transmit
where
    B: StaticReadBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn write(self, buffer: B) -> Transfer<R, B, Self>;
}

bitflags::bitflags! {
    pub struct Flags: u32 {
        const GLOBAL = 0b0001;
        const TRANSFER_COMPLETE = 0b0010;
        const HALF_TRANSFER = 0b0100;
        const TRANSFER_ERROR = 0b1000;
    }
}

pub trait ChannelLowLevel: Sized {
    /// Associated peripheral `address`
    fn set_peripheral_address(&mut self, address: u32);

    /// `address` where from/to data will be read/write
    fn set_memory_address(&mut self, address: u32);

    /// Number of bytes to transfer
    fn set_transfer_length(&mut self, len: usize);

    /// Starts the DMA transfer
    fn start(&mut self) {
        self.cr().modify(|_, w| w.en().set_bit());
    }

    /// Stops the DMA transfer
    fn stop(&mut self) {
        self.clear_flags(Flags::GLOBAL);
        self.cr().modify(|_, w| w.en().clear_bit());
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

impl<B, PAYLOAD, CX> CircBuffer<B, RxDma<PAYLOAD, CX>>
where
    CX: ChannelLowLevel,
    RxDma<PAYLOAD, CX>: TransferPayload,
{
    /// Peeks into the readable half of the buffer
    pub fn peek<R, F>(&mut self, f: F) -> Result<R>
    where
        F: FnOnce(&B, Half) -> R,
    {
        let half_being_read = self.readable_half()?;

        let buf = match half_being_read {
            Half::First => &self.buffer[0],
            Half::Second => &self.buffer[1],
        };

        // XXX does this need a compiler barrier?
        let ret = f(buf, half_being_read);

        let flags = self.payload.channel.get_flags();
        let first_half_is_done = flags.contains(Flags::HALF_TRANSFER);
        let second_half_is_done = flags.contains(Flags::TRANSFER_COMPLETE);

        if (half_being_read == Half::First && second_half_is_done)
            || (half_being_read == Half::Second && first_half_is_done)
        {
            Err(Error::Overrun)
        } else {
            Ok(ret)
        }
    }

    /// Returns the `Half` of the buffer that can be read
    pub fn readable_half(&mut self) -> Result<Half> {
        let flags = self.payload.channel.get_flags();
        let first_half_is_done = flags.contains(Flags::HALF_TRANSFER);
        let second_half_is_done = flags.contains(Flags::TRANSFER_COMPLETE);

        if first_half_is_done && second_half_is_done {
            return Err(Error::Overrun);
        }

        let last_read_half = self.readable_half;

        Ok(match last_read_half {
            Half::First => {
                if second_half_is_done {
                    self.payload.channel.clear_flags(Flags::TRANSFER_COMPLETE);

                    self.readable_half = Half::Second;
                    Half::Second
                } else {
                    last_read_half
                }
            }
            Half::Second => {
                if first_half_is_done {
                    self.payload.channel.clear_flags(Flags::HALF_TRANSFER);

                    self.readable_half = Half::First;
                    Half::First
                } else {
                    last_read_half
                }
            }
        })
    }

    /// Stops the transfer and returns the underlying buffer and RxDma
    pub fn stop(mut self) -> (&'static mut [B; 2], RxDma<PAYLOAD, CX>) {
        self.payload.stop();

        (self.buffer, self.payload)
    }
}

impl<BUFFER, PAYLOAD, MODE, CX> Transfer<MODE, BUFFER, RxDma<PAYLOAD, CX>>
where
    CX: ChannelLowLevel,
    RxDma<PAYLOAD, CX>: TransferPayload,
{
    pub fn is_done(&self) -> bool {
        !self.payload.channel.in_progress()
    }

    pub fn wait(mut self) -> (BUFFER, RxDma<PAYLOAD, CX>) {
        while !self.is_done() {}

        atomic::compiler_fence(Ordering::Acquire);

        self.payload.stop();

        // we need a read here to make the Acquire fence effective
        // we do *not* need this if `dma.stop` does a RMW operation
        unsafe {
            ptr::read_volatile(&0);
        }

        // we need a fence here for the same reason we need one in `Transfer.wait`
        atomic::compiler_fence(Ordering::Acquire);

        // `Transfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            let buffer = ptr::read(&self.buffer);
            let payload = ptr::read(&self.payload);
            mem::forget(self);
            (buffer, payload)
        }
    }
}

impl<BUFFER, PAYLOAD, MODE, CX> Transfer<MODE, BUFFER, TxDma<PAYLOAD, CX>>
where
    CX: ChannelLowLevel,
    TxDma<PAYLOAD, CX>: TransferPayload,
{
    pub fn is_done(&self) -> bool {
        !self.payload.channel.in_progress()
    }

    pub fn wait(mut self) -> (BUFFER, TxDma<PAYLOAD, CX>) {
        while !self.is_done() {}

        atomic::compiler_fence(Ordering::Acquire);

        self.payload.stop();

        // we need a read here to make the Acquire fence effective
        // we do *not* need this if `dma.stop` does a RMW operation
        unsafe {
            ptr::read_volatile(&0);
        }

        // we need a fence here for the same reason we need one in `Transfer.wait`
        atomic::compiler_fence(Ordering::Acquire);

        // `Transfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            let buffer = ptr::read(&self.buffer);
            let payload = ptr::read(&self.payload);
            mem::forget(self);
            (buffer, payload)
        }
    }
}

impl<BUFFER, PAYLOAD, CX> Transfer<W, BUFFER, RxDma<PAYLOAD, CX>>
where
    CX: ChannelLowLevel,
    RxDma<PAYLOAD, CX>: TransferPayload,
{
    pub fn peek<T>(&self) -> &[T]
    where
        BUFFER: AsRef<[T]>,
    {
        let pending = self.payload.channel.get_ndtr() as usize;

        let slice = self.buffer.as_ref();
        let capacity = slice.len();

        &slice[..(capacity - pending)]
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

pub trait Direction {
    const DIRECTION: stm32f1::stm32f103::dma1::ch::cr::DIR_A;
}

impl Direction for R {
    const DIRECTION: stm32f1::stm32f103::dma1::ch::cr::DIR_A =
        stm32f1::stm32f103::dma1::ch::cr::DIR_A::FROMPERIPHERAL;
}

impl Direction for W {
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
}

impl<CHANNEL, PERIPH> PeriphChannel<CHANNEL, PERIPH>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph<Direction = R>,
{
    pub fn send_linear<B>(mut self, mut buffer: B) -> LinearTransfer<CHANNEL, PERIPH, B>
    where
        B: StaticWriteBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, len) = unsafe { buffer.static_write_buffer() };
        self.channel.set_memory_address(ptr as u32);
        self.channel.set_transfer_length(len);
        self.channel.start();

        LinearTransfer {
            mychannel: self,
            buffer,
        }
    }
}

impl<CHANNEL, PERIPH> PeriphChannel<CHANNEL, PERIPH>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph<Direction = W>,
{
    pub fn recieve_linear<B>(mut self, buffer: B) -> LinearTransfer<CHANNEL, PERIPH, B>
    where
        B: StaticReadBuffer<Word = PERIPH::MemWord>,
    {
        let (ptr, len) = unsafe { buffer.static_read_buffer() };
        self.channel.set_memory_address(ptr as u32);
        self.channel.set_transfer_length(len);
        self.channel.start();

        LinearTransfer {
            mychannel: self,
            buffer,
        }
    }
}

pub struct LinearTransfer<CHANNEL, PERIPH, BUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    mychannel: PeriphChannel<CHANNEL, PERIPH>,
    buffer: BUFFER,
}

impl<CHANNEL, PERIPH, BUFFER> LinearTransfer<CHANNEL, PERIPH, BUFFER>
where
    CHANNEL: ChannelLowLevel,
    PERIPH: DmaPeriph,
{
    pub fn get_remaining_transfers(&self) -> Result<u16> {
        if self
            .mychannel
            .channel
            .get_flags()
            .contains(Flags::TRANSFER_ERROR)
        {
            Err(Error::TransferError)
        } else {
            Ok(self.mychannel.channel.get_ndtr() as u16)
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
        self.mychannel.channel.stop();
        (self.mychannel, self.buffer)
    }

    pub fn wait(self) -> ResultNonBlocking<(PeriphChannel<CHANNEL, PERIPH>, BUFFER)> {
        self.poll()?;
        Ok(self.abort())
    }

    pub fn restart(&mut self) {
        self.mychannel.channel.stop();
        self.mychannel.channel.start();
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
