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

                use crate::dma::{self, ChannelLowLevel, DmaExt, Event};
                use crate::rcc::{AHB, Enable};

                #[allow(clippy::manual_non_exhaustive)]
                pub struct Channels((), $(pub $CX),+);

                $(
                    /// A singleton that represents a single DMAx channel (channel X in this case)
                    ///
                    /// This singleton has exclusive access to the registers of the DMAx channel X
                    pub struct $CX { _0: () }

                    impl ChannelLowLevel for $CX {
                        /// Associated peripheral `address`
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        fn set_peripheral_address(&mut self, address: u32, inc: bool) {
                            self.ch().par.write(|w| w.pa().bits(address) );
                            self.ch().cr.modify(|_, w| w.pinc().bit(inc) );
                        }

                        /// `address` where from/to data will be read/write
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        fn set_memory_address(&mut self, address: u32, inc: bool) {
                            self.ch().mar.write(|w| w.ma().bits(address) );
                            self.ch().cr.modify(|_, w| w.minc().bit(inc) );
                        }

                        /// Number of bytes to transfer
                        fn set_transfer_length(&mut self, len: usize) {
                            self.ch().ndtr.write(|w| w.ndt().bits(cast::u16(len).unwrap()));
                        }

                        /// Starts the DMA transfer
                        fn start(&mut self) {
                            self.ch().cr.modify(|_, w| w.en().set_bit() );
                        }

                        /// Stops the DMA transfer
                        fn stop(&mut self) {
                            self.clear_flags(dma::Flags::GLOBAL);
                            self.ch().cr.modify(|_, w| w.en().clear_bit() );
                        }

                        /// Returns `true` if there's a transfer in progress
                        fn in_progress(&self) -> bool {
                            self.get_flags().contains(dma::Flags::TRANSFER_COMPLETE)
                        }

                        fn listen(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => self.ch().cr.modify(|_, w| w.htie().set_bit()),
                                Event::TransferComplete => {
                                    self.ch().cr.modify(|_, w| w.tcie().set_bit())
                                }
                            }
                        }

                        fn unlisten(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => {
                                    self.ch().cr.modify(|_, w| w.htie().clear_bit())
                                },
                                Event::TransferComplete => {
                                    self.ch().cr.modify(|_, w| w.tcie().clear_bit())
                                }
                            }
                        }

                        fn ch(&mut self) -> &dma1::CH {
                            unsafe { &(*$DMAX::ptr()).$chX }
                        }

                        fn isr(&self) -> dma1::isr::R {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).isr.read() }
                        }

                        fn ifcr(&self) -> &dma1::IFCR {
                            unsafe { &(*$DMAX::ptr()).ifcr }
                        }

                        fn get_ndtr(&self) -> u32 {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { &(*$DMAX::ptr())}.$chX.ndtr.read().bits()
                        }

                        fn get_flags(&self) -> dma::Flags {
                            dma::Flags::from_bits_truncate(self.isr().bits() >> ($x - 1))
                        }

                        fn clear_flags(&self, flags: dma::Flags) {
                            self.ifcr().write(|w| unsafe { w.bits(flags.bits() << ($x - 1)) });
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Channels = Channels;

                    fn split(self, ahb: &mut AHB) -> Channels {
                        $DMAX::enable(ahb);

                        // reset the DMA control registers (stops all on-going transfers)
                        $(
                            self.$chX.cr.reset();
                        )+

                        Channels((), $($CX { _0: () }),+)
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

pub trait ChannelLowLevel {
    /// Associated peripheral `address`
    ///
    /// `inc` indicates whether the address will be incremented after every byte transfer
    fn set_peripheral_address(&mut self, address: u32, inc: bool);

    /// `address` where from/to data will be read/write
    ///
    /// `inc` indicates whether the address will be incremented after every byte transfer
    fn set_memory_address(&mut self, address: u32, inc: bool);

    /// Number of bytes to transfer
    fn set_transfer_length(&mut self, len: usize);

    /// Starts the DMA transfer
    fn start(&mut self);

    /// Stops the DMA transfer
    fn stop(&mut self);

    /// Returns `true` if there's a transfer in progress
    fn in_progress(&self) -> bool;

    fn listen(&mut self, event: Event);

    fn unlisten(&mut self, event: Event);

    fn ch(&mut self) -> &crate::pac::dma1::CH;

    fn isr(&self) -> crate::pac::dma1::isr::R;

    fn ifcr(&self) -> &crate::pac::dma1::IFCR;

    fn get_ndtr(&self) -> u32;

    fn get_flags(&self) -> Flags;
    fn clear_flags(&self, flags: Flags);
}

impl<B, PAYLOAD, CX> CircBuffer<B, RxDma<PAYLOAD, CX>>
where
    CX: ChannelLowLevel,
    RxDma<PAYLOAD, CX>: TransferPayload,
{
    /// Peeks into the readable half of the buffer
    pub fn peek<R, F>(&mut self, f: F) -> Result<R, Error>
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
    pub fn readable_half(&mut self) -> Result<Half, Error> {
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
