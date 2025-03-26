//! DMA2D is the Chrom-ART Accelerator which is a specialized DMA dedicated to image manipulation.
//! Based on section 20 of reference manual rm0456

// This peripheral can be used in conjunction with the LTDC peripheral to achieve high frame rates when rendering frames to be sent to an LCD display
// It can perform the following operations:
// - Filling a part or the whole of a destination image with a specific color (see fill_rect)
// - Copying a part or the whole of a source image into a part or the whole of a destination image with or without pixel format conversion (see transfer_image)
// - Blending a part and/or two complete source images with different pixel format and copy the result into a part or the whole of a destination image with a different color format (see transfer_blended_image)
//
// Both async and blocking modes of operation are supported
// The source and destination image buffers need to be 4 byte aligned and your mcu may have special requirements about what memory can be used.
// You typically reserve specific memory regions using a linker script. Also, take care when enabling DCACHE as it may interfere with DMA.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
use stm32_metapac::dma2d::vals::*;

use crate::interrupt::typelevel::Interrupt;
use crate::interrupt::{self};
use crate::{peripherals, rcc, Peripheral};

static DMA2D_WAKER: AtomicWaker = AtomicWaker::new();

/// DMA2D error
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// CLUT access error
    ClutAccessError,
    /// Configuration error
    /// Refer to reference manual "rm0456" | "DMA2D configuration" | "Configuration error detection" for all the possible causes of this error
    /// For example, you will get a configuration error if the src and dst buffers are not correctly aligned in memory
    ConfigurationError,
    /// Transfer error
    TransferError,
    /// Unexpected result
    Unexpected(&'static str),
    /// Invalid source data
    InvalidSourceData(&'static str),
    /// Output color pixel data type does not match destination buffer
    OutputColorTypeMismatch(&'static str),
    /// Invalid destination buffer
    InvalidDestBuffer(&'static str),
    /// Destination buffer is not 4 byte alligned which is required for DMA2D
    DstBufferNotAligned,
    /// CLUT buffer format error
    ClutSizeError(&'static str),
}

/// Dma2d driver.
pub struct Dma2d<'d, T: Instance> {
    _peri: PeripheralRef<'d, T>,
    config: Option<Dma2dConfiguration>,
}

/// DMA2D interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

/// DMA2D instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + Peripheral<P = Self> + crate::rcc::RccPeripheral + 'static + Send {
    /// Interrupt for this instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

impl<'d, T: Instance> Drop for Dma2d<'d, T> {
    fn drop(&mut self) {}
}

trait SealedInstance: crate::rcc::SealedRccPeripheral {
    fn regs() -> crate::pac::dma2d::Dma2d;
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        cortex_m::asm::dsb();
        Dma2d::<T>::enable_interrupts(false);
        DMA2D_WAKER.wake();
    }
}

impl<'d, T: Instance> Dma2d<'d, T> {
    /// Create a new DMA2D driver
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        Self::setup_clocks();
        into_ref!(peri);
        Self {
            _peri: peri,
            config: None,
        }
    }

    /// Fill rectangle with given color
    pub async fn transfer_rect<'a, TDstPixel>(
        &mut self,
        src: FilledRect,
        dst: &'a mut [TDstPixel],
        config: DstImgConfig,
    ) -> Result<(), Error> {
        if self.transfer_rect_inner(src, dst, config)? {
            self.transfer().await?;
        }

        Ok(())
    }

    /// Fill rectangle with given color blocking and spin wait for completion
    pub fn transfer_rect_blocking<'a, TDstPixel>(
        &mut self,
        src: FilledRect,
        dst: &'a mut [TDstPixel],
        config: DstImgConfig,
    ) -> Result<(), Error> {
        if self.transfer_rect_inner(src, dst, config)? {
            // start the transfer
            T::regs().cr().modify(|w| w.set_start(CrStart::START));
            self.wait_for_transfer_blocking()?;
        }

        Ok(())
    }

    /// Transfer image
    pub async fn transfer_image<'a, TSrcPixel, TDstPixel>(
        &mut self,
        src: SrcFgImage<'a, TSrcPixel>,
        dst: DstBuffer<'a, TDstPixel>,
        enable_pixel_format_conversion: bool,
    ) -> Result<(), Error> {
        if self.transfer_image_inner(src, dst, enable_pixel_format_conversion)? {
            self.transfer().await?;
        }

        Ok(())
    }

    /// Transfer image blocking and spin wait until completion
    pub fn transfer_image_blocking<'a, TSrcPixel, TDstPixel>(
        &mut self,
        src: SrcFgImage<'a, TSrcPixel>,
        dst: DstBuffer<'a, TDstPixel>,
        enable_pixel_format_conversion: bool,
    ) -> Result<(), Error> {
        if self.transfer_image_inner(src, dst, enable_pixel_format_conversion)? {
            // start the transfer
            T::regs().cr().modify(|w| w.set_start(CrStart::START));
            self.wait_for_transfer_blocking()?;
        }

        Ok(())
    }

    /// Transfer blended image
    pub async fn transfer_blended_image<'a, TSrcFgPixel, TSrcBgPixel, TDstPixel>(
        &mut self,
        src: BlendSrc<'a, TSrcFgPixel, TSrcBgPixel>,
        dst: DstBuffer<'a, TDstPixel>,
    ) -> Result<(), Error> {
        if self.transfer_blended_image_inner(src, dst)? {
            self.transfer().await?;
        }

        Ok(())
    }

    /// Transfer blended image and spin wait until completion
    pub fn transfer_blended_image_blocking<'a, TSrcFgPixel, TSrcBgPixel, TDstPixel>(
        &mut self,
        src: BlendSrc<'a, TSrcFgPixel, TSrcBgPixel>,
        dst: DstBuffer<'a, TDstPixel>,
    ) -> Result<(), Error> {
        if self.transfer_blended_image_inner(src, dst)? {
            // start the transfer
            T::regs().cr().modify(|w| w.set_start(CrStart::START));
            self.wait_for_transfer_blocking()?;
        }

        Ok(())
    }

    /// Load color lookup table into clut registers
    pub async fn load_clut<TPixel>(
        &mut self,
        clut: &[TPixel],
        layer: Layer,
        color_mode: ClutColorMode,
    ) -> Result<(), Error> {
        // setup clut transfer
        self.load_clut_inner(clut, layer, color_mode)?;

        // load clut async
        self.clut_transfer(layer).await
    }

    /// Load color lookup table into clut registers
    pub fn load_clut_blocking<TPixel>(
        &mut self,
        clut: &[TPixel],
        layer: Layer,
        color_mode: ClutColorMode,
    ) -> Result<(), Error> {
        // setup clut transfer
        self.load_clut_inner(clut, layer, color_mode)?;

        // start clut transfer
        let dma2d = T::regs();
        match layer {
            Layer::Foreground => {
                dma2d.fgpfccr().modify(|w| {
                    w.set_cs(clut.len() as u8);
                });
            }
            Layer::Background => {
                dma2d.bgpfccr().modify(|w| {
                    w.set_cs(clut.len() as u8);
                });
            }
        }

        // spin wait for completion
        self.wait_for_transfer_blocking()
    }

    /// Limits bandwidth by inserting a wait time (in AHB clock cycles) between two consecutive accesses on the AHB master port.
    /// By default this is disabled
    pub fn set_dead_time(&mut self, dead_time: DeadTime) {
        let dma2d = T::regs();

        dma2d.amtcr().modify(|w| match dead_time {
            DeadTime::Disabled => {
                w.set_dt(0);
                w.set_en(false);
            }
            DeadTime::Enabled(num_ticks) => {
                w.set_dt(num_ticks);
                w.set_en(true);
            }
        });
    }

    /// Fill rectangle with given color
    fn transfer_rect_inner<'a, TDstPixel>(
        &mut self,
        src: FilledRect,
        dst: &'a mut [TDstPixel],
        config: DstImgConfig,
    ) -> Result<bool, Error> {
        if dst.len() != (config.width as usize * config.height as usize) {
            return Err(Error::InvalidDestBuffer(
                "destination buffer length does not match given width and height",
            ));
        }

        if let Some(offsets) = calculate_offsets(src.x, src.y, src.width, src.height, config.width, config.height) {
            let (color_mode, size) = match &src.color {
                OutputColor::Argb1555(_) => (OutputColorMode::Argb1555, size_of::<OcArgb1555>()),
                OutputColor::Argb4444(_) => (OutputColorMode::Argb4444, size_of::<OcArgb4444>()),
                OutputColor::Argb8888(_) => (OutputColorMode::Argb8888, size_of::<OcArgb8888>()),
                OutputColor::Rgb565(_) => (OutputColorMode::Rgb565, size_of::<OcRgb565>()),
                OutputColor::Rgb888(_) => (OutputColorMode::Rgb888, size_of::<OcRgb888>()),
            };

            if color_mode != config.color_mode {
                return Err(Error::OutputColorTypeMismatch(
                    "rect color does not match dst color mode",
                ));
            }

            if size != size_of::<TDstPixel>() {
                return Err(Error::OutputColorTypeMismatch(
                    "rect color does not match output pixel size",
                ));
            }

            let config = Dma2dConfiguration {
                transfer_mode: TransferMode::RegisterToMemory,
                line_offset_mode: LineOffsetMode::Pixels,
                output_offset: offsets.dst.line_offset,
                color_mode: config.color_mode,
                alpha_inverted: config.alpha_inverted,
                bytes_swap: config.bytes_swap,
                red_blue_swap: config.red_blue_swap,
            };
            self.init(config);

            let dst_addr = (&dst[offsets.dst.start_offset..]).as_ptr() as u32;

            let dma2d = T::regs();
            dma2d.ocolr().modify(|w| {
                let val = match src.color {
                    OutputColor::Argb1555(x) => x.0 as u32,
                    OutputColor::Argb4444(x) => x.0 as u32,
                    OutputColor::Argb8888(x) => x.0,
                    OutputColor::Rgb565(x) => x.0 as u32,
                    OutputColor::Rgb888(x) => x.0 as u32,
                };
                w.set_color(val)
            });

            self.setup_output(dst_addr, offsets.width, offsets.height);
            return Ok(true);
        }

        Ok(false)
    }

    /// Transfer image
    fn transfer_image_inner<'a, TSrcPixel, TDstPixel>(
        &mut self,
        src: SrcFgImage<'a, TSrcPixel>,
        dst: DstBuffer<'a, TDstPixel>,
        enable_pixel_format_conversion: bool,
    ) -> Result<bool, Error> {
        if src.addr.len() != (src.width as usize * src.height as usize) {
            return Err(Error::InvalidSourceData(
                "source buffer length does not match given width and height",
            ));

            // NOTE: we could also check that the user passed the correct pixel data type but this has not yet been implemented
        }

        if let Some(offsets) = calculate_offsets(src.x, src.y, src.width, src.height, dst.width, dst.height) {
            let transfer_mode = if enable_pixel_format_conversion {
                TransferMode::MemoryToMemoryPfc
            } else {
                TransferMode::MemoryToMemory
            };
            let config = Dma2dConfiguration {
                output_offset: offsets.dst.line_offset,
                transfer_mode,
                line_offset_mode: LineOffsetMode::Pixels,
                color_mode: dst.color_mode,
                alpha_inverted: dst.config.alpha_inverted,
                bytes_swap: dst.config.bytes_swap,
                red_blue_swap: dst.config.red_blue_swap,
            };
            self.init(config);

            let layer_config = LayerConfig {
                color_mode: LayerColorMode::Foreground(src.color_mode),
                line_offset: offsets.src.line_offset,
                alpha_inversion: src.config.alpha_inversion,
                alpha_mode: src.config.alpha_mode,
                alpha: src.config.alpha,
                red_blue_swap: src.config.red_blue_swap,
            };
            self.configure_layer(layer_config);

            let src_addr = &src.addr[offsets.src.start_offset..];
            let dst_addr = &dst.addr[offsets.dst.start_offset..];

            let dma2d = T::regs();
            dma2d.fgmar().modify(|w| w.set_ma(src_addr.as_ptr() as u32));
            self.setup_output(dst_addr.as_ptr() as u32, offsets.width, offsets.height);
            return Ok(true);
        }

        Ok(false)
    }

    /// Transfer blended image
    fn transfer_blended_image_inner<'a, TSrcFgPixel, TSrcBgPixel, TDstPixel>(
        &mut self,
        src: BlendSrc<'a, TSrcFgPixel, TSrcBgPixel>,
        dst: DstBuffer<'a, TDstPixel>,
    ) -> Result<bool, Error> {
        let dma2d = T::regs();

        if let Some((transfer_mode, output_offset)) = match src {
            BlendSrc::BlendFgFixedColorBgImg(color, src_bg) => {
                if let Some(offsets) =
                    calculate_offsets(src_bg.x, src_bg.y, src_bg.width, src_bg.height, dst.width, dst.height)
                {
                    dma2d.fgcolr().modify(|w| {
                        w.set_red(color.red);
                        w.set_green(color.green);
                        w.set_blue(color.blue);
                    });
                    let bg_layer = LayerConfig {
                        color_mode: LayerColorMode::Background(src_bg.color_mode),
                        line_offset: offsets.src.line_offset,
                        ..Default::default()
                    };
                    self.configure_layer(bg_layer);

                    let src_addr = &src_bg.addr[offsets.src.start_offset..];
                    let dst_addr = &dst.addr[offsets.dst.start_offset..];

                    dma2d.bgmar().modify(|x| x.set_ma(src_addr.as_ptr() as u32));
                    self.setup_output(dst_addr.as_ptr() as u32, offsets.width, offsets.height);
                    Some((
                        TransferMode::MemoryToMemoryPfcBlendingFixedColorFg,
                        offsets.dst.line_offset,
                    ))
                } else {
                    None
                }
            }
            BlendSrc::BlendFgImgBgFixedColor(src_fg, color) => {
                if let Some(offsets) =
                    calculate_offsets(src_fg.x, src_fg.y, src_fg.width, src_fg.height, dst.width, dst.height)
                {
                    dma2d.bgcolr().modify(|w| {
                        w.set_red(color.red);
                        w.set_green(color.green);
                        w.set_blue(color.blue);
                    });

                    let layer_config = LayerConfig {
                        color_mode: LayerColorMode::Foreground(src_fg.color_mode),
                        line_offset: offsets.src.line_offset,
                        ..Default::default()
                    };
                    self.configure_layer(layer_config);

                    let src_addr = &src_fg.addr[offsets.src.start_offset..];
                    let dst_addr = &dst.addr[offsets.dst.start_offset..];

                    dma2d.fgmar().modify(|w| w.set_ma(src_addr.as_ptr() as u32));
                    self.setup_output(dst_addr.as_ptr() as u32, offsets.width, offsets.height);
                    Some((
                        TransferMode::MemoryToMemoryPfcBlendingFixedColorBg,
                        offsets.dst.line_offset,
                    ))
                } else {
                    None
                }
            }
            BlendSrc::BlendFgImgBgImg(src_fg, src_bg) => {
                if let Some(offsets) = calculate_offsets_multi(&src_fg, &src_bg, &dst) {
                    let fg_layer = LayerConfig {
                        color_mode: LayerColorMode::Foreground(src_fg.color_mode),
                        line_offset: offsets.fg.line_offset,
                        ..Default::default()
                    };
                    self.configure_layer(fg_layer);
                    let bg_layer = LayerConfig {
                        color_mode: LayerColorMode::Background(src_bg.color_mode),
                        line_offset: offsets.bg.line_offset,
                        ..Default::default()
                    };
                    self.configure_layer(bg_layer);

                    let src_fg_addr = &src_fg.addr[offsets.fg.start_offset..];
                    let src_bg_addr = &src_bg.addr[offsets.bg.start_offset..];
                    let dst_addr = &dst.addr[offsets.dst.start_offset..];

                    dma2d.fgmar().modify(|w| w.set_ma(src_fg_addr.as_ptr() as u32));
                    dma2d.bgmar().modify(|w| w.set_ma(src_bg_addr.as_ptr() as u32));

                    self.setup_output(dst_addr.as_ptr() as u32, offsets.width, offsets.height);

                    Some((TransferMode::MemoryToMemoryPfcBlending, offsets.dst.line_offset))
                } else {
                    None
                }
            }
        } {
            let config = Dma2dConfiguration {
                transfer_mode,
                line_offset_mode: LineOffsetMode::Pixels,
                output_offset,
                color_mode: dst.color_mode,
                alpha_inverted: dst.config.alpha_inverted,
                bytes_swap: dst.config.bytes_swap,
                red_blue_swap: dst.config.red_blue_swap,
            };
            self.init(config);
            return Ok(true);
        }

        Ok(false)
    }

    fn load_clut_inner<TPixel>(
        &mut self,
        clut: &[TPixel],
        layer: Layer,
        color_mode: ClutColorMode,
    ) -> Result<(), Error> {
        let dma2d = T::regs();

        let size = size_of::<TPixel>();
        if size != 3 || size != 4 {
            return Err(Error::ClutSizeError(
                "CLUT pixel size must be 32 or 24 bits (ARGB8888 or RGB888)",
            ));
        }

        if clut.len() > 256 {
            return Err(Error::ClutSizeError("CLUT buffer must be 0-256 in length"));
        }

        match layer {
            Layer::Foreground => {
                // set the start address of the clut buffer
                dma2d.fgcmar().modify(|w| w.set_ma(clut.as_ptr() as u32));

                // forground pixel format converter control register
                dma2d.fgpfccr().modify(|w| {
                    w.set_ccm(match color_mode {
                        ClutColorMode::ARGB8888 => FgpfccrCcm::ARGB8888,
                        ClutColorMode::RGB888 => FgpfccrCcm::RGB888,
                    });
                    w.set_cs(clut.len() as u8);
                });
            }
            Layer::Background => {
                // set the start address of the clut buffer
                dma2d.bgcmar().modify(|w| w.set_ma(clut.as_ptr() as u32));

                // background pixel format converter control register
                dma2d.bgpfccr().modify(|w| {
                    w.set_ccm(match color_mode {
                        ClutColorMode::ARGB8888 => BgpfccrCcm::ARGB8888,
                        ClutColorMode::RGB888 => BgpfccrCcm::RGB888,
                    });
                    w.set_cs(clut.len() as u8);
                });
            }
        }

        Ok(())
    }

    fn wait_for_transfer_blocking(&self) -> Result<(), Error> {
        let dma2d = T::regs();

        loop {
            let status = dma2d.isr().read();
            if status.caeif() {
                return Err(Error::ClutAccessError);
            } else if status.ceif() {
                return Err(Error::ConfigurationError);
            } else if status.ctcif() {
                // CLUT transfer complete
                return Ok(());
            } else if status.tcif() {
                // transfer complete
                return Ok(());
            } else if status.teif() {
                return Err(Error::TransferError);
            } else if status.twif() {
                // transfer watermark complete
                return Ok(());
            } else {
                continue;
            }
        }
    }

    /// Start a dma2d clut transfer
    async fn clut_transfer(&self, layer: Layer) -> Result<(), Error> {
        let mut status = T::regs().isr().read();

        // if all interrupt flags are clear
        if !status.caeif() && !status.ceif() && !status.ctcif() && !status.tcif() && !status.teif() && !status.twif() {
            // wait for interrupt
            poll_fn(|cx| {
                let dma2d = T::regs();
                // quick check to avoid registration if already done.
                let status = dma2d.isr().read();
                if status.caeif() || status.ceif() || status.ctcif() || status.tcif() || status.teif() || status.twif()
                {
                    return Poll::Ready(());
                }

                DMA2D_WAKER.register(cx.waker());

                Self::clear_interrupt_flags(); // don't poison the request with old flags
                Self::enable_interrupts(true);

                // start the clut transfer
                match layer {
                    Layer::Foreground => {
                        dma2d.bgpfccr().modify(|w| {
                            w.set_start(BgpfccrStart::START);
                        });
                    }
                    Layer::Background => {
                        dma2d.fgpfccr().modify(|w| {
                            w.set_start(FgpfccrStart::START);
                        });
                    }
                }

                // need to check condition after register to avoid a race
                // condition that would result in lost notifications.
                let status = dma2d.isr().read();
                if status.caeif() || status.ceif() || status.ctcif() || status.tcif() || status.teif() || status.twif()
                {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;

            // re-read the status register after wait.
            status = T::regs().isr().read();
        }

        let result = if status.caeif() {
            Err(Error::ClutAccessError)
        } else if status.ceif() {
            Err(Error::ConfigurationError)
        } else if status.ctcif() {
            // CLUT transfer complete
            Ok(())
        } else if status.tcif() {
            // transfer complete
            Ok(())
        } else if status.teif() {
            Err(Error::TransferError)
        } else if status.twif() {
            // transfer watermark complete
            Ok(())
        } else {
            unreachable!("all interrupt status values checked")
        };

        Self::enable_interrupts(false);
        Self::clear_interrupt_flags();
        result
    }

    /// Initialise and enable the peripheral
    fn init(&mut self, config: Dma2dConfiguration) {
        let dma2d = T::regs();

        // set the transfer mode on the control register
        dma2d.cr().modify(|w| {
            w.set_mode(match config.transfer_mode {
                TransferMode::MemoryToMemory => Mode::MEMORY_TO_MEMORY,
                TransferMode::MemoryToMemoryPfc => Mode::MEMORY_TO_MEMORY_PFC,
                TransferMode::MemoryToMemoryPfcBlending => Mode::MEMORY_TO_MEMORY_PFCBLENDING,
                TransferMode::RegisterToMemory => Mode::REGISTER_TO_MEMORY,
                TransferMode::MemoryToMemoryPfcBlendingFixedColorFg => {
                    Mode::MEMORY_TO_MEMORY_PFCBLENDING_FIXED_COLOR_FG
                }
                TransferMode::MemoryToMemoryPfcBlendingFixedColorBg => {
                    Mode::MEMORY_TO_MEMORY_PFCBLENDING_FIXED_COLOR_BG
                }
            });
            w.set_lom(match config.line_offset_mode {
                LineOffsetMode::_Bytes => Lom::BYTES,
                LineOffsetMode::Pixels => Lom::PIXELS,
            });
        });

        // configure output pixel format converter
        dma2d.opfccr().modify(|w| {
            w.set_cm(match config.color_mode {
                OutputColorMode::Argb8888 => OpfccrCm::ARGB8888,
                OutputColorMode::Rgb888 => OpfccrCm::RGB888,
                OutputColorMode::Rgb565 => OpfccrCm::RGB565,
                OutputColorMode::Argb1555 => OpfccrCm::ARGB1555,
                OutputColorMode::Argb4444 => OpfccrCm::ARGB4444,
            });
            w.set_sb(match config.bytes_swap {
                BytesSwap::Regular => Sb::REGULAR,
                BytesSwap::Swap => Sb::SWAP_BYTES,
            });
            w.set_ai(match config.alpha_inverted {
                AlphaInversion::Regular => OpfccrAi::REGULAR_ALPHA,
                AlphaInversion::Inverted => OpfccrAi::INVERTED_ALPHA,
            });
            w.set_rbs(match config.red_blue_swap {
                RedBlueSwap::Regular => OpfccrRbs::REGULAR,
                RedBlueSwap::Swap => OpfccrRbs::SWAP,
            });
        });

        dma2d.oor().modify(|w| w.set_lo(config.output_offset));
        self.config = Some(config);
    }

    fn setup_output(&self, dst_addr: u32, width: u16, height: u16) {
        let dma2d = T::regs();

        // set width and height
        dma2d.nlr().modify(|w| {
            w.set_pl(width);
            w.set_nl(height);
        });

        // set destination address
        dma2d.omar().modify(|w| w.set_ma(dst_addr));
    }

    /// Configure forground or background layers
    fn configure_layer(&self, config: LayerConfig) {
        let dma2d = T::regs();

        match config.color_mode {
            LayerColorMode::Foreground(color_mode) => {
                dma2d.fgpfccr().modify(|w| {
                    w.set_alpha(config.alpha);
                    w.set_am(match config.alpha_mode {
                        AlphaMode::NoModify => FgpfccrAm::NO_MODIFY,
                        AlphaMode::Replace => FgpfccrAm::REPLACE,
                        AlphaMode::Multiply => FgpfccrAm::MULTIPLY,
                    });
                    w.set_ai(match config.alpha_inversion {
                        AlphaInversion::Regular => FgpfccrAi::REGULAR_ALPHA,
                        AlphaInversion::Inverted => FgpfccrAi::INVERTED_ALPHA,
                    });
                    w.set_rbs(match config.red_blue_swap {
                        RedBlueSwap::Regular => FgpfccrRbs::REGULAR,
                        RedBlueSwap::Swap => FgpfccrRbs::SWAP,
                    });
                    w.set_cm(match color_mode {
                        FgColorMode::Argb8888 => FgpfccrCm::ARGB8888,
                        FgColorMode::Rgb888 => FgpfccrCm::RGB888,
                        FgColorMode::Rgb565 => FgpfccrCm::RGB565,
                        FgColorMode::Argb1555 => FgpfccrCm::ARGB1555,
                        FgColorMode::Argb4444 => FgpfccrCm::ARGB4444,
                        FgColorMode::L8 => FgpfccrCm::L8,
                        FgColorMode::Al44 => FgpfccrCm::AL44,
                        FgColorMode::Al88 => FgpfccrCm::AL88,
                        FgColorMode::L4 => FgpfccrCm::L4,
                        FgColorMode::A8 => FgpfccrCm::A8,
                        FgColorMode::A4 => FgpfccrCm::A4,
                        FgColorMode::YcbCr => FgpfccrCm::YCB_CR,
                    });
                });
                dma2d.fgor().modify(|w| w.set_lo(config.line_offset));
            }
            LayerColorMode::Background(color_mode) => {
                dma2d.bgpfccr().modify(|w| {
                    w.set_alpha(config.alpha);
                    w.set_am(match config.alpha_mode {
                        AlphaMode::NoModify => BgpfccrAm::NO_MODIFY,
                        AlphaMode::Replace => BgpfccrAm::REPLACE,
                        AlphaMode::Multiply => BgpfccrAm::MULTIPLY,
                    });
                    w.set_ai(match config.alpha_inversion {
                        AlphaInversion::Regular => BgpfccrAi::REGULAR_ALPHA,
                        AlphaInversion::Inverted => BgpfccrAi::INVERTED_ALPHA,
                    });
                    w.set_rbs(match config.red_blue_swap {
                        RedBlueSwap::Regular => BgpfccrRbs::REGULAR,
                        RedBlueSwap::Swap => BgpfccrRbs::SWAP,
                    });
                    w.set_cm(match color_mode {
                        BgColorMode::Argb8888 => BgpfccrCm::ARGB8888,
                        BgColorMode::Rgb888 => BgpfccrCm::RGB888,
                        BgColorMode::Rgb565 => BgpfccrCm::RGB565,
                        BgColorMode::Argb1555 => BgpfccrCm::ARGB1555,
                        BgColorMode::Argb4444 => BgpfccrCm::ARGB4444,
                        BgColorMode::L8 => BgpfccrCm::L8,
                        BgColorMode::Al44 => BgpfccrCm::AL44,
                        BgColorMode::Al88 => BgpfccrCm::AL88,
                        BgColorMode::L4 => BgpfccrCm::L4,
                        BgColorMode::A8 => BgpfccrCm::A8,
                        BgColorMode::A4 => BgpfccrCm::A4,
                    });
                    w.set_alpha(0);
                    w.set_am(BgpfccrAm::NO_MODIFY);
                    w.set_ai(BgpfccrAi::REGULAR_ALPHA);
                    w.set_rbs(BgpfccrRbs::REGULAR);
                    w.set_cm(BgpfccrCm::RGB565);
                });
                dma2d.bgor().modify(|w| w.set_lo(config.line_offset));
            }
        }
    }

    /// Start a dma2d transfer
    /// This is the async equivalent of calling start_transfer followed by repeatedly calling poll_transfer until completion
    /// The benefit of this function is that you don't have to hold up the cpu while waiting for completion
    async fn transfer(&self) -> Result<(), Error> {
        let mut status = T::regs().isr().read();

        // if all interrupt flags are clear
        if !status.caeif() && !status.ceif() && !status.ctcif() && !status.tcif() && !status.teif() && !status.twif() {
            // wait for interrupt
            poll_fn(|cx| {
                let dma2d = T::regs();
                // quick check to avoid registration if already done.
                let status = dma2d.isr().read();
                if status.caeif() || status.ceif() || status.ctcif() || status.tcif() || status.teif() || status.twif()
                {
                    return Poll::Ready(());
                }

                DMA2D_WAKER.register(cx.waker());

                Self::clear_interrupt_flags(); // don't poison the request with old flags
                Self::enable_interrupts(true);

                // start the transfer
                dma2d.cr().modify(|w| w.set_start(CrStart::START));

                // need to check condition after register to avoid a race
                // condition that would result in lost notifications.
                let status = dma2d.isr().read();
                if status.caeif() || status.ceif() || status.ctcif() || status.tcif() || status.teif() || status.twif()
                {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;

            // re-read the status register after wait.
            status = T::regs().isr().read();
        }

        let result = if status.caeif() {
            Err(Error::ClutAccessError)
        } else if status.ceif() {
            Err(Error::ConfigurationError)
        } else if status.ctcif() {
            // CLUT transfer complete
            Ok(())
        } else if status.tcif() {
            // transfer complete
            Ok(())
        } else if status.teif() {
            Err(Error::TransferError)
        } else if status.twif() {
            // transfer watermark complete
            Ok(())
        } else {
            unreachable!("all interrupt status values checked")
        };

        Self::enable_interrupts(false);
        Self::clear_interrupt_flags();
        result
    }

    fn setup_clocks() {
        rcc::enable_and_reset::<T>();
    }

    fn clear_interrupt_flags() {
        T::regs().ifcr().modify(|w| {
            w.set_caecif(Caecif::CLEAR);
            w.set_cceif(Cceif::CLEAR);
            w.set_cctcif(Cctcif::CLEAR);
            w.set_ctcif(Ctcif::CLEAR);
            w.set_cteif(Cteif::CLEAR);
            w.set_ctwif(Ctwif::CLEAR);
        });
    }

    fn enable_interrupts(enable: bool) {
        T::regs().cr().modify(|w| {
            w.set_caeie(false);
            w.set_ceie(enable);
            w.set_ctcie(false);
            w.set_tcie(enable);
            w.set_teie(enable);
            w.set_twie(false);
        });

        // enable interrupts for DMA2D peripheral
        T::Interrupt::unpend();
        if enable {
            unsafe { T::Interrupt::enable() };
        } else {
            T::Interrupt::disable()
        }
    }
}

foreach_interrupt!(
    ($inst:ident, dma2d, DMA2D, GLOBAL, $irq:ident) => {
        impl Instance for peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }

        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::dma2d::Dma2d {
                crate::pac::$inst
            }
        }
    };
);

/// CLUT color mode - format of the pixels in the clut buffer
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClutColorMode {
    /// ARGB8888 - 32 bit
    ARGB8888,
    /// RGB888 - 24 bit
    RGB888,
}

/// Output color
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputColor {
    /// ARGB8888 color
    Argb8888(OcArgb8888),
    /// RGB888 color
    Rgb888(OcRgb888),
    /// RGB565 color
    Rgb565(OcRgb565),
    /// ARGB1555 color
    Argb1555(OcArgb1555),
    /// ARGB444 color
    Argb4444(OcArgb4444),
}

/// Output color RGB565
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OcRgb565(pub u16);

/// Output color RGB888 - 24 bits but stored in 32 bits
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// RGB888 color - 24 bits but stored in 32 bits
pub struct OcRgb888(pub u32);

/// Output color ARGB444
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OcArgb4444(pub u16);

/// Output color ARGB1555
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OcArgb1555(pub u16);

/// Output color ARGB8888
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OcArgb8888(pub u32);

/// Dead time in AHB clock ticks between two consecutive accesses on the AHB master port - limits bandwidth if enabled
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeadTime {
    /// Disabled
    Disabled,
    /// Enabled with specified number of AHB clock ticks for wait time
    Enabled(u8),
}

/// Source image configuration
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SrcImgConfig {
    /// Alpha value (0-255 transparrent-opaque)
    pub alpha: u8,
    /// Alpha mode
    pub alpha_mode: AlphaMode,
    /// Alpha inversion
    pub alpha_inversion: AlphaInversion,
    /// Red blue swap
    pub red_blue_swap: RedBlueSwap,
}

impl Default for SrcImgConfig {
    fn default() -> Self {
        Self {
            alpha: 0xFF,                     // fully opaque
            alpha_mode: AlphaMode::NoModify, // alpha not applied
            alpha_inversion: AlphaInversion::Regular,
            red_blue_swap: RedBlueSwap::Regular,
        }
    }
}

/// Destination image configuration
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DstImgConfig {
    /// width in pixels
    pub width: u16,
    /// height in pixels
    pub height: u16,
    /// color mode
    pub color_mode: OutputColorMode,
    /// Regular or inverted alpha value for the output pixel format converter
    pub alpha_inverted: AlphaInversion,
    /// Regular more (RGB or ARGB) or swap mode (BGR or ABGR) for the output pixel format converter
    pub red_blue_swap: RedBlueSwap,
    /// Byte regular mode or bytes swap mode (two by two)
    pub bytes_swap: BytesSwap,
}

impl Default for DstImgConfig {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            color_mode: OutputColorMode::Rgb565,
            alpha_inverted: AlphaInversion::Regular,
            red_blue_swap: RedBlueSwap::Regular,
            bytes_swap: BytesSwap::Regular,
        }
    }
}

/// Output color mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputColorMode {
    /// ARGB8888
    Argb8888,
    /// RBG888
    Rgb888,
    /// RGB565,
    Rgb565,
    /// ARGB1555,
    Argb1555,
    /// ARGB4444
    Argb4444,
}

/// Foreground layer color mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FgColorMode {
    /// ARGB8888
    Argb8888,
    /// RGB888
    Rgb888,
    /// RGB565
    Rgb565,
    /// ARGB1555
    Argb1555,
    /// ARGB4444
    Argb4444,
    /// L8
    L8,
    /// AL44
    Al44,
    /// AL88
    Al88,
    /// L4
    L4,
    /// A8
    A8,
    /// A4
    A4,
    /// YCbCr
    YcbCr,
}

/// Background layer color mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BgColorMode {
    /// ARGB8888
    Argb8888,
    /// RGB888
    Rgb888,
    /// RGB565
    Rgb565,
    /// ARGB1555
    Argb1555,
    /// ARGB4444
    Argb4444,
    /// L8
    L8,
    /// AL44
    Al44,
    /// AL88
    Al88,
    /// L4
    L4,
    /// A8
    A8,
    /// A4
    A4,
}

/// Alpha inversion
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AlphaInversion {
    /// Regular
    Regular,
    /// Inverted
    Inverted,
}

/// Red blue swap
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RedBlueSwap {
    /// Regular
    Regular,
    /// Swap
    Swap,
}

/// Bytes swap
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BytesSwap {
    /// Regular
    Regular,
    /// Swap
    Swap,
}

/// Layer config
pub struct LayerConfig {
    /// Alpha value
    pub alpha: u8,
    /// Alpha mode
    pub alpha_mode: AlphaMode,
    /// Alpha inversion
    pub alpha_inversion: AlphaInversion,
    /// Red blue swap
    pub red_blue_swap: RedBlueSwap,
    /// Color mode
    pub color_mode: LayerColorMode,
    /// Line offset
    pub line_offset: u16,
}

impl Default for LayerConfig {
    fn default() -> Self {
        Self {
            alpha: 0,
            alpha_mode: AlphaMode::NoModify,
            alpha_inversion: AlphaInversion::Regular,
            red_blue_swap: RedBlueSwap::Regular,
            color_mode: LayerColorMode::Foreground(FgColorMode::Rgb565),
            line_offset: 0,
        }
    }
}

/// Layer
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Layer {
    /// Foreground
    Foreground,
    /// Background
    Background,
}

/// Layer color mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LayerColorMode {
    /// Foreground color mode
    Foreground(FgColorMode),
    /// Background color mode
    Background(BgColorMode),
}

/// Alpha mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AlphaMode {
    /// No modify
    NoModify,
    /// Replace
    Replace,
    /// Multiply
    Multiply,
}

/// Filled rectangle
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FilledRect {
    /// top left x (negative is out of bounds and allowed)
    pub x: i32,
    /// top left y (negative is out of bounds and allowed)
    pub y: i32,
    /// width in pixels
    pub width: u16,
    /// height in pixels
    pub height: u16,
    /// fill color
    pub color: OutputColor,
}

impl FilledRect {
    /// Creates anew instance
    pub fn new(x: i32, y: i32, width: u16, height: u16, color: OutputColor) -> Self {
        Self {
            x,
            y,
            width,
            height,
            color,
        }
    }
}

/// Source foreground image
pub struct SrcFgImage<'a, TSrcPixel> {
    /// memory address of start of the image buffer
    pub addr: &'a [TSrcPixel],
    /// top left x (negative is out of bounds and allowed)
    pub x: i32,
    /// top left y (negative is out of bounds and allowed)
    pub y: i32,
    /// width in pixels
    pub width: u16,
    /// height in pixels
    pub height: u16,
    /// color format of the pixels in the buffer
    pub color_mode: FgColorMode,
    /// config
    pub config: SrcImgConfig,
}

impl<'a, TSrcPixel> SrcFgImage<'a, TSrcPixel> {
    /// Creates a new instance of the source
    pub fn new(
        addr: &'a [TSrcPixel],
        x: i32,
        y: i32,
        width: u16,
        height: u16,
        color_mode: FgColorMode,
        config: SrcImgConfig,
    ) -> Self {
        Self {
            addr,
            x,
            y,
            width,
            height,
            color_mode,
            config,
        }
    }
}

/// Source background image
pub struct SrcBgImage<'a, TSrcPixel> {
    /// memory address of start of the image buffer
    pub addr: &'a [TSrcPixel],
    /// top left x (negative is out of bounds and allowed)
    pub x: i32,
    /// top left y (negative is out of bounds and allowed)
    pub y: i32,
    /// width in pixels
    pub width: u16,
    /// height in pixels
    pub height: u16,
    /// color format of the pixels in the buffer
    pub color_mode: BgColorMode,
    /// config
    pub config: SrcImgConfig,
}

impl<'a, TSrcPixel> SrcBgImage<'a, TSrcPixel> {
    /// Creates a new instance of the source
    pub fn new(
        addr: &'a [TSrcPixel],
        x: i32,
        y: i32,
        width: u16,
        height: u16,
        color_mode: BgColorMode,
        config: SrcImgConfig,
    ) -> Self {
        Self {
            addr,
            x,
            y,
            width,
            height,
            color_mode,
            config,
        }
    }
}

/// Destination buffer
pub struct DstBuffer<'a, TPixel> {
    /// memory address of start of buffer
    pub addr: &'a mut [TPixel],
    /// width in pixels
    pub width: u16,
    /// height in pixels
    pub height: u16,
    /// color mode
    pub color_mode: OutputColorMode,
    /// extra config
    pub config: DstImgConfig,
}

impl<'a, TPixel> DstBuffer<'a, TPixel> {
    /// Creates a new instance of DestBuffer
    pub fn new(
        addr: &'a mut [TPixel],
        width: u16,
        height: u16,
        color_mode: OutputColorMode,
        config: DstImgConfig,
    ) -> Self {
        Self {
            addr,
            width,
            height,
            color_mode,
            config,
        }
    }
}

/// Layer color
pub struct LayerColor {
    /// Red
    pub red: u8,
    /// Green
    pub green: u8,
    /// Blue
    pub blue: u8,
}

impl LayerColor {
    /// Creates a new instance of a layer color
    pub fn new(red: u8, green: u8, blue: u8) -> Self {
        Self { red, green, blue }
    }
}

/// Blend source
pub enum BlendSrc<'a, TSrcFgPixel, TSrcBgPixel> {
    /// Blend foreground fixed color with background image
    BlendFgFixedColorBgImg(LayerColor, SrcBgImage<'a, TSrcBgPixel>),
    /// Blend foreground image with background fixed color
    BlendFgImgBgFixedColor(SrcFgImage<'a, TSrcFgPixel>, LayerColor),
    /// Blend foreground image with background image
    BlendFgImgBgImg(SrcFgImage<'a, TSrcFgPixel>, SrcBgImage<'a, TSrcBgPixel>),
}

#[derive(Debug)]
struct ImgOffsets {
    pub line_offset: u16,
    pub start_offset: usize,
}

impl ImgOffsets {
    pub fn new(line_offset: u16, start_offset: usize) -> Self {
        Self {
            line_offset,
            start_offset,
        }
    }
}

struct Offsets {
    pub src: ImgOffsets,
    pub dst: ImgOffsets,
    pub width: u16,
    pub height: u16,
}

fn calculate_offsets(
    src_x: i32,
    src_y: i32,
    src_width: u16,
    src_height: u16,
    dst_width: u16,
    dst_height: u16,
) -> Option<Offsets> {
    let x0 = src_x.max(0);
    let x1 = (src_x + src_width as i32).min(dst_width as i32);
    let y0 = src_y.max(0);
    let y1 = (src_y + src_height as i32).min(dst_height as i32);

    let width = x1 - x0;
    let height = y1 - y0;

    if width < 0 || height < 0 {
        // out of bounds
        None
    } else {
        let src1_offsets = ImgOffsets::new(
            src_width - width as u16,
            ((x0 - src_x) + (y0 - src_y) * src_width as i32) as usize,
        );
        let dst_offsets = ImgOffsets::new(dst_width - width as u16, (x0 + y0 * dst_width as i32) as usize);
        let offsets = Offsets {
            src: src1_offsets,
            dst: dst_offsets,
            width: width as u16,
            height: height as u16,
        };
        Some(offsets)
    }
}

struct OffsetsMulti {
    pub fg: ImgOffsets,
    pub bg: ImgOffsets,
    pub dst: ImgOffsets,
    pub width: u16,
    pub height: u16,
}

fn calculate_offsets_multi<T0, T1, T2>(
    src1: &SrcFgImage<'_, T0>,
    scr2: &SrcBgImage<'_, T1>,
    dst: &DstBuffer<'_, T2>,
) -> Option<OffsetsMulti> {
    let x0 = src1.x.max(scr2.x).max(0);
    let x1 = (src1.x + src1.width as i32)
        .min(scr2.x + scr2.width as i32)
        .min(dst.width as i32);
    let y0 = src1.y.max(scr2.y).max(0);
    let y1 = (src1.y + src1.height as i32)
        .min(scr2.y + scr2.height as i32)
        .min(dst.height as i32);

    let width = x1 - x0;
    let height = y1 - y0;

    if width < 0 || height < 0 {
        // out of bounds
        None
    } else {
        let src1_offsets = ImgOffsets::new(
            src1.width as u16 - width as u16,
            ((x0 - src1.x) + (y0 - src1.y) * src1.width as i32) as usize,
        );
        let src2_offsets = ImgOffsets::new(
            scr2.width as u16 - width as u16,
            ((x0 - scr2.x) + (y0 - scr2.y) * scr2.width as i32) as usize,
        );
        let dst_offsets = ImgOffsets::new(dst.width as u16 - width as u16, (x0 + y0 * dst.width as i32) as usize);
        let offsets = OffsetsMulti {
            fg: src1_offsets,
            bg: src2_offsets,
            dst: dst_offsets,
            width: width as u16,
            height: height as u16,
        };
        Some(offsets)
    }
}

impl OcArgb1555 {
    /// Valid values: alpha (0 or 1 - transparent or opaque) red (0-31) green (0-31) blue (0-31)
    pub const fn new(alpha: u8, red: u8, green: u8, blue: u8) -> Self {
        const A_BITS: usize = 1;
        const R_BITS: usize = 4;
        const G_BITS: usize = 4;
        const B_BITS: usize = 4;
        let a_shifted = shift_u16(alpha, A_BITS, R_BITS + G_BITS + B_BITS);
        let r_shifted = shift_u16(red, R_BITS, G_BITS + B_BITS);
        let g_shifted = shift_u16(green, G_BITS, B_BITS);
        let b_shifted = shift_u16(blue, B_BITS, 0);
        Self(a_shifted | r_shifted | g_shifted | b_shifted)
    }
}

impl OcArgb4444 {
    /// Valid values: alpha (0-15 transparent-opaque) red (0-15) green (0-15) blue (0-15)
    pub const fn new(alpha: u8, red: u8, green: u8, blue: u8) -> Self {
        const A_BITS: usize = 4;
        const R_BITS: usize = 4;
        const G_BITS: usize = 4;
        const B_BITS: usize = 4;
        let a_shifted = shift_u16(alpha, A_BITS, R_BITS + G_BITS + B_BITS);
        let r_shifted = shift_u16(red, R_BITS, G_BITS + B_BITS);
        let g_shifted = shift_u16(green, G_BITS, B_BITS);
        let b_shifted = shift_u16(blue, B_BITS, 0);
        Self(a_shifted | r_shifted | g_shifted | b_shifted)
    }
}

impl OcArgb8888 {
    /// Valid values: alpha (0-255 transparent-opaque) red (0-255) green (0-255) blue (0-255)
    pub const fn new(alpha: u8, red: u8, green: u8, blue: u8) -> Self {
        const A_BITS: usize = 8;
        const R_BITS: usize = 8;
        const G_BITS: usize = 8;
        const B_BITS: usize = 8;
        let a_shifted = shift_u32(alpha, A_BITS, R_BITS + G_BITS + B_BITS);
        let r_shifted = shift_u32(red, R_BITS, G_BITS + B_BITS);
        let g_shifted = shift_u32(green, G_BITS, B_BITS);
        let b_shifted = shift_u32(blue, B_BITS, 0);
        Self(a_shifted | r_shifted | g_shifted | b_shifted)
    }
}

impl OcRgb888 {
    /// Valid values: red (0-255) green (0-255) blue (0-255)
    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        const R_BITS: usize = 8;
        const G_BITS: usize = 8;
        const B_BITS: usize = 8;
        let r_shifted = shift_u32(red, R_BITS, G_BITS + B_BITS);
        let g_shifted = shift_u32(green, G_BITS, B_BITS);
        let b_shifted = shift_u32(blue, B_BITS, 0);
        Self(r_shifted | g_shifted | b_shifted)
    }
}

impl OcRgb565 {
    /// Valid values: red (0-31) green (0-63) blue (0-31)
    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        const R_BITS: usize = 5;
        const G_BITS: usize = 6;
        const B_BITS: usize = 5;
        let r_shifted = shift_u16(red, R_BITS, G_BITS + B_BITS);
        let g_shifted = shift_u16(green, G_BITS, B_BITS);
        let b_shifted = shift_u16(blue, B_BITS, 0);
        Self(r_shifted | g_shifted | b_shifted)
    }
}

const fn shift_u16(value: u8, num_bits: usize, shift_by: usize) -> u16 {
    let max = ((1usize << num_bits) - 1) as u8;
    ((value & max) as u16) << shift_by
}

const fn shift_u32(value: u8, num_bits: usize, shift_by: usize) -> u32 {
    let max = ((1usize << num_bits) - 1) as u8;
    ((value & max) as u32) << shift_by
}

/// Transfer mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum TransferMode {
    /// Memory to memory
    MemoryToMemory,
    /// Memory to memory with pixel format conversion (PFC)
    MemoryToMemoryPfc,
    /// Memory to memory with pixel format conversion (PFC) and blending
    MemoryToMemoryPfcBlending,
    /// Register to memory
    RegisterToMemory,
    /// TODO: restrict this to only the stm32 chips that support it (like stm32u5g9)
    /// Memory to memory with pixel format conversion (PFC), blending and fixed color foreground
    MemoryToMemoryPfcBlendingFixedColorFg,
    /// Memory to memory with pixel format conversion (PFC), blending and fixed color backgreound
    MemoryToMemoryPfcBlendingFixedColorBg,
}

/// Display configuration parameters
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Dma2dConfiguration {
    /// Transfer mode
    pub transfer_mode: TransferMode,
    /// Color format of the output image
    pub color_mode: OutputColorMode,
    /// Offset value between 0x0000 and 0x3FFF. This value is used for the address generation. It is added at the end of each line to determine the starting address of the next line.
    pub output_offset: u16,
    /// Regular or inverted alpha value for the output pixel format converter
    pub alpha_inverted: AlphaInversion,
    /// Regular more (RGB or ARGB) or swap mode (BGR or ABGR) for the output pixel format converter
    pub red_blue_swap: RedBlueSwap,
    /// Byte regular mode or bytes swap mode (two by two)
    pub bytes_swap: BytesSwap,
    /// Line offset for the foreground, background and output_offset
    pub line_offset_mode: LineOffsetMode,
}

impl Default for Dma2dConfiguration {
    fn default() -> Self {
        Self {
            transfer_mode: TransferMode::RegisterToMemory,
            color_mode: OutputColorMode::Argb8888,
            output_offset: 0,
            alpha_inverted: AlphaInversion::Regular,
            red_blue_swap: RedBlueSwap::Regular,
            bytes_swap: BytesSwap::Regular,
            line_offset_mode: LineOffsetMode::Pixels,
        }
    }
}

/// Line offset mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum LineOffsetMode {
    /// Line offset expressed in pixels
    Pixels,
    /// Line offset expressed in bytes
    _Bytes,
}

/// Chroma sub-sampling
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum _ChromaSubSampling {
    /// No chroma sub-sampling 4:4:4
    None,
    /// Chroma sub-sampling 4:2:2
    _422,
    /// Chroma sub-sampling 4:2:0
    _420,
}
