//! DMA2D is the Chrom-ART Accelerator which is a specialized DMA dedicated to image manipulation.

// This peripheral can be used in conjunction with the LTDC peripheral to achieve high frame rates when rendering images to be sent to an LCD display
// It can perform the following operations:
// - Filling a part or the whole of a destination image with a specific color
// - Copying a part or the whole of a source image into a part or the whole of a destination image
// - Copying a part or the whole of a source image into a part or the whole of a destination image with a pixel format conversion
// - Blending a part and/or two complete source images with different pixel format and copy the result into a part or the whole of a destination image with a different color format

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

/// Pixel offsets to be used when copying images
pub struct MemToMemPixelOffsets {
    /// Number of pixels to jump after each line
    pub output_offset: u16,
    /// Number of pixels to skip on each line when reading an imaged that is clipped to the right
    pub line_offset: u16,
    /// Number of pixels to skip before reading image data (should be multiplied by bytes_per_pixel to offset the memory address
    pub src_offset: u32,
    /// Number of pixels to skip before writing image data to the frame buffer
    pub dst_offset: u32,
    /// Width with clipping applied
    pub width: u16,
    /// Height with clipping applied
    pub height: u16,
}

/// Output color
pub enum OutputColor {
    /// ARGB8888 color
    Argb8888(Argb8888),

    /// RGB888 color
    Rgb888(Rgb888),

    /// RGB565 color
    Rgb565(Rgb565),

    /// ARGB1555 color
    Argb1555(Argb1555),

    /// ARGB444 color
    Argb4444(Argb4444),
}

/// RGB565 color
pub struct Rgb565(pub u16);

/// RGB888 color - 24 bits but stored in 32 bits
pub struct Rgb888(pub u32);

/// ARGB444 color
pub struct Argb4444(pub u16);

/// ARGB1555 color
pub struct Argb1555(pub u16);

/// ARGB8888 color
pub struct Argb8888(pub u32);

impl Argb1555 {
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

impl Argb4444 {
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

impl Argb8888 {
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

impl Rgb888 {
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

impl Rgb565 {
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

impl MemToMemPixelOffsets {
    /// Considers how an image is placed in the screen and calculates the required pixel offsets needed to setup the dma
    pub fn calculate(
        image_x: i32,
        image_y: i32,
        image_width: u16,
        image_height: u16,
        screen_width: u16,
        screen_height: u16,
    ) -> Option<Self> {
        let right_clip = screen_width as i32 - image_x;
        let bottom_clip = screen_height as i32 - image_y;
        let left_clip = image_width as i32 + image_x;
        let top_clip = image_height as i32 + image_y;

        if right_clip < 0 || bottom_clip < 0 || left_clip < 0 || top_clip < 0 {
            // nothing to draw as rectangle is out of bounds
            return None;
        }

        let right_clip = right_clip as u16;
        let bottom_clip = bottom_clip as u16;
        let left_clip = left_clip as u16;
        let top_clip = top_clip as u16;

        let x = if image_x < 0 { 0 } else { image_x as u32 };
        let y = if image_y < 0 { 0 } else { image_y as u32 };

        // clip the width and height if they go past the edge of the screen
        let width = image_width.min(right_clip).min(left_clip);
        let height = image_height.min(bottom_clip).min(top_clip);

        let src_offset_x = if image_x < 0 { image_x.abs() } else { 0 };
        let src_offset_y = if image_y < 0 { image_y.abs() } else { 0 };

        let output_offset = screen_width - width; // number of pixels to jump after each line
        let src_offset = src_offset_x as u32 + src_offset_y as u32 * image_width as u32;
        let dst_offset = x + y * screen_width as u32; // number of pixels to skip from the top left of screen
        let line_offset = image_width - width; // if image is clipped to the right

        /*
        info!(
            "x: {} y: {} width: {} height: {}, output_offset: {}, line_offset: {}, src_offset: {}, dst_offset: {}, left_clip: {}, top_clip: {}",
            image.x, image.y, width, height, output_offset, line_offset, src_offset, dst_offset,left_clip, top_clip
        );*/

        Some(Self {
            output_offset,
            line_offset,
            src_offset,
            dst_offset,
            width,
            height,
        })
    }
}

/// Pixel offsets to be used using RegisterToMemory operations
pub struct RegToMemPixelOffsets {
    /// Number of pixels to jump after each line
    pub output_offset: u16,
    /// Number of pixels to skip before writing data to the frame buffer
    pub dst_offset: u32,
    /// Width with clipping applied
    pub width: u16,
    /// Height with clipping applied
    pub height: u16,
}

impl RegToMemPixelOffsets {
    /// Considers how a filled rectangle is placed in the screen and calculates the required pixel offsets needed to setup the dma
    pub fn calculate(
        rect_x: i32,
        rect_y: i32,
        rect_width: u16,
        rect_height: u16,
        screen_width: u16,
        screen_height: u16,
    ) -> Option<Self> {
        let right_clip = screen_width as i32 - rect_x;
        let bottom_clip = screen_height as i32 - rect_y;
        let left_clip = rect_width as i32 + rect_x;
        let top_clip = rect_height as i32 + rect_y;

        if right_clip < 0 || bottom_clip < 0 || left_clip < 0 || top_clip < 0 {
            // nothing to draw as rectangle is out of bounds
            return None;
        }

        let right_clip = right_clip as u16;
        let bottom_clip = bottom_clip as u16;
        let left_clip = left_clip as u16;
        let top_clip = top_clip as u16;

        let x = if rect_x < 0 { 0 } else { rect_x as u32 };
        let y = if rect_y < 0 { 0 } else { rect_y as u32 };

        // clip the width and height if they go past the edge of the screen
        let width = rect_width.min(right_clip).min(left_clip);
        let height = rect_height.min(bottom_clip).min(top_clip);

        // clip the width and height if they go past the edge of the screen
        let output_offset = screen_width - width; // number of pixels to jump after each line
        let dst_offset = x as u32 + y as u32 * screen_width as u32; // number of pixels to skip from the top left of screen

        Some(Self {
            output_offset,
            dst_offset,
            width,
            height,
        })
    }
}

/// Display configuration parameters
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Dma2dConfiguration {
    /// Transfer mode
    pub transfer_mode: TransferMode,

    /// Color format of the output image
    pub color_mode: ColorMode,

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
            color_mode: ColorMode::Argb8888,
            output_offset: 0,
            alpha_inverted: AlphaInversion::Regular,
            red_blue_swap: RedBlueSwap::Regular,
            bytes_swap: BytesSwap::Regular,
            line_offset_mode: LineOffsetMode::Pixels,
        }
    }
}

/// Transfer mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransferMode {
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
}

/// Output color mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ColorMode {
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

/// Source data
pub enum SourceData {
    /// Output color used for RegisterToMemory transfers
    OutputColor(OutputColor),
    /// Source address used for all other transfers besides RegisterToMemory
    SrcAddress(&'static [u16]),
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

/// Line offset mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LineOffsetMode {
    /// Line offset expressed in pixels
    Pixels,
    /// Line offset expressed in bytes
    Bytes,
}

/// Chroma sub-sampling
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChromaSubSampling {
    /// No chroma sub-sampling 4:4:4
    None,
    /// Chroma sub-sampling 4:2:2
    _422,
    /// Chroma sub-sampling 4:2:0
    _420,
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

/// Layer color mode
pub enum LayerColorMode {
    /// Foreground color mode
    Foreground(FgColorMode),
    /// Background color mode
    Background(BgColorMode),
}

/// Alpha mode
pub enum AlphaMode {
    /// No modify
    NoModify,
    /// Replace
    Replace,
    /// Multiply
    Multiply,
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

impl<'d, T: Instance> Drop for Dma2d<'d, T> {
    fn drop(&mut self) {}
}

/// Poll result
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PollResult {
    /// Pending - Operation is still running
    Pending,
    /// Transfer complete
    TransferComplete,
    /// CLUT transfer complete
    ClutTransferComplete,
    /// Transfer watermark complete
    TransferWatermark,
}

trait SealedInstance: crate::rcc::SealedRccPeripheral {
    fn regs() -> crate::pac::dma2d::Dma2d;
}

/// DMA2D instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + Peripheral<P = Self> + crate::rcc::RccPeripheral + 'static + Send {
    /// Interrupt for this instance.
    type Interrupt: interrupt::typelevel::Interrupt;
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

    /// Initialise and enable the peripheral
    pub fn init(&mut self, config: Dma2dConfiguration) {
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
                LineOffsetMode::Bytes => Lom::BYTES,
                LineOffsetMode::Pixels => Lom::PIXELS,
            });
        });

        // configure output pixel format converter
        dma2d.opfccr().modify(|w| {
            w.set_cm(match config.color_mode {
                ColorMode::Argb8888 => OpfccrCm::ARGB8888,
                ColorMode::Rgb888 => OpfccrCm::RGB888,
                ColorMode::Rgb565 => OpfccrCm::RGB565,
                ColorMode::Argb1555 => OpfccrCm::ARGB1555,
                ColorMode::Argb4444 => OpfccrCm::ARGB4444,
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

    fn set_config<TPixelSize>(
        &self,
        src: SourceData,
        dst_addr: &'static [TPixelSize],
        width: u16,
        height: u16,
    ) -> Result<(), Error> {
        let dma2d = T::regs();

        let config = self.config.as_ref().expect("init has not yet been called");

        // set width and height
        dma2d.nlr().modify(|w| {
            w.set_pl(width);
            w.set_nl(height);
        });

        // set destination address
        dma2d.omar().modify(|w| w.set_ma(dst_addr.as_ptr() as u32));

        // set source addres
        match config.transfer_mode {
            TransferMode::RegisterToMemory => dma2d.ocolr().modify(|w| match src {
                SourceData::OutputColor(color) => {
                    let val = match color {
                        OutputColor::Argb1555(x) => x.0 as u32,
                        OutputColor::Argb4444(x) => x.0 as u32,
                        OutputColor::Argb8888(x) => x.0,
                        OutputColor::Rgb565(x) => x.0 as u32,
                        OutputColor::Rgb888(x) => x.0 as u32,
                    };
                    w.set_color(val);
                    Ok(())
                }
                _ => Err(Error::InvalidSourceData("expected output color")),
            }),
            TransferMode::MemoryToMemoryPfcBlendingFixedColorFg => match src {
                SourceData::SrcAddress(src_addr) => {
                    dma2d.bgmar().modify(|x| x.set_ma(src_addr.as_ptr() as u32));
                    Ok(())
                }
                _ => Err(Error::InvalidSourceData("expected src address for background buffer")),
            },
            TransferMode::MemoryToMemory
            | TransferMode::MemoryToMemoryPfc
            | TransferMode::MemoryToMemoryPfcBlending
            | TransferMode::MemoryToMemoryPfcBlendingFixedColorBg => match src {
                SourceData::SrcAddress(src_addr) => {
                    dma2d.fgmar().modify(|w| w.set_ma(src_addr.as_ptr() as u32));
                    Ok(())
                }
                _ => Err(Error::InvalidSourceData("expected src address for foreground buffer")),
            },
        }
    }

    /// Configure forground or background layers
    pub fn configure_layer(&self, config: LayerConfig) {
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

    /// Transfer blocking
    pub fn transfer_blocking<TPixelSize>(
        &self,
        src_data: SourceData,
        dest_addr: &'static [TPixelSize],
        width: u16,
        height: u16,
    ) -> Result<(), Error> {
        Self::clear_interrupt_flags(); // don't poison the request with old flags
        self.set_config(src_data, dest_addr, width, height)?;
        let dma2d = T::regs();
        dma2d.cr().modify(|w| w.set_start(CrStart::START));
        loop {
            let status = dma2d.isr().read();

            if status.caeif() {
                return Err(Error::ClutAccessError);
            } else if status.ceif() {
                return Err(Error::ConfigurationError);
            } else if status.ctcif() {
                return Err(Error::Unexpected("CLUT transfer complete"));
            } else if status.tcif() {
                return Ok(());
            } else if status.teif() {
                return Err(Error::TransferError);
            } else if status.twif() {
                return Err(Error::Unexpected("Transfer watermark"));
            }
        }
    }

    /// Blend
    pub async fn blend(
        &self,
        _scr_addr1: u32,
        _src_addr2: u32,
        _dst_addr: u32,
        _width: u16,
        _height: u16,
    ) -> Result<(), Error> {
        let _dma2d = T::regs();

        //  dma2d.fgcolr().modify(|w| w.)
        Ok(())
    }

    /// Start a dma2d transfer
    /// This is the async equivalent of calling start_transfer followed by repeatedly calling poll_transfer until completion
    /// The benefit of this function is that you don't have to hold up the cpu while waiting for completion
    pub async fn transfer(
        &self,
        src_data: SourceData,
        dst_addr: &'static [u8],
        width: u16,
        height: u16,
    ) -> Result<(), Error> {
        let mut status = T::regs().isr().read();
        self.set_config(src_data, dst_addr, width, height)?;

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
