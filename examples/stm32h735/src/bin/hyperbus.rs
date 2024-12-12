#![no_std]
#![no_main]
#![macro_use]

/// The following example demonstrates the use of the HyperBus ospi peripheral (aka HyperRAM) used on a stm32h735g-dk to access 16MB of the onboard psram
/// The ospi config has been tuned for the specific high speed internal oscillator set in the rcc_setup module. If you use the external oscillator
/// you will need to tweak the config or the communication will break down and you will see it as memory corruption.
/// The example below initialises a 16MB buffer (4 million u32 integers) to zero
/// It then tests that the sum of all the values is, in fact, zero
/// Then, it writes an incrementing integer to each value
/// After that each value is read and tested for its expected value
///
/// Troubleshooting:
/// The hyperbus is very sensitive to the ospi config passed in (especially the hyperbus latency config)
/// and you can easily cause your device to become unresponsive via ST-LINK even after a power cycle.
/// If this happens you can always toggle SW1 on the dk to SYS MEM and erase the flash using STM32CubeProgrammer to unbrick your dk. This uses the ROM bootloader instead of booting from flash.
/// Additionally, make sure the clock setup matches the ospi config you are using. Incorrect clock setup causes data corruption.
/// If you are using the hyperbus for a frame buffer along with the LTDC peripheral be aware that using DCACHE can result in corrupted data because dma transfers are not cache aware.
/// See disable_dcache_for_memory_region() for more details or just disable DCACHE altogether.
///
use core::mem::MaybeUninit;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{ospi, Peripherals};
use embassy_time::{Duration, Timer};

use {defmt_rtt as _, panic_probe as _};

const LEN: usize = 4 * 1024 * 1024;

// the data behind this symbol lives in SDRAM (see memory.x) and consumes all 16MB of the available memory (in this example)
#[link_section = ".sdram"]
pub static mut BIG_BUFFER: MaybeUninit<[u32; LEN]> = MaybeUninit::uninit();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = stm32h735g_hsi_init();
    info!("started");

    // enable instruction and data cache to show that this performance improvement works without cache corruption
    let mut core_peri = cortex_m::Peripherals::take().unwrap();
    core_peri.SCB.enable_icache();
    core_peri.SCB.enable_dcache(&mut core_peri.CPUID);

    // setup the hyperbus which allows read write access to the 16MB of ram in memory mapped mode
    // starting at .sdram section (see memory.x)
    let config = ospi::Config {
        memory_type: ospi::MemoryType::HyperBusMemory,
        fifo_threshold: ospi::FIFOThresholdLevel::_4Bytes,
        chip_select_high_time: ospi::ChipSelectHighTime::_4Cycle,
        device_size: ospi::MemorySize::_16MiB,
        clock_prescaler: 0x1,
        chip_select_boundary: 0x17,
        refresh: 0x190,
        delay_hold_quarter_cycle: true,
        delay_block_bypass: false,
        hyperbus_latency_config: ospi::HyperbusLatencyConfig {
            fixed_latency_mode: true,
            write_zero_latency: false,
            device_access_time_cycles: 6,
            read_recovery_time_cycles: 4,
        },
        ..Default::default()
    };
    let _hyperbus = ospi::Ospi::new_hyperbus(
        p.OCTOSPI2, p.PF4, p.PF0, p.PF1, p.PF2, p.PF3, p.PG0, p.PG1, p.PG10, p.PG11, p.PG12, p.PF12, config,
    )
    .unwrap();

    // Safety: this is the only place we get a mutable reference to BIG_BUFFER
    let buf = unsafe {
        // set memory to zero - this does not take any significant stack space (due to compiler tricks)
        BIG_BUFFER.write([0; LEN]);

        // get a mutable static reference to it
        BIG_BUFFER.assume_init_mut()
    };

    info!("running memory test");

    // make sure that the array is zeroed
    let sum: u32 = buf.iter().sum();
    assert_eq!(sum, 0);

    // increment each value in the array by one
    for (i, val) in buf.iter_mut().enumerate() {
        *val = i as _;
    }

    // test that each value was set correctly
    for (i, val) in buf.iter().enumerate() {
        assert_eq!(i, *val as _);
    }

    info!("memory test passed");

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Sets up clocks for the stm32h735g mcu using internal clocks
pub fn stm32h735g_hsi_init() -> Peripherals {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV32,
            mul: PllMul::MUL200,
            divp: Some(PllDiv::DIV1),
            divq: Some(PllDiv::DIV4),
            divr: Some(PllDiv::DIV2),
        });
        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV32,
            mul: PllMul::MUL200,
            divp: Some(PllDiv::DIV5),
            divq: Some(PllDiv::DIV2),
            divr: Some(PllDiv::DIV2),
        });
        config.rcc.pll3 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV32,
            mul: PllMul::MUL200,
            divp: Some(PllDiv::DIV1),
            divq: Some(PllDiv::DIV1),
            divr: Some(PllDiv::DIV42),
        });

        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV2;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.apb3_pre = APBPrescaler::DIV2;
        config.rcc.apb4_pre = APBPrescaler::DIV2;
        config.rcc.mux.adcsel = mux::Adcsel::PLL2_P;

        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.supply_config = SupplyConfig::DirectSMPS;
    }

    embassy_stm32::init(config)
}

pub fn disable_dcache_for_memory_region(
    mpu: &mut cortex_m::peripheral::MPU,
    scb: &mut cortex_m::peripheral::SCB,
    base_addr: u32,
    size: u32,
    region_number: u32, // Can be zero but don't use the same region for different memory areas!
) {
    // dcache needs to be on for the best performance when sending data to the lcd panel.
    // However, it causes coherrence issues with dma so we need to disable caching
    // for things like dma buffers
    // memory regions are defined in memory.x
    // see section B3.5.9 of Arm v7-M Architecture Reference Manual

    // must be a power of 2 (e.g. 16K)
    let size = size as i32;

    // Refer to ARM®v7-M Architecture Reference Manual ARM DDI 0403
    // Version E.b Section B3.5
    const MEMFAULTENA: u32 = 1 << 16;

    unsafe {
        // Make sure outstanding transfers are done
        cortex_m::asm::dmb();

        scb.shcsr.modify(|r| r & !MEMFAULTENA);

        // Disable the MPU and clear the control register
        mpu.ctrl.write(0);
    }

    const REGION_FULL_ACCESS: u32 = 0x03;
    const REGION_ENABLE: u32 = 0x01;
    const REGION_TEX: u32 = 0b001; // Outer and inner Non-cacheable

    assert_eq!(size & (size - 1), 0, "memory region size must be a power of 2");
    assert_eq!(size & 0x1F, 0, "memory region size must be 32 bytes or more");
    fn log2minus1(sz: u32) -> u32 {
        for i in 5..=31 {
            if sz == (1 << i) {
                return i - 1;
            }
        }
        panic!("Unknown memory region size!");
    }

    // Configure region to be Non-Cacheable
    unsafe {
        mpu.rnr.write(region_number);
        mpu.rbar.write(base_addr);

        let rasr_val = (REGION_FULL_ACCESS << 24) | (REGION_TEX << 19) | (log2minus1(size as u32) << 1) | REGION_ENABLE;

        mpu.rasr.write(rasr_val);
    }

    const MPU_ENABLE: u32 = 0x01;
    const MPU_DEFAULT_MMAP_FOR_PRIVILEGED: u32 = 0x04;

    // Enable
    unsafe {
        mpu.ctrl.modify(|r| r | MPU_DEFAULT_MMAP_FOR_PRIVILEGED | MPU_ENABLE);

        scb.shcsr.modify(|r| r | MEMFAULTENA);

        // Ensure MPU settings take effect
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}
