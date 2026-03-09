//! MCXA5xx IRTC diagnostic tool.
//!
//! Investigates why all RTC register reads return 0xFFFE ("clock domains synchronizing").
//!
//! ## Confirmed findings
//! - FRO16K oscillator IS running (verified via SysTick CLK_16K mux: ~16384 Hz)
//! - VBAT FROCLKE=0x7 (all FRO16K outputs enabled)
//! - CTRL reset value = 0x0000 (CLK_SEL=0 = FRO16K, correct)
//! - RTC register offsets verified: CTRL=0x10, STATUS=0x12 (from PERI_RTC.h)
//! - Write-protect unlock + SWR does not clear the 0xFFFE condition
//!
//! ## Open question
//! - LP_OSC (RTC clock mux output = clk_16k[0] in VDD_SYS domain) liveness is untested.
//!   CTimer0-based test causes chip lockup (bus fault) despite clock gate + reset release.
//!   Need to verify CTimer0 base address and MRCC gate config for MCXA577, or use an
//!   alternative measurement path (FREQME, CLKOUT pin, etc.)
//!
//! ## Recovery
//! If the chip locks up: `probe-rs erase --chip MCXA577 --connect-under-reset` (J-Link)

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

fn set_mrcc_mux(syscon_base: u32, mrcc_base: u32, sel_offset: u32, div_offset: u32, value: u32) {
    let clkunlock = (syscon_base + 0xFC) as *mut u32;
    unsafe {
        let v = core::ptr::read_volatile(clkunlock);
        core::ptr::write_volatile(clkunlock, v & !1);
        core::ptr::write_volatile((mrcc_base + sel_offset) as *mut u32, value);
        // NXP SDK divider sequence: HALT -> RESET -> set DIV
        core::ptr::write_volatile((mrcc_base + div_offset) as *mut u32, 1 << 30); // HALT
        core::ptr::write_volatile((mrcc_base + div_offset) as *mut u32, 1 << 29); // RESET
        core::ptr::write_volatile((mrcc_base + div_offset) as *mut u32, 0); // DIV=0 (div-by-1)
        let v = core::ptr::read_volatile(clkunlock);
        core::ptr::write_volatile(clkunlock, v | 1);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let rtc_base = 0x400E_E000u32;
    let syscon_base = 0x4009_1000u32;
    let mrcc_base = 0x4009_1800u32;

    let _p = hal::init(hal::config::Config::default());

    defmt::info!("=== MCXA5xx IRTC Diagnostic ===");

    // === Part 1: Verify FRO16K via SysTick external clock mux ===
    // SysTick mux: 0=CPU_CLK, 1=CLK_1M, 2=CLK_16K
    // SYSTICK_CLKSEL=0x280, SYSTICK_CLKDIV=0x284 (from PERI_MRCC.h)
    defmt::info!("--- FRO16K verification via SysTick ---");

    let syst_csr = 0xE000_E010u32 as *mut u32;
    let syst_rvr = 0xE000_E014u32 as *mut u32;
    let syst_cvr = 0xE000_E018u32 as *mut u32;

    // Test CLK_1M first (sanity check that mux works)
    set_mrcc_mux(syscon_base, mrcc_base, 0x280, 0x284, 1); // CLK_1M
    unsafe {
        core::ptr::write_volatile(syst_csr, 0);
        core::ptr::write_volatile(syst_rvr, 0x00FF_FFFF);
        core::ptr::write_volatile(syst_cvr, 0);
        core::ptr::write_volatile(syst_csr, 0x1); // external clock, no interrupt
    }
    cortex_m::asm::delay(100);
    let c1 = unsafe { core::ptr::read_volatile(syst_cvr as *const u32) };
    cortex_m::asm::delay(4_800_000); // ~0.8s at 6 MHz
    let c2 = unsafe { core::ptr::read_volatile(syst_cvr as *const u32) };
    unsafe { core::ptr::write_volatile(syst_csr, 0) };
    let clk1m_delta = if c1 > c2 { c1 - c2 } else { c1 + (0x00FF_FFFF - c2) + 1 };
    defmt::info!("CLK_1M: {} counts (expect ~1M)", clk1m_delta);

    // Test CLK_16K
    set_mrcc_mux(syscon_base, mrcc_base, 0x280, 0x284, 2); // CLK_16K
    unsafe {
        core::ptr::write_volatile(syst_csr, 0);
        core::ptr::write_volatile(syst_rvr, 0x00FF_FFFF);
        core::ptr::write_volatile(syst_cvr, 0);
        core::ptr::write_volatile(syst_csr, 0x1);
    }
    cortex_m::asm::delay(100);
    let c1 = unsafe { core::ptr::read_volatile(syst_cvr as *const u32) };
    cortex_m::asm::delay(4_800_000);
    let c2 = unsafe { core::ptr::read_volatile(syst_cvr as *const u32) };
    unsafe { core::ptr::write_volatile(syst_csr, 0) };
    let clk16k_delta = if c1 > c2 { c1 - c2 } else { c1 + (0x00FF_FFFF - c2) + 1 };
    defmt::info!("CLK_16K: {} counts (expect ~16K)", clk16k_delta);

    if clk1m_delta != clk16k_delta && clk16k_delta > 10000 && clk16k_delta < 25000 {
        defmt::info!("FRO16K confirmed running via clk_16k[1] (core domain)");
    } else {
        defmt::info!("FRO16K status unclear");
    }

    // === Part 2: VBAT state ===
    let vbat = 0x4009_3000u32;
    let froclke = unsafe { core::ptr::read_volatile((vbat + 0x20) as *const u32) };
    let froctla = unsafe { core::ptr::read_volatile((vbat + 0x30) as *const u32) };
    let frolcka = unsafe { core::ptr::read_volatile((vbat + 0x38) as *const u32) };
    defmt::info!("FROCLKE={:#010x} FROCTLA={:#010x} FROLCKA={:#010x}", froclke, froctla, frolcka);

    // === Part 3: RTC register reads ===
    defmt::info!("--- RTC registers (all should be 0xFFFE if stuck) ---");
    let yearmon = unsafe { core::ptr::read_volatile(rtc_base as *const u16) };
    let days = unsafe { core::ptr::read_volatile((rtc_base + 0x02) as *const u16) };
    let ctrl = unsafe { core::ptr::read_volatile((rtc_base + 0x10) as *const u16) };
    let status = unsafe { core::ptr::read_volatile((rtc_base + 0x12) as *const u16) };
    defmt::info!("YEARMON={:#06x} DAYS={:#06x} CTRL={:#06x} STATUS={:#06x}", yearmon, days, ctrl, status);

    // === Part 4: Blind SWR attempt ===
    // Write-protect unlock sequence (8-bit writes to STATUS[7:0])
    let status_addr = (rtc_base + 0x12) as *mut u8;
    let ctrl_addr = (rtc_base + 0x10) as *mut u16;
    unsafe {
        core::ptr::write_volatile(status_addr, 0x00);
        core::ptr::write_volatile(status_addr, 0x40);
        core::ptr::write_volatile(status_addr, 0xC0);
        core::ptr::write_volatile(status_addr, 0x80);
        // Software reset
        core::ptr::write_volatile(ctrl_addr, 0x0100); // SWR=1
        cortex_m::asm::delay(1000);
        core::ptr::write_volatile(ctrl_addr, 0x0000); // SWR=0, CLK_SEL=0 (FRO16K)
    }
    defmt::info!("SWR done, waiting 1s...");
    cortex_m::asm::delay(6_000_000);
    let yearmon_after = unsafe { core::ptr::read_volatile(rtc_base as *const u16) };
    defmt::info!("YEARMON after SWR: {:#06x}", yearmon_after);

    defmt::info!("=== Done ===");
}
