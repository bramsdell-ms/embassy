use nxp_pac::mrcc::vals::{ClkdivHalt, ClkdivReset, ClkdivUnstab, LpuartClkselMux};

use super::{Div4, PreEnableParts, SPConfHelper};
use crate::clocks::config::VddLevel;
use crate::clocks::{ClockError, Clocks, PoweredClock, WakeGuard};
use crate::{apply_div4, pac};

//
// LPUart
//

/// Selectable clocks for Lpuart peripherals
#[derive(Debug, Clone, Copy)]
pub enum LpuartClockSel {
    /// FRO12M/FRO_LF/SIRC clock source, passed through divider
    FroLfDiv,
    /// SOSC/XTAL/EXTAL clock source
    #[cfg(not(feature = "sosc-as-gpio"))]
    ClkIn,
    /// LP_OSC (low-power oscillator — RTC clock mux output, defaults to FRO16K)
    LpOsc,
    /// clk_1m/FRO_LF divided by 12
    Clk1M,
    /// Output of PLL1, passed through clock divider
    Pll1ClkDiv,
    /// Disabled
    None,
}

/// Which instance of the Lpuart is this?
///
/// Should not be directly selectable by end-users.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LpuartInstance {
    /// Instance 0
    Lpuart0,
    /// Instance 1
    Lpuart1,
    /// Instance 2
    Lpuart2,
    /// Instance 3
    Lpuart3,
    /// Instance 4
    Lpuart4,
    /// Instance 5
    Lpuart5,
}

/// Top level configuration for `Lpuart` instances.
pub struct LpuartConfig {
    /// Power state required for this peripheral
    pub power: PoweredClock,
    /// Clock source
    pub source: LpuartClockSel,
    /// Clock divisor
    pub div: Div4,
    /// Which instance is this?
    // NOTE: should not be user settable
    pub(crate) instance: LpuartInstance,
}

impl SPConfHelper for LpuartConfig {
    fn pre_enable_config(&self, clocks: &Clocks) -> Result<PreEnableParts, ClockError> {
        let mrcc0 = pac::MRCC0;

        let (clkdiv, clksel) = match self.instance {
            LpuartInstance::Lpuart0 => (mrcc0.mrcc_lpuart0_clkdiv(), mrcc0.mrcc_lpuart0_clksel()),
            LpuartInstance::Lpuart1 => (mrcc0.mrcc_lpuart1_clkdiv(), mrcc0.mrcc_lpuart1_clksel()),
            LpuartInstance::Lpuart2 => (mrcc0.mrcc_lpuart2_clkdiv(), mrcc0.mrcc_lpuart2_clksel()),
            LpuartInstance::Lpuart3 => (mrcc0.mrcc_lpuart3_clkdiv(), mrcc0.mrcc_lpuart3_clksel()),
            LpuartInstance::Lpuart4 => (mrcc0.mrcc_lpuart4_clkdiv(), mrcc0.mrcc_lpuart4_clksel()),
            LpuartInstance::Lpuart5 => (mrcc0.mrcc_lpuart5_clkdiv(), mrcc0.mrcc_lpuart5_clksel()),
        };

        let (freq, variant) = match self.source {
            LpuartClockSel::FroLfDiv => {
                let freq = clocks.ensure_fro_lf_div_active(&self.power)?;
                (freq, LpuartClkselMux::I0_CLKROOT_SIRC_DIV)
            }
            #[cfg(not(feature = "sosc-as-gpio"))]
            LpuartClockSel::ClkIn => {
                let freq = clocks.ensure_clk_in_active(&self.power)?;
                (freq, LpuartClkselMux::I3_CLKROOT_SOSC)
            }
            LpuartClockSel::LpOsc => {
                let freq = clocks.ensure_clk_16k_vdd_core_active(&self.power)?;
                (freq, LpuartClkselMux::I4_CLKROOT_LPOSC)
            }
            LpuartClockSel::Clk1M => {
                let freq = clocks.ensure_clk_1m_active(&self.power)?;
                (freq, LpuartClkselMux::I5_CLKROOT_1M)
            }
            LpuartClockSel::Pll1ClkDiv => {
                let freq = clocks.ensure_pll1_clk_div_active(&self.power)?;
                (freq, LpuartClkselMux::I6_CLKROOT_SPLL_DIV)
            }
            LpuartClockSel::None => {
                clksel.write(|w| w.set_mux(LpuartClkselMux::_RESERVED_7));
                clkdiv.modify(|w| {
                    w.set_reset(ClkdivReset::ON);
                    w.set_halt(ClkdivHalt::ON);
                });
                return Ok(PreEnableParts::empty());
            }
        };

        // Check clock speed is reasonable
        let div = self.div.into_divisor();
        let expected = freq / div;
        let power = match self.power {
            PoweredClock::NormalEnabledDeepSleepDisabled => clocks.active_power,
            PoweredClock::AlwaysEnabled => clocks.lp_power,
        };
        let fmax = match power {
            VddLevel::MidDriveMode => 45_000_000,
            VddLevel::NormalMode => 96_000_000,
            VddLevel::OverDriveMode => 180_000_000,
        };
        if expected > fmax {
            return Err(ClockError::BadConfig {
                clock: "lpuart fclk",
                reason: "exceeds max rating",
            });
        }

        // set clksel
        apply_div4!(self, clksel, clkdiv, variant, freq)
    }
}
