//! RTC DateTime driver.
//!
//! Supports both MCXA2xx (Unix timestamp-based) and MCXA5xx (BCD calendar-based)
//! RTC peripherals behind a unified API.
use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};
use maitake_sync::WaitCell;

use crate::clocks::{WakeGuard, with_clocks};
use crate::interrupt::typelevel::{Handler, Interrupt};
use crate::pac;
use crate::pac::rtc::vals::Swr;

/// RTC interrupt handler.
pub struct InterruptHandler<I: Instance> {
    _phantom: PhantomData<I>,
}

trait SealedInstance {
    fn info() -> &'static Info;

    const PERF_INT_INCR: fn();
    const PERF_INT_WAKE_INCR: fn();
}

/// Trait for RTC peripheral instances
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    type Interrupt: Interrupt;
}

struct Info {
    regs: pac::rtc::Rtc,
    wait_cell: WaitCell,
}

impl Info {
    #[inline(always)]
    fn regs(&self) -> pac::rtc::Rtc {
        self.regs
    }

    #[inline(always)]
    fn wait_cell(&self) -> &WaitCell {
        &self.wait_cell
    }
}

unsafe impl Sync for Info {}

/// Token for RTC0
pub type Rtc0 = crate::peripherals::RTC0;
impl SealedInstance for crate::peripherals::RTC0 {
    #[inline(always)]
    fn info() -> &'static Info {
        static INFO: Info = Info {
            regs: pac::RTC0,
            wait_cell: WaitCell::new(),
        };
        &INFO
    }

    const PERF_INT_INCR: fn() = crate::perf_counters::incr_interrupt_rtc0;
    const PERF_INT_WAKE_INCR: fn() = crate::perf_counters::incr_interrupt_rtc0_wake;
}

impl Instance for crate::peripherals::RTC0 {
    type Interrupt = crate::interrupt::typelevel::RTC;
}

/// Number of days in a standard year
const DAYS_IN_A_YEAR: u32 = 365;
/// Number of seconds in a day
const SECONDS_IN_A_DAY: u32 = 86400;
/// Number of seconds in an hour
const SECONDS_IN_A_HOUR: u32 = 3600;
/// Number of seconds in a minute
const SECONDS_IN_A_MINUTE: u32 = 60;
/// Unix epoch start year (MCXA2xx)
const YEAR_RANGE_START: u16 = 1970;
/// IRTC base year (MCXA5xx) — YROFST is a signed 8-bit offset from this
#[cfg(feature = "mcxa5xx")]
const IRTC_BASE_YEAR: i16 = 2112;

/// Date and time structure for RTC operations
#[derive(Debug, Clone, Copy)]
pub struct RtcDateTime {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}
#[cfg(feature = "mcxa2xx")]
use crate::pac::rtc::vals::{Tcr, Um};

#[cfg(feature = "mcxa2xx")]
#[derive(Copy, Clone)]
pub struct RtcConfig {
    #[allow(dead_code)]
    wakeup_select: bool,
    update_mode: Um,
    #[allow(dead_code)]
    supervisor_access: bool,
    compensation_interval: u8,
    compensation_time: Tcr,
}

#[cfg(feature = "mcxa2xx")]
/// RTC interrupt enable flags
#[derive(Copy, Clone)]
pub struct RtcInterruptEnable;
#[cfg(feature = "mcxa2xx")]
impl RtcInterruptEnable {
    pub const RTC_TIME_INVALID_INTERRUPT_ENABLE: u32 = 1 << 0;
    pub const RTC_TIME_OVERFLOW_INTERRUPT_ENABLE: u32 = 1 << 1;
    pub const RTC_ALARM_INTERRUPT_ENABLE: u32 = 1 << 2;
    pub const RTC_SECONDS_INTERRUPT_ENABLE: u32 = 1 << 4;
}

#[cfg(feature = "mcxa5xx")]
use crate::pac::rtc::vals::AlmMatch;

/// RTC configuration for MCXA5xx
#[cfg(feature = "mcxa5xx")]
#[derive(Copy, Clone)]
pub struct RtcConfig {
    /// Alarm match granularity (default: match up to year for exact alarm)
    pub alarm_match: AlmMatch,
}

/// Unlock IRTC write protection by writing the magic sequence to STATUS[7:0].
///
/// The IRTC requires 8-bit writes of 0x00, 0x40, 0xC0, 0x80 to the low byte
/// of the STATUS register before any register writes are accepted.
#[cfg(feature = "mcxa5xx")]
fn irtc_unlock_write_protect(regs: pac::rtc::Rtc) {
    let status_addr = regs.status().as_ptr() as *mut u8;
    unsafe {
        core::ptr::write_volatile(status_addr, 0x00);
        core::ptr::write_volatile(status_addr, 0x40);
        core::ptr::write_volatile(status_addr, 0xC0);
        core::ptr::write_volatile(status_addr, 0x80);
    }
}

/// Lock IRTC write protection.
#[cfg(feature = "mcxa5xx")]
fn irtc_lock_write_protect(regs: pac::rtc::Rtc) {
    let status_addr = regs.status().as_ptr() as *mut u8;
    unsafe {
        core::ptr::write_volatile(status_addr, 0x00);
    }
}

/// Convert a calendar year to the IRTC YROFST register value (signed 8-bit offset from 2112).
#[cfg(feature = "mcxa5xx")]
fn year_to_yrofst(year: u16) -> u8 {
    ((year as i16 - IRTC_BASE_YEAR) as i8) as u8
}

/// Convert the IRTC YROFST register value back to a calendar year.
#[cfg(feature = "mcxa5xx")]
fn yrofst_to_year(yrofst: u8) -> u16 {
    (IRTC_BASE_YEAR + (yrofst as i8) as i16) as u16
}

/// Converts a DateTime structure to Unix timestamp (seconds since 1970-01-01)
///
/// # Arguments
///
/// * `datetime` - The date and time to convert
///
/// # Returns
///
/// Unix timestamp as u32
///
/// # Note
///
/// This function handles leap years correctly.
pub fn convert_datetime_to_seconds(datetime: &RtcDateTime) -> u32 {
    let month_days: [u16; 13] = [0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];

    let mut seconds = (datetime.year as u32 - 1970) * DAYS_IN_A_YEAR;
    seconds += (datetime.year as u32 / 4) - (1970 / 4);
    seconds += month_days[datetime.month as usize] as u32;
    seconds += datetime.day as u32 - 1;

    if (datetime.year & 3 == 0) && (datetime.month <= 2) {
        seconds -= 1;
    }

    seconds = seconds * SECONDS_IN_A_DAY
        + (datetime.hour as u32 * SECONDS_IN_A_HOUR)
        + (datetime.minute as u32 * SECONDS_IN_A_MINUTE)
        + datetime.second as u32;

    seconds
}

/// Converts Unix timestamp to DateTime structure
///
/// # Arguments
///
/// * `seconds` - Unix timestamp (seconds since 1970-01-01)
///
/// # Returns
///
/// RtcDateTime structure with the converted date and time
///
/// # Note
///
/// This function handles leap years correctly.
pub fn convert_seconds_to_datetime(seconds: u32) -> RtcDateTime {
    let mut seconds_remaining = seconds;
    let mut days = seconds_remaining / SECONDS_IN_A_DAY + 1;
    seconds_remaining %= SECONDS_IN_A_DAY;

    let hour = (seconds_remaining / SECONDS_IN_A_HOUR) as u8;
    seconds_remaining %= SECONDS_IN_A_HOUR;
    let minute = (seconds_remaining / SECONDS_IN_A_MINUTE) as u8;
    let second = (seconds_remaining % SECONDS_IN_A_MINUTE) as u8;

    let mut year = YEAR_RANGE_START;
    let mut days_in_year = DAYS_IN_A_YEAR;

    while days > days_in_year {
        days -= days_in_year;
        year += 1;

        days_in_year = if year.is_multiple_of(4) {
            DAYS_IN_A_YEAR + 1
        } else {
            DAYS_IN_A_YEAR
        };
    }

    let days_per_month = [
        31,
        if (year.is_multiple_of(4) && !year.is_multiple_of(100)) || year.is_multiple_of(400) {
            29
        } else {
            28
        },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];

    let mut month = 1;
    for (m, month_days) in days_per_month.iter().enumerate() {
        let m = m + 1;
        if days <= *month_days as u32 {
            month = m;
            break;
        } else {
            days -= *month_days as u32;
        }
    }

    let day = days as u8;

    RtcDateTime {
        year,
        month: month as u8,
        day,
        hour,
        minute,
        second,
    }
}

/// Returns default RTC configuration
///
/// # Returns
///
/// RtcConfig with sensible default values:
/// - No wakeup selection
/// - Update mode 0 (immediate updates)
/// - No supervisor access restriction
/// - No compensation
#[cfg(feature = "mcxa2xx")]
pub fn get_default_config() -> RtcConfig {
    RtcConfig {
        wakeup_select: false,
        update_mode: Um::UM_0,
        supervisor_access: false,
        compensation_interval: 0,
        compensation_time: Tcr::TCR_0,
    }
}

/// Returns default RTC configuration
///
/// # Returns
///
/// RtcConfig with sensible default values (alarm match on full date+time)
#[cfg(feature = "mcxa5xx")]
pub fn get_default_config() -> RtcConfig {
    RtcConfig {
        alarm_match: AlmMatch::YEAR,
    }
}
/// Minimal RTC handle for a specific instance I (store the zero-sized token like embassy)
pub struct Rtc<'a> {
    _inst: core::marker::PhantomData<&'a mut ()>,
    info: &'static Info,
    _wg: Option<WakeGuard>,
    #[cfg(feature = "mcxa5xx")]
    alarm_match: AlmMatch,
}

// =============================================================================
// MCXA2xx implementation — Unix timestamp-based RTC
// =============================================================================
#[cfg(feature = "mcxa2xx")]
impl<'a> Rtc<'a> {
    /// Create a new instance of the real time clock.
    pub fn new<T: Instance>(
        _inst: Peri<'a, T>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'a,
        config: RtcConfig,
    ) -> Self {
        let info = T::info();

        // The RTC is NOT gated by the MRCC, but we DO need to make sure the 16k clock
        // on the vsys domain is active
        let clocks = with_clocks(|c| c.clk_16k_vsys.clone());
        let clk = match clocks {
            None => panic!("Clocks have not been initialized"),
            Some(None) => panic!("Clocks initialized, but clk_16k_vsys not active"),
            Some(Some(clk)) => clk,
        };

        // RTC reset
        info.regs().cr().modify(|w| w.set_swr(Swr::SWR_1));
        info.regs().cr().modify(|w| w.set_swr(Swr::SWR_0));
        info.regs().tsr().write(|w| w.0 = 1);

        info.regs().cr().modify(|w| w.set_um(config.update_mode));

        info.regs().tcr().modify(|w| {
            w.set_cir(config.compensation_interval);
            w.set_tcr(config.compensation_time);
        });

        // Enable RTC interrupt
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self {
            _inst: core::marker::PhantomData,
            info,
            _wg: WakeGuard::for_power(&clk.power),
        }
    }

    /// Set the current date and time
    ///
    /// # Arguments
    ///
    /// * `datetime` - The date and time to set
    ///
    /// # Note
    ///
    /// The datetime is converted to Unix timestamp and written to the time seconds register.
    pub fn set_datetime(&self, datetime: RtcDateTime) {
        let seconds = convert_datetime_to_seconds(&datetime);
        self.info.regs().tsr().write(|w| w.0 = seconds);
    }

    /// Get the current date and time
    ///
    /// # Returns
    ///
    /// Current date and time as RtcDateTime
    ///
    /// # Note
    ///
    /// Reads the current Unix timestamp from the time seconds register and converts it.
    pub fn get_datetime(&self) -> RtcDateTime {
        let seconds = self.info.regs().tsr().read().0;
        convert_seconds_to_datetime(seconds)
    }

    /// Set the alarm date and time
    ///
    /// # Arguments
    ///
    /// * `alarm` - The date and time when the alarm should trigger
    ///
    /// # Note
    ///
    /// This function:
    /// - Clears any existing alarm by writing 0 to the alarm register
    /// - Waits for the clear operation to complete
    /// - Sets the new alarm time
    /// - Waits for the write operation to complete
    /// - Uses timeouts to prevent infinite loops
    /// - Enables the alarm interrupt after setting
    pub fn set_alarm(&self, alarm: RtcDateTime) {
        let seconds = convert_datetime_to_seconds(&alarm);

        self.info.regs().tar().write(|w| w.0 = 0);
        let mut timeout = 10000;
        while self.info.regs().tar().read().0 != 0 && timeout > 0 {
            timeout -= 1;
        }

        self.info.regs().tar().write(|w| w.0 = seconds);

        let mut timeout = 10000;
        while self.info.regs().tar().read().0 != seconds && timeout > 0 {
            timeout -= 1;
        }

        self.set_interrupt(RtcInterruptEnable::RTC_ALARM_INTERRUPT_ENABLE);
    }

    /// Get the current alarm date and time
    ///
    /// # Returns
    ///
    /// Alarm date and time as RtcDateTime
    ///
    /// # Note
    ///
    /// Reads the alarm timestamp from the time alarm register and converts it.
    pub fn get_alarm(&self) -> RtcDateTime {
        let alarm_seconds = self.info.regs().tar().read().0;
        convert_seconds_to_datetime(alarm_seconds)
    }

    /// Start the RTC time counter
    ///
    /// # Note
    ///
    /// Sets the Time Counter Enable (TCE) bit in the status register.
    pub fn start(&self) {
        self.info.regs().sr().modify(|w| w.set_tce(true));
    }

    /// Stop the RTC time counter
    ///
    /// # Note
    ///
    /// Clears the Time Counter Enable (TCE) bit in the status register.
    pub fn stop(&self) {
        self.info.regs().sr().modify(|w| w.set_tce(false));
    }

    /// Enable specific RTC interrupts
    ///
    /// # Arguments
    ///
    /// * `mask` - Bitmask of interrupts to enable (use RtcInterruptEnable constants)
    ///
    /// # Note
    ///
    /// This function enables the specified interrupt types and resets the alarm occurred flag.
    /// Available interrupts:
    /// - Time Invalid Interrupt
    /// - Time Overflow Interrupt  
    /// - Alarm Interrupt
    /// - Seconds Interrupt
    pub fn set_interrupt(&self, mask: u32) {
        if (RtcInterruptEnable::RTC_TIME_INVALID_INTERRUPT_ENABLE & mask) != 0 {
            self.info.regs().ier().modify(|w| w.set_tiie(true));
        }
        if (RtcInterruptEnable::RTC_TIME_OVERFLOW_INTERRUPT_ENABLE & mask) != 0 {
            self.info.regs().ier().modify(|w| w.set_toie(true));
        }
        if (RtcInterruptEnable::RTC_ALARM_INTERRUPT_ENABLE & mask) != 0 {
            self.info.regs().ier().modify(|w| w.set_taie(true));
        }
        if (RtcInterruptEnable::RTC_SECONDS_INTERRUPT_ENABLE & mask) != 0 {
            self.info.regs().ier().modify(|w| w.set_tsie(true));
        }
    }

    /// Disable specific RTC interrupts
    ///
    /// # Arguments
    ///
    /// * `mask` - Bitmask of interrupts to disable (use RtcInterruptEnable constants)
    ///
    /// # Note
    ///
    /// This function disables the specified interrupt types.
    pub fn disable_interrupt(&self, mask: u32) {
        if (RtcInterruptEnable::RTC_TIME_INVALID_INTERRUPT_ENABLE & mask) != 0 {
            self.info.regs().ier().modify(|w| w.set_tiie(false));
        }
        if (RtcInterruptEnable::RTC_TIME_OVERFLOW_INTERRUPT_ENABLE & mask) != 0 {
            self.info.regs().ier().modify(|w| w.set_toie(false));
        }
        if (RtcInterruptEnable::RTC_ALARM_INTERRUPT_ENABLE & mask) != 0 {
            self.info.regs().ier().modify(|w| w.set_taie(false));
        }
        if (RtcInterruptEnable::RTC_SECONDS_INTERRUPT_ENABLE & mask) != 0 {
            self.info.regs().ier().modify(|w| w.set_tsie(false));
        }
    }

    /// Clear the alarm interrupt flag
    ///
    /// # Note
    ///
    /// This function clears the Time Alarm Interrupt Enable bit.
    pub fn clear_alarm_flag(&self) {
        self.info.regs().ier().modify(|w| w.set_taie(false));
    }

    /// Wait for an RTC alarm to trigger.
    ///
    /// # Arguments
    ///
    /// * `alarm` - The date and time when the alarm should trigger
    ///
    /// This function will wait until the RTC alarm is triggered.
    /// If no alarm is scheduled, it will wait indefinitely until one is scheduled and triggered.
    pub async fn wait_for_alarm(&mut self, alarm: RtcDateTime) {
        let wait = self.info.wait_cell().subscribe().await;

        self.set_alarm(alarm);
        self.start();

        // REVISIT: propagate error?
        let _ = wait.await;

        // Clear the interrupt and disable the alarm after waking up
        self.disable_interrupt(RtcInterruptEnable::RTC_ALARM_INTERRUPT_ENABLE);
    }
}

/// RTC interrupt handler
///
/// This struct implements the interrupt handler for RTC events.
#[cfg(feature = "mcxa2xx")]
impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::PERF_INT_INCR();
        let rtc = pac::RTC0;
        // Check if this is actually a time alarm interrupt
        let sr = rtc.sr().read();
        if sr.taf() {
            rtc.ier().modify(|w| w.set_taie(false));
            T::PERF_INT_WAKE_INCR();
            T::info().wait_cell().wake();
        }
    }
}

// =============================================================================
// MCXA5xx implementation — BCD calendar-based RTC
// =============================================================================
#[cfg(feature = "mcxa5xx")]
impl<'a> Rtc<'a> {
    /// Create a new instance of the real time clock.
    pub fn new<T: Instance>(
        _inst: Peri<'a, T>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'a,
        config: RtcConfig,
    ) -> Self {
        let info = T::info();

        let clocks = with_clocks(|c| c.clk_16k_vsys.clone());
        let clk = match clocks {
            None => panic!("Clocks have not been initialized"),
            Some(None) => panic!("Clocks initialized, but clk_16k_vsys not active"),
            Some(Some(clk)) => clk,
        };

        // Unlock write protection before any register writes
        irtc_unlock_write_protect(info.regs());

        // RTC software reset (note: YROFST is unaffected by SWR per ref manual)
        info.regs().ctrl().modify(|w| w.set_swr(Swr::ASSERTED));
        info.regs().ctrl().modify(|w| w.set_swr(Swr::CLEARED));

        // Configure alarm match mode
        info.regs().ctrl().modify(|w| {
            w.set_alm_match(config.alarm_match);
        });

        // Initialize to a valid date (2025-01-01 00:00:00)
        info.regs().yearmon().write(|w| {
            w.set_yrofst(year_to_yrofst(2025));
            w.set_mon_cnt(crate::pac::rtc::vals::MonCnt::JANUARY);
        });
        info.regs().days().write(|w| {
            w.set_day_cnt(1);
        });
        info.regs().hourmin().write(|w| {
            w.set_hour_cnt(0);
            w.set_min_cnt(0);
        });
        info.regs().seconds().write(|w| {
            w.set_sec_cnt(0);
        });

        // Re-lock write protection
        irtc_lock_write_protect(info.regs());

        // Enable RTC interrupt
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self {
            _inst: core::marker::PhantomData,
            info,
            _wg: WakeGuard::for_power(&clk.power),
            alarm_match: config.alarm_match,
        }
    }

    /// Set the current date and time
    ///
    /// Writes directly to the BCD calendar registers.
    pub fn set_datetime(&self, datetime: RtcDateTime) {
        let r = self.info.regs();

        irtc_unlock_write_protect(r);

        r.yearmon().write(|w| {
            w.set_yrofst(year_to_yrofst(datetime.year));
            w.set_mon_cnt(crate::pac::rtc::vals::MonCnt::from_bits(datetime.month));
        });
        r.days().write(|w| {
            w.set_day_cnt(datetime.day);
        });
        r.hourmin().write(|w| {
            w.set_hour_cnt(datetime.hour);
            w.set_min_cnt(datetime.minute);
        });
        r.seconds().write(|w| {
            w.set_sec_cnt(datetime.second);
        });

        irtc_lock_write_protect(r);
    }

    /// Get the current date and time
    ///
    /// Reads from the BCD calendar registers.
    pub fn get_datetime(&self) -> RtcDateTime {
        let r = self.info.regs();

        let ym = r.yearmon().read();
        let d = r.days().read();
        let hm = r.hourmin().read();
        let s = r.seconds().read();

        RtcDateTime {
            year: yrofst_to_year(ym.yrofst()),
            month: ym.mon_cnt().to_bits(),
            day: d.day_cnt(),
            hour: hm.hour_cnt(),
            minute: hm.min_cnt(),
            second: s.sec_cnt(),
        }
    }

    /// Set the alarm date and time
    ///
    /// Writes to the alarm calendar registers and enables the alarm interrupt.
    pub fn set_alarm(&self, alarm: RtcDateTime) {
        let r = self.info.regs();

        irtc_unlock_write_protect(r);

        // Clear any pending alarm interrupt (W1C)
        r.isr().modify(|w| w.set_alm_is(true));

        r.alm_yearmon().write(|w| {
            w.set_alm_year(year_to_yrofst(alarm.year));
            w.set_alm_mon(alarm.month);
        });
        r.alm_days().write(|w| {
            w.set_alm_day(alarm.day);
        });
        r.alm_hourmin().write(|w| {
            w.set_alm_hour(alarm.hour);
            w.set_alm_min(alarm.minute);
        });
        r.alm_seconds().write(|w| {
            w.set_alm_sec(alarm.second);
        });

        r.ctrl().modify(|w| w.set_alm_match(self.alarm_match));
        r.ier().modify(|w| w.set_alm_ie(true));

        irtc_lock_write_protect(r);
    }

    /// Get the current alarm date and time
    pub fn get_alarm(&self) -> RtcDateTime {
        let r = self.info.regs();

        let ym = r.alm_yearmon().read();
        let d = r.alm_days().read();
        let hm = r.alm_hourmin().read();
        let s = r.alm_seconds().read();

        RtcDateTime {
            year: yrofst_to_year(ym.alm_year()),
            month: ym.alm_mon(),
            day: d.alm_day(),
            hour: hm.alm_hour(),
            minute: hm.alm_min(),
            second: s.alm_sec(),
        }
    }

    /// Start the RTC time counter
    ///
    /// The MCXA5xx RTC runs automatically after reset — this is provided for API compatibility.
    pub fn start(&self) {
        // MCXA5xx RTC has no explicit counter enable bit — it runs once SWR is deasserted.
    }

    /// Stop the RTC time counter
    ///
    /// The MCXA5xx RTC cannot be paused without a destructive software reset that clears
    /// all registers. Calendar registers can be written while the RTC is running, so this
    /// is a no-op for API compatibility.
    pub fn stop(&self) {
        // No-op: SWR would reset all register state, which is not the intended behavior.
    }

    /// Clear the alarm interrupt flag
    pub fn clear_alarm_flag(&self) {
        let r = self.info.regs();
        irtc_unlock_write_protect(r);
        // W1C: write 1 to clear
        r.isr().modify(|w| w.set_alm_is(true));
        r.ier().modify(|w| w.set_alm_ie(false));
        irtc_lock_write_protect(r);
    }

    /// Wait for an RTC alarm to trigger.
    ///
    /// # Arguments
    ///
    /// * `alarm` - The date and time when the alarm should trigger
    ///
    /// This function will wait until the RTC alarm is triggered.
    pub async fn wait_for_alarm(&mut self, alarm: RtcDateTime) {
        let wait = self.info.wait_cell().subscribe().await;

        self.set_alarm(alarm);

        // REVISIT: propagate error?
        let _ = wait.await;

        // Clear and disable the alarm after waking up
        self.clear_alarm_flag();
    }
}

/// MCXA5xx RTC interrupt handler — checks alarm interrupt status in ISR
#[cfg(feature = "mcxa5xx")]
impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::PERF_INT_INCR();
        let rtc = pac::RTC0;
        let isr = rtc.isr().read();
        if isr.alm_is() {
            // W1C: write 1 to clear the alarm status
            rtc.isr().modify(|w| w.set_alm_is(true));
            // Disable alarm interrupt to prevent re-entry
            rtc.ier().modify(|w| w.set_alm_ie(false));
            T::PERF_INT_WAKE_INCR();
            T::info().wait_cell().wake();
        }
    }
}
