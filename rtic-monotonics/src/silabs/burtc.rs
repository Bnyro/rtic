//! [`Monotonic`](rtic_time::Monotonic) implementation for Silabs EFR32 and EFM32's BURTC ("Backup Real
//! Time Counter") peripheral.
//!
//! Always runs at a fixed rate of 32768 Hz, which is a resolution of 30.518 µs.
//!
//! # Example
//!
//! ```
//! use rtic_monotonics::silabs::burtc::prelude::*;
//!
//! // Create the type `Mono`. It will manage the BURTC peripheral,
//! // which is a 32768 Hz, 32-bit timer.
//! silabs_burtc_monotonic!(Mono);
//!
//! fn init() {
//!     # // This is normally provided by the selected PAC
//!     # let peripherals = unsafe { core::mem::transmute(()) };
//!     #
//!     // Start the monotonic - passing ownership of an silabs_metapac object for
//!     // BURTC, and temporary access of the clock management unit.
//!     Mono::start(peripherals.burtc_ns);
//! }
//!
//! async fn usage() {
//!     loop {
//!          // You can use the monotonic to get the time...
//!          let timestamp = Mono::now();
//!          // ...and you can use it to add a delay to this async function
//!          Mono::delay(100.millis()).await;
//!     }
//! }
//! ```

/// Common definitions and traits for using the silabs rtc monotonic
pub mod prelude {
    pub use crate::silabs_burtc_monotonic;
    pub use silabs_metapac;

    pub use crate::fugit::{self, ExtU64, ExtU64Ceil};
    pub use crate::Monotonic;
}

use crate::{rtic_time::timer_queue::TimerQueue, silabs::NVIC_PRIO_BITS, TimerQueueBackend};
use cortex_m::peripheral::NVIC;

pub use silabs_metapac::burtc_v0::Burtc;
pub use silabs_metapac::cmu_v1::Cmu;
use silabs_metapac::{Interrupt, BURTC};

/// Timer implementing [`TimerQueueBackend`].
pub struct TimerBackend;

impl TimerBackend {
    /// Starts the monotonic timer.
    ///
    /// **Do not use this function directly.**
    ///
    /// Use the prelude macros instead.
    pub fn _start(timer: Burtc, cmu: &Cmu) {
        // enable required bus clock
        cmu.clken0().modify(|w| w.set_burtc(true));

        // enable burtc
        timer.en().write(|w| w.set_en(true));

        // enable interrupts on value match
        timer.ien().modify(|w| w.set_comp(true));

        // start burtc
        timer.cmd().write(|w| w.set_start(true));
        // reset clock
        timer.cnt().write(|w| w.set_cnt(0));

        // HACK: busy wait until timer resetted
        while timer.cnt().read().cnt() != 0 {}

        TIMER_QUEUE.initialize(Self {});

        unsafe {
            crate::set_monotonic_prio(NVIC_PRIO_BITS, Interrupt::BURTC);
            NVIC::unmask(Interrupt::BURTC);
        }
    }

    fn burtc() -> &'static Burtc {
        &BURTC
    }
}

static TIMER_QUEUE: TimerQueue<TimerBackend> = TimerQueue::new();

impl TimerQueueBackend for TimerBackend {
    type Ticks = u32;

    fn now() -> Self::Ticks {
        let timer = Self::burtc();

        timer.cnt().read().cnt()
    }

    fn set_compare(instant: Self::Ticks) {
        Self::burtc().comp().write(|w| w.set_comp(instant));
    }

    fn clear_compare_flag() {
        // clear interrupt flag
        Self::burtc().if_clr().write(|w| w.set_comp(true));
    }

    fn pend_interrupt() {
        NVIC::pend(Interrupt::BURTC);
    }

    fn timer_queue() -> &'static TimerQueue<Self> {
        &TIMER_QUEUE
    }
}

/// Create a EFR32/EFM32 BURTC based monotonic and register the necessary interrupt for it.
///
/// See [`crate::silabs::burtc`] for more details.
///
/// # Arguments
///
/// * `name` - The name that the monotonic type will have.
#[macro_export]
macro_rules! silabs_burtc_monotonic {
    ($name:ident) => {
        use $crate::{fugit, rtic_time};

        /// A `Monotonic` based on the Silabs's BUTC peripheral.
        pub struct $name;

        impl $name {
            /// Starts the `Monotonic`.
            ///
            /// This method must be called only once.
            pub fn start(timer: $crate::silabs::burtc::Burtc, cmu: &$crate::silabs::burtc::Cmu) {
                #[no_mangle]
                #[allow(non_snake_case)]
                unsafe extern "C" fn BURTC() {
                    use $crate::TimerQueueBackend;
                    $crate::silabs::burtc::TimerBackend::timer_queue().on_monotonic_interrupt();
                }

                $crate::silabs::burtc::TimerBackend::_start(timer, cmu);
            }
        }

        impl $crate::TimerQueueBasedMonotonic for $name {
            type Backend = $crate::silabs::burtc::TimerBackend;
            type Instant =
                fugit::Instant<<Self::Backend as $crate::TimerQueueBackend>::Ticks, 1, 32768>;
            type Duration =
                fugit::Duration<<Self::Backend as $crate::TimerQueueBackend>::Ticks, 1, 32768>;
        }

        rtic_time::impl_embedded_hal_delay_fugit!($name);
        rtic_time::impl_embedded_hal_async_delay_fugit!($name);
    };
}
