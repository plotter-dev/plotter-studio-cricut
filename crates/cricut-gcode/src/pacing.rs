//! How fast vector frames are offered to the machine's FIFO.
//!
//! The machine's FIFO holds 34 slots, and a frame of `n` vectors takes `n + 2` of them, so two full
//! frames fit. Its ACK to a frame reports the slots still free after taking it (a full frame into
//! an empty FIFO reports 19, a 5-vector one 27); a frame that does not fit is dropped (NAK), which
//! costs only the exchange (30-45 ms) and tells us nothing about how full the FIFO is.
//!
//! [`Pacing::LEGACY`] is what opencutter documents and what the first hardware tests used: sleep
//! 100 ms for every slot short, however fast the machine really empties it. On a fill of 9,254
//! tiny vectors that kept the host waiting for 87 % of the job and left the FIFO empty (the
//! carriage stopped) before nearly half the frames. [`Pacing::quick`] tries again after about one
//! exchange instead. It is not the default until it has been verified on the machine.
//!
//! The engine and the async controller both take their numbers from here, so they pace alike.

use std::time::Duration;

use crate::job::VECTORS_PER_FRAME;

/// How long without any frame accepted before [`Pacing::quick`] gives up.
pub const STALL_LIMIT: Duration = Duration::from_secs(30);
/// Attempts at one frame before [`Pacing::LEGACY`] gives up.
const LEGACY_MAX_ATTEMPTS: u32 = 300;

/// What to do when the FIFO has no room for the next frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Wait {
    /// Sleep 100 ms per slot short before a frame, and the same (3 s at most) after a refusal.
    Legacy,
    /// Wait `poll` and try again, and learn nothing from a refusal.
    Retry {
        /// Time between attempts, about one exchange.
        poll: Duration,
    },
}

impl std::fmt::Display for Pacing {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.wait {
            Wait::Legacy => write!(f, "legacy (100 ms a slot)")?,
            Wait::Retry { poll } => write!(f, "retry every {} ms", poll.as_millis())?,
        }
        write!(f, ", {} vectors a frame", self.frame_entries)
    }
}

/// The pacing of vector frames: how many vectors a frame holds and how to wait for room.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pacing {
    frame_entries: usize,
    wait: Wait,
}

impl Default for Pacing {
    fn default() -> Self {
        Self::LEGACY
    }
}

impl Pacing {
    /// The pacing the first hardware tests used; see the module documentation.
    pub const LEGACY: Self = Self {
        frame_entries: VECTORS_PER_FRAME,
        wait: Wait::Legacy,
    };

    /// Full frames, retried every `poll` while the FIFO has no room.
    #[must_use]
    pub const fn quick(poll: Duration) -> Self {
        Self {
            frame_entries: VECTORS_PER_FRAME,
            wait: Wait::Retry { poll },
        }
    }

    /// The same pacing with `entries` vectors in a frame; `None` unless that is 1 to 13.
    #[must_use]
    pub fn with_frame_entries(self, entries: usize) -> Option<Self> {
        (1..=VECTORS_PER_FRAME).contains(&entries).then_some(Self {
            frame_entries: entries,
            ..self
        })
    }

    /// Vectors in a frame.
    #[must_use]
    pub fn frame_entries(self) -> usize {
        self.frame_entries
    }

    /// Whether a refusal says anything about how full the FIFO is. The real machine's NAK carries
    /// no free count (its fifth byte is padding), so only [`Pacing::LEGACY`], which always read it,
    /// does.
    #[must_use]
    pub fn reads_free_from_refusals(self) -> bool {
        matches!(self.wait, Wait::Legacy)
    }

    /// How long to wait before sending a frame that needs `need` slots when the last reply
    /// reported `free`.
    #[must_use]
    pub fn before_frame(self, free: Option<u8>, need: usize) -> Duration {
        let Some(free) = free.filter(|f| usize::from(*f) < need) else {
            return Duration::ZERO;
        };
        match self.wait {
            Wait::Legacy => {
                let slots = u64::try_from(need - usize::from(free)).unwrap_or(50);
                Duration::from_millis((slots * 100).min(5_000))
            }
            Wait::Retry { poll } => poll,
        }
    }

    /// How long to wait after the machine dropped a frame that needs `need` slots.
    #[must_use]
    pub fn after_refusal(self, free: Option<u8>, need: usize) -> Duration {
        match self.wait {
            Wait::Legacy => {
                let slots = u64::try_from(need.saturating_sub(usize::from(free.unwrap_or(0))))
                    .unwrap_or(50);
                Duration::from_millis((slots * 100).clamp(300, 3_000))
            }
            Wait::Retry { poll } => poll,
        }
    }

    /// Whether to stop trying: `attempts` refusals in a row, the last `since_progress` after a
    /// frame was last accepted.
    #[must_use]
    pub fn give_up(self, attempts: u32, since_progress: Duration) -> bool {
        match self.wait {
            Wait::Legacy => attempts >= LEGACY_MAX_ATTEMPTS,
            Wait::Retry { .. } => since_progress >= STALL_LIMIT,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: fn(u64) -> Duration = Duration::from_millis;

    // The legacy numbers are opencutter's and what the first hardware runs used: 100 ms for every
    // slot short of the frame's `n + 2`, a full frame needing 15.

    #[test]
    fn legacy_sleeps_a_hundred_milliseconds_per_slot_short() {
        let p = Pacing::LEGACY;
        assert_eq!(p.before_frame(Some(4), 15), MS(1_100));
        assert_eq!(p.before_frame(Some(0), 15), MS(1_500));
        assert_eq!(p.before_frame(Some(14), 15), MS(100));
        assert_eq!(p.before_frame(Some(15), 15), MS(0));
        assert_eq!(p.before_frame(Some(19), 15), MS(0));
        assert_eq!(p.before_frame(None, 15), MS(0));
        // Never more than 5 s.
        assert_eq!(p.before_frame(Some(0), 200), MS(5_000));
    }

    #[test]
    fn legacy_waits_between_300_ms_and_3_s_after_a_refusal() {
        let p = Pacing::LEGACY;
        assert_eq!(p.after_refusal(Some(4), 15), MS(1_100));
        assert_eq!(p.after_refusal(Some(14), 15), MS(300));
        assert_eq!(p.after_refusal(None, 15), MS(1_500));
        assert_eq!(p.after_refusal(Some(0), 100), MS(3_000));
    }

    #[test]
    fn legacy_gives_up_after_300_refusals_however_quickly() {
        let p = Pacing::LEGACY;
        assert!(!p.give_up(299, Duration::from_secs(10_000)));
        assert!(p.give_up(300, Duration::ZERO));
    }

    #[test]
    fn quick_pacing_polls_instead_of_sleeping_by_the_slot() {
        let p = Pacing::quick(MS(40));
        assert_eq!(p.before_frame(Some(4), 15), MS(40));
        assert_eq!(p.before_frame(Some(0), 15), MS(40));
        assert_eq!(p.before_frame(Some(15), 15), MS(0));
        assert_eq!(p.before_frame(None, 15), MS(0));
        assert_eq!(p.after_refusal(Some(4), 15), MS(40));
        assert_eq!(p.after_refusal(None, 15), MS(40));
    }

    #[test]
    fn quick_pacing_gives_up_after_thirty_seconds_without_progress() {
        let p = Pacing::quick(MS(40));
        assert!(!p.give_up(10_000, Duration::from_millis(29_999)));
        assert!(p.give_up(1, Duration::from_secs(30)));
    }

    #[test]
    fn only_legacy_reads_a_free_count_from_a_refusal() {
        assert!(Pacing::LEGACY.reads_free_from_refusals());
        assert!(!Pacing::quick(MS(40)).reads_free_from_refusals());
    }

    #[test]
    fn a_frame_holds_one_to_thirteen_vectors() {
        assert_eq!(Pacing::LEGACY.frame_entries(), 13);
        let six = Pacing::quick(MS(40)).with_frame_entries(6).unwrap();
        assert_eq!(six.frame_entries(), 6);
        assert_eq!(
            six.before_frame(Some(4), 8),
            MS(40),
            "the same pacing, smaller frames"
        );
        assert_eq!(Pacing::LEGACY.with_frame_entries(0), None);
        assert_eq!(Pacing::LEGACY.with_frame_entries(14), None);
    }

    #[test]
    fn a_pacing_describes_itself() {
        assert_eq!(
            Pacing::LEGACY.to_string(),
            "legacy (100 ms a slot), 13 vectors a frame"
        );
        let quick = Pacing::quick(MS(40)).with_frame_entries(6).unwrap();
        assert_eq!(quick.to_string(), "retry every 40 ms, 6 vectors a frame");
    }

    #[test]
    fn the_default_is_the_pacing_that_was_verified_on_hardware() {
        assert_eq!(Pacing::default(), Pacing::LEGACY);
    }
}
