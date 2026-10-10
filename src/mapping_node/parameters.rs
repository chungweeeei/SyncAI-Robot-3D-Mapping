use rclrs::*;
use std::sync::Arc;
use std::time::Duration;

/// The range a rate parameter may turn into: `Duration::from_secs_f64` panics on a value that
/// overflows a `Duration`, and a zero period would make the timer fire as fast as it can.
const MIN_PERIOD: Duration = Duration::from_millis(1);
const MAX_PERIOD: Duration = Duration::from_secs(3600);

/// Hz -> timer period. A non-positive, NaN or infinite rate falls back to 1 Hz, and the result is
/// held inside [`MIN_PERIOD`]..=[`MAX_PERIOD`].
#[must_use]
pub fn period_from_rate(rate_hz: f64) -> Duration {
    let safe_rate = if rate_hz > 0.0 && rate_hz.is_finite() {
        rate_hz
    } else {
        1.0
    };

    // Clamped BEFORE the conversion, since from_secs_f64 panics on overflow
    let seconds = 1.0 / safe_rate;
    Duration::from_secs_f64(seconds.clamp(MIN_PERIOD.as_secs_f64(), MAX_PERIOD.as_secs_f64()))
}

/// The effective values the node runs on, after clamping.
pub struct Config {
    pub frame_id: String,
    pub heartbeat_period: Duration,
}

/// The node's parameters, all load-time only: declared `read_only()` so `ros2 param set` is
/// rejected instead of silently ignored. Values still come from the params file and launch
/// overrides, which are applied at declaration time.
///
/// The handles are kept for the node's lifetime because a parameter is undeclared when its handle
/// drops.
pub struct Parameters {
    _frame_id: ReadOnlyParameter<Arc<str>>,
    _heartbeat_rate: ReadOnlyParameter<f64>,
}

impl Parameters {
    pub fn declare(node: &Node) -> Result<(Self, Config), DeclarationError> {
        let frame_id = node
            .declare_parameter("frame_id")
            // Turbofished: nothing downstream pins the element type
            .default(Arc::<str>::from("map"))
            .description("frame the map is built in")
            .read_only()?;
        let heartbeat_rate = node
            .declare_parameter("heartbeat_rate")
            .default(1.0)
            .description("heartbeat publish rate, in Hz")
            .read_only()?;

        let config = Config {
            frame_id: frame_id.get().to_string(),
            heartbeat_period: period_from_rate(heartbeat_rate.get()),
        };

        log_info!(
            node.logger(),
            "[Parameters] frame_id={}, heartbeat_rate={} Hz",
            config.frame_id,
            heartbeat_rate.get(),
        );

        Ok((
            Self {
                _frame_id: frame_id,
                _heartbeat_rate: heartbeat_rate,
            },
            config,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rate_becomes_its_period() {
        assert_eq!(period_from_rate(1.0), Duration::from_secs(1));
        assert_eq!(period_from_rate(10.0), Duration::from_millis(100));
    }

    #[test]
    fn unusable_rates_fall_back_or_clamp_instead_of_panicking() {
        assert_eq!(period_from_rate(0.0), Duration::from_secs(1));
        assert_eq!(period_from_rate(-5.0), Duration::from_secs(1));
        assert_eq!(period_from_rate(f64::NAN), Duration::from_secs(1));
        assert_eq!(period_from_rate(f64::INFINITY), Duration::from_secs(1));
        assert_eq!(period_from_rate(5e-324), MAX_PERIOD);
        assert_eq!(period_from_rate(1e9), MIN_PERIOD);
    }
}
