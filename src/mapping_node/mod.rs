mod parameters;

use std::error::Error;

use rclrs::*;
use ros_env::std_msgs::msg::String as StringMsg;

use parameters::Parameters;

/// A minimal starting point for the 3D mapping node: it declares its parameters and publishes a
/// `std_msgs/String` heartbeat on the relative topic `heartbeat` (so it lands on
/// `<namespace>/heartbeat`). The mapping pipeline itself will replace the heartbeat.
///
/// # Threading
///
/// The timer runs on ONE rclrs [`Worker`], whose payload owns the publisher and the counter, so
/// the tick needs no lock. Add further subscriptions / timers to the same worker unless a callback
/// blocks.
pub struct MappingNode {
    // Fields drop in declaration order: the timer stops firing before the worker it runs on.
    _heartbeat_timer: WorkerTimer<Heartbeat>,
    _worker: Worker<Heartbeat>,
    // Parameters are undeclared when their handles drop, so they are kept for the node's life
    _parameters: Parameters,
    _node: Node,
}

impl MappingNode {
    pub fn new(node: Node) -> Result<Self, Box<dyn Error>> {
        let (parameters, config) = Parameters::declare(&node)?;

        let publisher: Publisher<StringMsg> =
            node.create_publisher("heartbeat".keep_last(1).reliable())?;

        let worker = node.create_worker(Heartbeat {
            frame_id: config.frame_id,
            count: 0,
            publisher,
            logger: node.logger().clone(),
        });
        let heartbeat_timer = worker
            .create_timer_repeating(config.heartbeat_period, |heartbeat: &mut Heartbeat| {
                heartbeat.tick()
            })?;

        log_info!(
            node.logger(),
            "[MappingNode] 3D Mapping initialized successfully (heartbeat every {:?})",
            config.heartbeat_period,
        );

        Ok(Self {
            _heartbeat_timer: heartbeat_timer,
            _worker: worker,
            _parameters: parameters,
            _node: node,
        })
    }
}

/// The worker payload: everything the heartbeat tick touches.
struct Heartbeat {
    frame_id: String,
    count: u64,
    publisher: Publisher<StringMsg>,
    logger: Logger,
}

impl Heartbeat {
    fn tick(&mut self) {
        self.count += 1;
        let msg = StringMsg {
            data: heartbeat_text(&self.frame_id, self.count),
        };
        if let Err(err) = self.publisher.publish(msg) {
            log_error!(
                &self.logger,
                "[MappingNode] Failed to publish heartbeat: {err}"
            );
        }
    }
}

/// The heartbeat payload, kept pure so it is unit-testable without a ROS environment.
#[must_use]
fn heartbeat_text(frame_id: &str, count: u64) -> String {
    format!("syncai_robot_3d_mapping alive (frame_id={frame_id}, count={count})")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heartbeat_text_carries_frame_and_count() {
        assert_eq!(
            heartbeat_text("map", 3),
            "syncai_robot_3d_mapping alive (frame_id=map, count=3)"
        );
    }
}
