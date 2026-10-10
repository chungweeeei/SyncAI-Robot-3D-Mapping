mod mapping_node;

use rclrs::*;

use mapping_node::MappingNode;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // rclrs inverts the rclcpp order: the executor comes first, and the node is created from it.
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("syncai_robot_3d_mapping")?;
    let _mapping = MappingNode::new(node)?;

    // No SIGINT handler: rclrs does not install one, so Ctrl-C ends the process without running
    // Drop. In scripts, stop a backgrounded node with SIGTERM.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
