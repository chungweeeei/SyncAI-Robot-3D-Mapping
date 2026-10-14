#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_mapping/pgo_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Named, not add_node(std::make_shared<PGONode>()): the executor keeps only a weak_ptr, so a
  // temporary would die at once and leave the process spinning over no nodes.
  auto node = std::make_shared<syncai_mapping::PGONode>();
  // Three threads: the default group serialises the timer, subscriptions and save_maps as
  // rclcpp::spin() did; the other two let start / reset_mapping block on the ResetLIO client
  // without deadlocking (handler on one, response on the other). Hence m_pgo_mutex.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
