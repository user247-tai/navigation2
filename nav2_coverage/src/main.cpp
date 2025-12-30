#include "rclcpp/rclcpp.hpp"
#include "nav2_coverage/coverage_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<nav2_coverage::CoverageServer>();

  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
