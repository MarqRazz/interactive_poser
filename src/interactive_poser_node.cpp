// Copyright (c) 2026 Marq Rasmussen. BSD 3-Clause.

#include <interactive_poser/interactive_poser.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto poser = std::make_shared<interactive_poser::InteractivePoser>();

  // Multi-threaded with an explicit spin timeout: the prototype recorded a
  // deadlock spinning without one, and the note is cheaper to keep than to
  // rediscover.
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(poser->get_node_base_interface());
  exec.spin();
  exec.remove_node(poser->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
