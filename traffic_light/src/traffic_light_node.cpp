// src/traffic_light_node.cpp

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "traffic_light/traffic_light_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node =
    std::make_shared<traffic_light::TrafficLightComponent>(
    options);

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions{},
    3);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
