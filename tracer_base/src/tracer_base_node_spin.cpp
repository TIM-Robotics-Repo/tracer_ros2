/* Copyright (C) TIM Robotics Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by TIM Robotics <dev@tim-robotics.com>, May 2024
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tracer_base/tracer_base_ros.hpp"

int main(int argc, char **argv) {
  // setup ROS node
  rclcpp::init(argc, argv);
  auto robot = std::make_shared<westonrobot::TracerBaseRos>("tracer");
  rclcpp::spin(robot->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
