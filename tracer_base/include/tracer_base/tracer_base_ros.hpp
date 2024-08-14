/*
 * tracer_base_ros.hpp
 *
 * Created on: 3 2, 2022 16:38
 * Description:
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#ifndef TRACER_BASE_ROS_HPP
#define TRACER_BASE_ROS_HPP

#include <atomic>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tim_common_utils/lifecycle_node.h"

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"
#include "tracer_base/tracer_messenger.hpp"

namespace westonrobot {
class TracerBaseRos : public tim_common_utils::LifecycleNode {
 public:
  TracerBaseRos(std::string node_name);

  bool Initialize();
  void Run();
  void Stop();

 private:
  tim_common_utils::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
  tim_common_utils::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
  tim_common_utils::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
  tim_common_utils::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);

  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool is_tracer_mini_ = false;
  // bool is_omni_wheel_ = false;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;


  std::shared_ptr<TracerRobot> robot_;
  // std::shared_ptr<TracerMiniOmniRobot> omni_robot_;

  std::atomic<bool> keep_running_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::shared_ptr<TracerMessenger<TracerRobot>> messenger_;

  void LoadParameters();
};
}  // namespace westonrobot

#endif /* SCOUT_BASE_ROS_HPP */
