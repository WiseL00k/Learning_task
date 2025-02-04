//
// Created by wk on 2025/2/3.
//
#pragma once

#ifndef SRC_BUFF_CONTROLLER_H
#define SRC_BUFF_CONTROLLER_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <boost/random.hpp>
#include <dynamic_reconfigure/server.h>
#include <buff_controller/paramsConfig.h>

namespace buff_controller
{
class BuffController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  BuffController() = default;
  ~BuffController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  hardware_interface::JointHandle joint_;

private:
  void update_target_vel();
  void update_joint_state();
  void update_current_state(const std_msgs::Int32::ConstPtr& msg);
  void param_callback(buff_controller::paramsConfig& config, uint32_t level)
  {
    Kf_ = config.Kf;
  }

  std::string joint_name_;
  ros::Time start_time_;
  ros::Time current_time_;
  ros::Duration period_;

  double target_vel_{};
  double last_target_vel_{};
  double current_position_{};
  double current_velocity_{};
  double current_effort_{};
  double error_{};
  double error_target_{};
  double pid_out_{};
  double Kf_{};
  double feedForward_out_{};
  double commanded_effort_{};
  bool feedForward_{};
  bool state_changed_{};

  typedef enum
  {
    BIG = 1,
    SMALL,
  } BuffState;
  int state_ = BIG;

  ros::Subscriber state_sub_;
  ros::Publisher target_vel_pub_;
  control_toolbox::Pid joint_pid_controller_;

  dynamic_reconfigure::Server<buff_controller::paramsConfig>::CallbackType cbType;

}; /* buff_controller */

}  // namespace buff_controller

#endif  // SRC_BUFF_CONTROLLER_H
