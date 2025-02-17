//
// Created by wk on 2025/2/3.
//
#include "buff_controller/buff_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace buff_controller
{
bool BuffController::init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  if (!controller_nh.getParam("joint_name", joint_name_))
  {
    ROS_ERROR("Could not find joint_name parameter");
    return false;
  }
  controller_nh.getParam("feedForward", feedForward_);
  controller_nh.getParam("Kf", Kf_);

  joint_ = effort_joint_interface->getHandle(joint_name_);

  state_sub_ = controller_nh.subscribe("state", 1, &BuffController::update_current_state, this);
  target_vel_pub_ = controller_nh.advertise<std_msgs::Float64>("target_vel", 10);
  // load PID Controller using gains set on parameter server
  joint_pid_controller_.init(ros::NodeHandle(controller_nh, "joint_pid"));

  state_ = STOP;

  start_time_ = ros::Time::now();

  static dynamic_reconfigure::Server<buff_controller::paramsConfig> dr_server_(
      ros::NodeHandle(controller_nh, "feedForward"));

  cbType = boost::bind(&BuffController::param_callback, this, _1, _2);
  // 服务器对象调用回调对象
  dr_server_.setCallback(cbType);

  return true;
}

void BuffController::update(const ros::Time& time, const ros::Duration& period)
{
  current_time_ = time;
  period_ = period;

  update_target_vel();
  update_joint_state();

  compute_command();

  joint_.setCommand(commanded_effort_);
}

void BuffController::update_joint_state()
{
  current_effort_ = joint_.getEffort();
  current_velocity_ = joint_.getVelocity();
  current_position_ = joint_.getPosition();
}

void BuffController::update_target_vel()
{
  static double a{}, b{}, w{}, lower_bound{}, upper_bound{}, t{};

  if (state_changed_ && state_ == BIG)
  {
    boost::random::mt19937 gen(static_cast<unsigned int>(std::time(nullptr)));

    // 定义随机数分布范围
    lower_bound = 0.780;
    upper_bound = 1.045;
    boost::random::uniform_real_distribution<> dis(lower_bound, upper_bound);
    a = dis(gen);  // 生成一个随机小数

    lower_bound = 1.884;
    upper_bound = 2.000;
    dis.param(boost::random::uniform_real_distribution<>::param_type(lower_bound, upper_bound));

    w = dis(gen);  // 生成一个随机小数
    b = 2.090 - a;
    start_time_ = ros::Time::now();
    state_changed_ = false;

    ROS_INFO("a:%.3lf, b:%.3lf, w:%.3lf", a, b, w);
  }
  last_target_vel_ = target_vel_;
  if (state_ == SMALL)
  {
    target_vel_ = M_PI / 3.0f;
  }
  else if (state_ == BIG)
  {
    t = (current_time_ - start_time_).toSec();
    target_vel_ = a * sin(w * t) + b;
  }
  else if (state_ == STOP)
  {
    target_vel_ = 0;
  }
  std_msgs::Float64 target_vel_msg;
  target_vel_msg.data = target_vel_;
  target_vel_pub_.publish(target_vel_msg);
}

void BuffController::update_current_state(const std_msgs::Int32::ConstPtr& msg)
{
  if (msg->data != state_)
  {
    state_changed_ = true;
    ROS_INFO("state: %d", msg->data);
  }
  else
    state_changed_ = false;
  state_ = static_cast<BuffState>(msg->data);
}

void BuffController::compute_command()
{
  error_ = target_vel_ - current_velocity_;
  if (feedForward_)
  {
    error_target_ = target_vel_ - last_target_vel_;
    feedForward_out_ = error_target_ * Kf_;
  }
  else
  {
    feedForward_out_ = 0;
  }
  pid_out_ = joint_pid_controller_.computeCommand(error_, period_);
  commanded_effort_ = pid_out_ + feedForward_out_;
  //  ROS_INFO("Kf:%.2lf pid_out_:%.2lf feedForward_out_:%.2lf", Kf_, pid_out_, feedForward_out_);
}

PLUGINLIB_EXPORT_CLASS(buff_controller::BuffController, controller_interface::ControllerBase)
}  // namespace buff_controller
