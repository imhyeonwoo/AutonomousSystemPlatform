// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/RoverVelocityStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/rover_velocity_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_RoverVelocityStatus_pid_throttle_body_y_integral
{
public:
  explicit Init_RoverVelocityStatus_pid_throttle_body_y_integral(::px4_msgs::msg::RoverVelocityStatus & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::RoverVelocityStatus pid_throttle_body_y_integral(::px4_msgs::msg::RoverVelocityStatus::_pid_throttle_body_y_integral_type arg)
  {
    msg_.pid_throttle_body_y_integral = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::RoverVelocityStatus msg_;
};

class Init_RoverVelocityStatus_adjusted_speed_body_y_setpoint
{
public:
  explicit Init_RoverVelocityStatus_adjusted_speed_body_y_setpoint(::px4_msgs::msg::RoverVelocityStatus & msg)
  : msg_(msg)
  {}
  Init_RoverVelocityStatus_pid_throttle_body_y_integral adjusted_speed_body_y_setpoint(::px4_msgs::msg::RoverVelocityStatus::_adjusted_speed_body_y_setpoint_type arg)
  {
    msg_.adjusted_speed_body_y_setpoint = std::move(arg);
    return Init_RoverVelocityStatus_pid_throttle_body_y_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverVelocityStatus msg_;
};

class Init_RoverVelocityStatus_measured_speed_body_y
{
public:
  explicit Init_RoverVelocityStatus_measured_speed_body_y(::px4_msgs::msg::RoverVelocityStatus & msg)
  : msg_(msg)
  {}
  Init_RoverVelocityStatus_adjusted_speed_body_y_setpoint measured_speed_body_y(::px4_msgs::msg::RoverVelocityStatus::_measured_speed_body_y_type arg)
  {
    msg_.measured_speed_body_y = std::move(arg);
    return Init_RoverVelocityStatus_adjusted_speed_body_y_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverVelocityStatus msg_;
};

class Init_RoverVelocityStatus_pid_throttle_body_x_integral
{
public:
  explicit Init_RoverVelocityStatus_pid_throttle_body_x_integral(::px4_msgs::msg::RoverVelocityStatus & msg)
  : msg_(msg)
  {}
  Init_RoverVelocityStatus_measured_speed_body_y pid_throttle_body_x_integral(::px4_msgs::msg::RoverVelocityStatus::_pid_throttle_body_x_integral_type arg)
  {
    msg_.pid_throttle_body_x_integral = std::move(arg);
    return Init_RoverVelocityStatus_measured_speed_body_y(msg_);
  }

private:
  ::px4_msgs::msg::RoverVelocityStatus msg_;
};

class Init_RoverVelocityStatus_adjusted_speed_body_x_setpoint
{
public:
  explicit Init_RoverVelocityStatus_adjusted_speed_body_x_setpoint(::px4_msgs::msg::RoverVelocityStatus & msg)
  : msg_(msg)
  {}
  Init_RoverVelocityStatus_pid_throttle_body_x_integral adjusted_speed_body_x_setpoint(::px4_msgs::msg::RoverVelocityStatus::_adjusted_speed_body_x_setpoint_type arg)
  {
    msg_.adjusted_speed_body_x_setpoint = std::move(arg);
    return Init_RoverVelocityStatus_pid_throttle_body_x_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverVelocityStatus msg_;
};

class Init_RoverVelocityStatus_measured_speed_body_x
{
public:
  explicit Init_RoverVelocityStatus_measured_speed_body_x(::px4_msgs::msg::RoverVelocityStatus & msg)
  : msg_(msg)
  {}
  Init_RoverVelocityStatus_adjusted_speed_body_x_setpoint measured_speed_body_x(::px4_msgs::msg::RoverVelocityStatus::_measured_speed_body_x_type arg)
  {
    msg_.measured_speed_body_x = std::move(arg);
    return Init_RoverVelocityStatus_adjusted_speed_body_x_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverVelocityStatus msg_;
};

class Init_RoverVelocityStatus_timestamp
{
public:
  Init_RoverVelocityStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverVelocityStatus_measured_speed_body_x timestamp(::px4_msgs::msg::RoverVelocityStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverVelocityStatus_measured_speed_body_x(msg_);
  }

private:
  ::px4_msgs::msg::RoverVelocityStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::RoverVelocityStatus>()
{
  return px4_msgs::msg::builder::Init_RoverVelocityStatus_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__BUILDER_HPP_
