// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverVelocityStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoverVelocityStatus in the package px4_msgs.
typedef struct px4_msgs__msg__RoverVelocityStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Measured speed in body x direction. Positiv: forwards, Negativ: backwards
  float measured_speed_body_x;
  /// Post slew rate speed setpoint in body x direction. Positiv: forwards, Negativ: backwards
  float adjusted_speed_body_x_setpoint;
  /// Integral of the PID for the closed loop controller of the speed in body x direction
  float pid_throttle_body_x_integral;
  /// Measured speed in body y direction. Positiv: right, Negativ: left (Mecanum only)
  float measured_speed_body_y;
  /// Post slew rate speed setpoint in body y direction. Positiv: right, Negativ: left (Mecanum only)
  float adjusted_speed_body_y_setpoint;
  /// Integral of the PID for the closed loop controller of the speed in body y direction (Mecanum only)
  float pid_throttle_body_y_integral;
} px4_msgs__msg__RoverVelocityStatus;

// Struct for a sequence of px4_msgs__msg__RoverVelocityStatus.
typedef struct px4_msgs__msg__RoverVelocityStatus__Sequence
{
  px4_msgs__msg__RoverVelocityStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverVelocityStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__STRUCT_H_
