// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverThrottleSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_THROTTLE_SETPOINT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_THROTTLE_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoverThrottleSetpoint in the package px4_msgs.
typedef struct px4_msgs__msg__RoverThrottleSetpoint
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// throttle setpoint along body X axis [-1, 1] (Positiv = forwards, Negativ = backwards)
  float throttle_body_x;
  /// throttle setpoint along body Y axis [-1, 1] (Mecanum only, Positiv = right, Negativ = left)
  float throttle_body_y;
} px4_msgs__msg__RoverThrottleSetpoint;

// Struct for a sequence of px4_msgs__msg__RoverThrottleSetpoint.
typedef struct px4_msgs__msg__RoverThrottleSetpoint__Sequence
{
  px4_msgs__msg__RoverThrottleSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverThrottleSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_THROTTLE_SETPOINT__STRUCT_H_
