// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverSteeringSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_STEERING_SETPOINT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_STEERING_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoverSteeringSetpoint in the package px4_msgs.
typedef struct px4_msgs__msg__RoverSteeringSetpoint
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// [-1, 1] Normalized steering angle (Ackermann only, Positiv: Steer right, Negativ: Steer left)
  float normalized_steering_angle;
  /// [-1, 1] Normalized speed difference between the left and right wheels of the rover (Differential/Mecanum only, Positiv = Turn right, Negativ: Turn left)
  float normalized_speed_diff;
} px4_msgs__msg__RoverSteeringSetpoint;

// Struct for a sequence of px4_msgs__msg__RoverSteeringSetpoint.
typedef struct px4_msgs__msg__RoverSteeringSetpoint__Sequence
{
  px4_msgs__msg__RoverSteeringSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverSteeringSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_STEERING_SETPOINT__STRUCT_H_
