// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverPositionSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_POSITION_SETPOINT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_POSITION_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoverPositionSetpoint in the package px4_msgs.
typedef struct px4_msgs__msg__RoverPositionSetpoint
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// 2-dimensional position setpoint in NED frame
  float position_ned[2];
  /// (Optional) 2-dimensional start position in NED frame used to define the line that the rover will track to position_ned (Defaults to vehicle position)
  float start_ned[2];
  /// (Optional) Specify rover speed (Defaults to maximum speed)
  float cruising_speed;
  /// (Optional) Specify arrival speed (Defaults to zero)
  float arrival_speed;
  /// [-pi,pi] from North. Optional, NAN if not set. Mecanum only. (Defaults to vehicle yaw)
  float yaw;
} px4_msgs__msg__RoverPositionSetpoint;

// Struct for a sequence of px4_msgs__msg__RoverPositionSetpoint.
typedef struct px4_msgs__msg__RoverPositionSetpoint__Sequence
{
  px4_msgs__msg__RoverPositionSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverPositionSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_POSITION_SETPOINT__STRUCT_H_
