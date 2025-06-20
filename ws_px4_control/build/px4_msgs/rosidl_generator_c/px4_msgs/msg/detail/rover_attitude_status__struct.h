// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverAttitudeStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_ATTITUDE_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_ATTITUDE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoverAttitudeStatus in the package px4_msgs.
typedef struct px4_msgs__msg__RoverAttitudeStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Measured yaw rate
  float measured_yaw;
  /// Yaw setpoint that is being tracked (Applied slew rates)
  float adjusted_yaw_setpoint;
} px4_msgs__msg__RoverAttitudeStatus;

// Struct for a sequence of px4_msgs__msg__RoverAttitudeStatus.
typedef struct px4_msgs__msg__RoverAttitudeStatus__Sequence
{
  px4_msgs__msg__RoverAttitudeStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverAttitudeStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_ATTITUDE_STATUS__STRUCT_H_
