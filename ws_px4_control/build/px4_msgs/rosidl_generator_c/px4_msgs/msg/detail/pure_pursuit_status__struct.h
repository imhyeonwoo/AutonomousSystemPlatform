// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/PurePursuitStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__PURE_PURSUIT_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__PURE_PURSUIT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PurePursuitStatus in the package px4_msgs.
typedef struct px4_msgs__msg__PurePursuitStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Lookahead distance of pure the pursuit controller
  float lookahead_distance;
  /// Target bearing calculated by the pure pursuit controller
  float target_bearing;
  /// Shortest distance from the vehicle to the path (Positiv: Vehicle is on the right hand side with respect to the oriented path vector, Negativ: Left of the path)
  float crosstrack_error;
  /// Distance from the vehicle to the current waypoint
  float distance_to_waypoint;
  /// Bearing towards current waypoint
  float bearing_to_waypoint;
} px4_msgs__msg__PurePursuitStatus;

// Struct for a sequence of px4_msgs__msg__PurePursuitStatus.
typedef struct px4_msgs__msg__PurePursuitStatus__Sequence
{
  px4_msgs__msg__PurePursuitStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__PurePursuitStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__PURE_PURSUIT_STATUS__STRUCT_H_
