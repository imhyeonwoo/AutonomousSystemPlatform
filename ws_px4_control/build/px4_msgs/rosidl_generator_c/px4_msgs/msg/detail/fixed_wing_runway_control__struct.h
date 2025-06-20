// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/FixedWingRunwayControl.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__FIXED_WING_RUNWAY_CONTROL__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__FIXED_WING_RUNWAY_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/FixedWingRunwayControl in the package px4_msgs.
/**
  * Auxiliary control fields for fixed-wing runway takeoff/landing
 */
typedef struct px4_msgs__msg__FixedWingRunwayControl
{
  /// Passes information from the FixedWingModeManager to the FixedWingAttitudeController
  /// [us] time since system start
  uint64_t timestamp;
  /// Flag that enables the wheel steering.
  bool wheel_steering_enabled;
  /// [norm] [@range -1, 1] [FRD] Manual wheel nudging, added to controller output. NAN is interpreted as 0.
  float wheel_steering_nudging_rate;
} px4_msgs__msg__FixedWingRunwayControl;

// Struct for a sequence of px4_msgs__msg__FixedWingRunwayControl.
typedef struct px4_msgs__msg__FixedWingRunwayControl__Sequence
{
  px4_msgs__msg__FixedWingRunwayControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__FixedWingRunwayControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__FIXED_WING_RUNWAY_CONTROL__STRUCT_H_
