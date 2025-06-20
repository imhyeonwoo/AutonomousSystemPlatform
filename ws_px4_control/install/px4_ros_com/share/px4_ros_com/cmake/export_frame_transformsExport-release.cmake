#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "px4_ros_com::frame_transforms" for configuration "Release"
set_property(TARGET px4_ros_com::frame_transforms APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(px4_ros_com::frame_transforms PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libframe_transforms.so"
  IMPORTED_SONAME_RELEASE "libframe_transforms.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS px4_ros_com::frame_transforms )
list(APPEND _IMPORT_CHECK_FILES_FOR_px4_ros_com::frame_transforms "${_IMPORT_PREFIX}/lib/libframe_transforms.so" )

# Import target "px4_ros_com::sensor_combined_listener" for configuration "Release"
set_property(TARGET px4_ros_com::sensor_combined_listener APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(px4_ros_com::sensor_combined_listener PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/px4_ros_com/sensor_combined_listener"
  )

list(APPEND _IMPORT_CHECK_TARGETS px4_ros_com::sensor_combined_listener )
list(APPEND _IMPORT_CHECK_FILES_FOR_px4_ros_com::sensor_combined_listener "${_IMPORT_PREFIX}/lib/px4_ros_com/sensor_combined_listener" )

# Import target "px4_ros_com::vehicle_gps_position_listener" for configuration "Release"
set_property(TARGET px4_ros_com::vehicle_gps_position_listener APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(px4_ros_com::vehicle_gps_position_listener PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/px4_ros_com/vehicle_gps_position_listener"
  )

list(APPEND _IMPORT_CHECK_TARGETS px4_ros_com::vehicle_gps_position_listener )
list(APPEND _IMPORT_CHECK_FILES_FOR_px4_ros_com::vehicle_gps_position_listener "${_IMPORT_PREFIX}/lib/px4_ros_com/vehicle_gps_position_listener" )

# Import target "px4_ros_com::debug_vect_advertiser" for configuration "Release"
set_property(TARGET px4_ros_com::debug_vect_advertiser APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(px4_ros_com::debug_vect_advertiser PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/px4_ros_com/debug_vect_advertiser"
  )

list(APPEND _IMPORT_CHECK_TARGETS px4_ros_com::debug_vect_advertiser )
list(APPEND _IMPORT_CHECK_FILES_FOR_px4_ros_com::debug_vect_advertiser "${_IMPORT_PREFIX}/lib/px4_ros_com/debug_vect_advertiser" )

# Import target "px4_ros_com::offboard_control" for configuration "Release"
set_property(TARGET px4_ros_com::offboard_control APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(px4_ros_com::offboard_control PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/px4_ros_com/offboard_control"
  )

list(APPEND _IMPORT_CHECK_TARGETS px4_ros_com::offboard_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_px4_ros_com::offboard_control "${_IMPORT_PREFIX}/lib/px4_ros_com/offboard_control" )

# Import target "px4_ros_com::offboard_control_srv" for configuration "Release"
set_property(TARGET px4_ros_com::offboard_control_srv APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(px4_ros_com::offboard_control_srv PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/px4_ros_com/offboard_control_srv"
  )

list(APPEND _IMPORT_CHECK_TARGETS px4_ros_com::offboard_control_srv )
list(APPEND _IMPORT_CHECK_FILES_FOR_px4_ros_com::offboard_control_srv "${_IMPORT_PREFIX}/lib/px4_ros_com/offboard_control_srv" )

# Import target "px4_ros_com::offboard_waypoint_map" for configuration "Release"
set_property(TARGET px4_ros_com::offboard_waypoint_map APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(px4_ros_com::offboard_waypoint_map PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/px4_ros_com/offboard_waypoint_map"
  )

list(APPEND _IMPORT_CHECK_TARGETS px4_ros_com::offboard_waypoint_map )
list(APPEND _IMPORT_CHECK_FILES_FOR_px4_ros_com::offboard_waypoint_map "${_IMPORT_PREFIX}/lib/px4_ros_com/offboard_waypoint_map" )

# Import target "px4_ros_com::offboard_waypoint_trigger" for configuration "Release"
set_property(TARGET px4_ros_com::offboard_waypoint_trigger APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(px4_ros_com::offboard_waypoint_trigger PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/px4_ros_com/offboard_waypoint_trigger"
  )

list(APPEND _IMPORT_CHECK_TARGETS px4_ros_com::offboard_waypoint_trigger )
list(APPEND _IMPORT_CHECK_FILES_FOR_px4_ros_com::offboard_waypoint_trigger "${_IMPORT_PREFIX}/lib/px4_ros_com/offboard_waypoint_trigger" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
