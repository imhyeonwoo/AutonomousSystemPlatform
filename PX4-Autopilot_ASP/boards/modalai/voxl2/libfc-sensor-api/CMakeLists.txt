cmake_minimum_required(VERSION 3.10.0)
project(libfc_sensor VERSION 1.0.3 DESCRIPTION "Flight Controller (FC) sensor library")

add_library(fc_sensor SHARED
            src/fc_sensor_stub.c)

include_directories(inc/)

set_target_properties(fc_sensor PROPERTIES PUBLIC_HEADER "inc/fc_sensor.h")


