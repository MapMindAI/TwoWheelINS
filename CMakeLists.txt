# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause

# Builds the C11 INS (fusion/ins_fusion.h) for C projects; Bazel builds everything else.
# - ESP-IDF: this directory is a component. Add it to EXTRA_COMPONENT_DIRS (or put it under
#   components/) and REQUIRES it by its directory name.
# - Plain CMake: the static library `ins_fusion`, via add_subdirectory or on its own.
set(INS_FUSION_SOURCES
  common/math/geodesy.c
  common/math/quaternion.c
  common/sensor/gps_accuracy.c
  fusion/ins_fusion.c
  fusion/kfcore_ins.c
  third_party/kfcore/kalman_udu.c
  third_party/kfcore/linalg.c
  third_party/kfcore/miniblas.c
)

if(ESP_PLATFORM)
  idf_component_register(SRCS ${INS_FUSION_SOURCES} INCLUDE_DIRS ".")
  return()
endif()

cmake_minimum_required(VERSION 3.13)
project(two_wheel_ins LANGUAGES C)

add_library(ins_fusion STATIC ${INS_FUSION_SOURCES})
target_include_directories(ins_fusion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(ins_fusion PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
  target_link_libraries(ins_fusion PUBLIC ${MATH_LIBRARY})
endif()
