# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_stage_ros2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED stage_ros2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(stage_ros2_FOUND FALSE)
  elseif(NOT stage_ros2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(stage_ros2_FOUND FALSE)
  endif()
  return()
endif()
set(_stage_ros2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT stage_ros2_FIND_QUIETLY)
  message(STATUS "Found stage_ros2: 0.0.1 (${stage_ros2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'stage_ros2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${stage_ros2_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(stage_ros2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${stage_ros2_DIR}/${_extra}")
endforeach()
