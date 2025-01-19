# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_astro_odometry_tf_broadcaster_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED astro_odometry_tf_broadcaster_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(astro_odometry_tf_broadcaster_FOUND FALSE)
  elseif(NOT astro_odometry_tf_broadcaster_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(astro_odometry_tf_broadcaster_FOUND FALSE)
  endif()
  return()
endif()
set(_astro_odometry_tf_broadcaster_CONFIG_INCLUDED TRUE)

# output package information
if(NOT astro_odometry_tf_broadcaster_FIND_QUIETLY)
  message(STATUS "Found astro_odometry_tf_broadcaster: 0.0.0 (${astro_odometry_tf_broadcaster_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'astro_odometry_tf_broadcaster' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${astro_odometry_tf_broadcaster_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(astro_odometry_tf_broadcaster_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${astro_odometry_tf_broadcaster_DIR}/${_extra}")
endforeach()
