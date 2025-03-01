# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mobotrun_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mobotrun_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mobotrun_FOUND FALSE)
  elseif(NOT mobotrun_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mobotrun_FOUND FALSE)
  endif()
  return()
endif()
set(_mobotrun_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mobotrun_FIND_QUIETLY)
  message(STATUS "Found mobotrun: 0.0.0 (${mobotrun_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mobotrun' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mobotrun_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mobotrun_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mobotrun_DIR}/${_extra}")
endforeach()
