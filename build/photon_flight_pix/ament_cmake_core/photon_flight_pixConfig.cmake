# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_photon_flight_pix_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED photon_flight_pix_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(photon_flight_pix_FOUND FALSE)
  elseif(NOT photon_flight_pix_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(photon_flight_pix_FOUND FALSE)
  endif()
  return()
endif()
set(_photon_flight_pix_CONFIG_INCLUDED TRUE)

# output package information
if(NOT photon_flight_pix_FIND_QUIETLY)
  message(STATUS "Found photon_flight_pix: 0.0.0 (${photon_flight_pix_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'photon_flight_pix' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT photon_flight_pix_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(photon_flight_pix_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${photon_flight_pix_DIR}/${_extra}")
endforeach()
