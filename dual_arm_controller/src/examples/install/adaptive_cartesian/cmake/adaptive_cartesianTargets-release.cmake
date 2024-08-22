#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "adaptive_cartesian::adaptive_cartesian" for configuration "Release"
set_property(TARGET adaptive_cartesian::adaptive_cartesian APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(adaptive_cartesian::adaptive_cartesian PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/adaptive_cartesian/libadaptive_cartesian.so"
  IMPORTED_SONAME_RELEASE "libadaptive_cartesian.so"
  )

list(APPEND _cmake_import_check_targets adaptive_cartesian::adaptive_cartesian )
list(APPEND _cmake_import_check_files_for_adaptive_cartesian::adaptive_cartesian "${_IMPORT_PREFIX}/adaptive_cartesian/libadaptive_cartesian.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
