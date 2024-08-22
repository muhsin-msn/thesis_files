#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cartesian_dual::cartesian_dual" for configuration "Release"
set_property(TARGET cartesian_dual::cartesian_dual APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cartesian_dual::cartesian_dual PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/cartesian_dual/libcartesian_dual.so"
  IMPORTED_SONAME_RELEASE "libcartesian_dual.so"
  )

list(APPEND _cmake_import_check_targets cartesian_dual::cartesian_dual )
list(APPEND _cmake_import_check_files_for_cartesian_dual::cartesian_dual "${_IMPORT_PREFIX}/cartesian_dual/libcartesian_dual.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
