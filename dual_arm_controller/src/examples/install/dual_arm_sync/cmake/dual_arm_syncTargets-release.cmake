#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dual_arm_sync::dual_arm_sync" for configuration "Release"
set_property(TARGET dual_arm_sync::dual_arm_sync APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(dual_arm_sync::dual_arm_sync PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/dual_arm_sync/libdual_arm_sync.so"
  IMPORTED_SONAME_RELEASE "libdual_arm_sync.so"
  )

list(APPEND _cmake_import_check_targets dual_arm_sync::dual_arm_sync )
list(APPEND _cmake_import_check_files_for_dual_arm_sync::dual_arm_sync "${_IMPORT_PREFIX}/dual_arm_sync/libdual_arm_sync.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
