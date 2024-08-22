# Set the version of cppserver
set(cppserver_VERSION 1.0)

# Define the include and library directories
set(cppserver_INCLUDE_DIRS "/home/ur10_pn/perc_neuron_ros_ur10/cpp_server/CppServer/include")
set(cppserver_LIBRARY_DIRS "/home/ur10_pn/perc_neuron_ros_ur10/cpp_server/CppServer/bin")

# Set the cppserver libraries
set(cppserver_LIBRARIES cppserver)

# Export the targets and variables
export(TARGETS cppserver FILE cppserverTargets.cmake)
export(PACKAGE cppserver)

# Create a package configuration file
configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cppserverConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cppserverConfig.cmake"
  INSTALL_DESTINATION "${cppserver_LIBRARY_DIRS}"
  PATH_VARS cppserver_INCLUDE_DIRS cppserver_LIBRARY_DIRS
)

# Install the package configuration file
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/cppserverConfig.cmake"
  DESTINATION "${cppserver_LIBRARY_DIRS}"
)
