cmake_minimum_required(VERSION 3.10...3.20)
project(simulink_pipeline VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_THREAD_LIBS_INIT "-pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the list of executable names
set(executables
    joint_impedance_controller
    
    
    dual_arm_controller
    
    EE_pose
   
)

set(required_packages
    cntr_joint_imp
    
    cartesian_dual
   
    adaptive_cartesian
)


add_library(examples_common STATIC
  src/examples_common.cpp
)

# Set the corresponding package directories
foreach(package ${required_packages})
    set(${package}_DIR "${CMAKE_SOURCE_DIR}/examples/install/${package}/cmake")
endforeach()




# Find required packages
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(package ${required_packages})
    find_package(${package} REQUIRED)
endforeach()




# Find the required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

# Add the include directories for the ROS packages
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(/usr/include/eigen3)

add_service_files(
  FILES
  DMPLoad.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)


# Create and link executables
foreach(executable ${executables})
    add_executable(${executable} src/${executable}.cpp)
    
    # Link required_packages
    foreach(package ${required_packages})
        target_link_libraries(${executable} PRIVATE ${package}::${package})
    endforeach()
    
    # Link other packages
    target_link_libraries(${executable} PRIVATE Franka::Franka examples_common  ${catkin_LIBRARIES})
    
endforeach()


# Install executables
foreach(executable ${executables})
    install(TARGETS ${executable} DESTINATION ${CMAKE_SOURCE_DIR}/bin)
endforeach()

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

