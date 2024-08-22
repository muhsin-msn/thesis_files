
cmake_minimum_required(VERSION 3.12)
project(motion_mapping)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  eigen_conversions
  message_generation
  tf
)



generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs eigen_conversions message_runtime tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add the common source files
set(COMMON_SOURCES
    bvh-parser.h
    bvh.h
    franka_ik_He.hpp
    joint.h
    utils.h
    bvh-parser_2.h
    geometry.h
    inv_kin.h
    kinematics.h
    panda_kinematics.h
    types.h
    
    inv_kin.cpp
    
    kinematics.cpp
    panda_kinematics.cpp

    


    
    

)

# List of executable targets
set(EXECUTABLE_TARGETS
   
    convert_joints_tcp
    convert_joints_file
    tcp_test_client
    pn_tf
    
    hands_tf
    hands_tf_vive
    
    pn_inv
    
   
    
    
    tf_origin
   
    hands_tf_vive_right
    hands_tf_vive_target
    hands_tf_vive_left

)

# Find Eigen package
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
#find_package(cppserver REQUIRED)

add_subdirectory(../../../CppServer cppserver)

# Include directories for Eigen and Boost
foreach(target ${EXECUTABLE_TARGETS})
    add_executable(${target} ${target}.cpp ${COMMON_SOURCES})
    target_include_directories(${target} PRIVATE ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/ros_lib)
    target_link_libraries(${target} PRIVATE ${catkin_LIBRARIES} Boost::filesystem Boost::system)
    #target_include_directories(${target} PRIVATE "../../../CppServer")
    target_include_directories(${target} PRIVATE "../../../CppServer/examples")
    target_link_libraries(${target} PRIVATE cppserver)  
endforeach()
