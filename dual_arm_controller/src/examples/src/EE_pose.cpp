#include <iostream>
#include <array>
#include <thread>
#include <mutex>
#include "franka/exception.h"
#include "franka/robot.h"
#include "franka/model.h"
#include "dual_arm_adaptive_cartesian/dual_arm_adaptive_cartesian_wrapper.hpp"
#include <Eigen/Dense>
#include "include/pseudo_inversion.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/JointState.h"
#include <cmath>
#include "examples_common.h"

#include <sstream>
#include <fstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "dual_arm_end_effector_tf_publisher");
    ros::NodeHandle nh;

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <left_robot_hostname> <right_robot_hostname>" << std::endl;
        return 1;
    }

    franka::Robot robot_left(argv[1]);  
    franka::Robot robot_right(argv[2]);  
    franka::Model model_left = robot_left.loadModel();
    franka::Model model_right = robot_right.loadModel();

    tf::TransformBroadcaster broadcaster_left;
    tf::TransformBroadcaster broadcaster_right;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
    
        franka::RobotState state_left = robot_left.readOnce();
        franka::RobotState state_right = robot_right.readOnce();

        Eigen::Map<const Eigen::Matrix<double, 4, 4>> ee_pose_left(state_left.O_T_EE.data());
        Eigen::Map<const Eigen::Matrix<double, 4, 4>> ee_pose_right(state_right.O_T_EE.data());

        Eigen::Matrix3d rotation_left = ee_pose_left.block<3, 3>(0, 0);
        Eigen::Vector3d translation_left(ee_pose_left(0, 3), ee_pose_left(1, 3), ee_pose_left(2, 3));

        Eigen::Matrix3d rotation_right = ee_pose_right.block<3, 3>(0, 0);
        Eigen::Vector3d translation_right(ee_pose_right(0, 3), ee_pose_right(1, 3), ee_pose_right(2, 3));

      

         Eigen::Vector3d position_difference = translation_right - translation_left;
    std::cout << "Position Difference (Right - Left):\n"
              << "  Delta X: " << position_difference.x() << "\n"
              << "  Delta Y: " << position_difference.y() << "\n"
              << "  Delta Z: " << position_difference.z() << "\n";
        Eigen::Quaterniond quaternion_left(rotation_left);
        Eigen::Quaterniond quaternion_right(rotation_right);

        tf::StampedTransform transform_left(
            tf::Transform(tf::Quaternion(quaternion_left.x(), quaternion_left.y(), quaternion_left.z(), quaternion_left.w()), tf::Vector3(translation_left.x(), translation_left.y(), translation_left.z())),
            
            ros::Time::now(),
            "panda_1_link0",
            "left_dmp_tf"
        );
        broadcaster_left.sendTransform(transform_left);

        tf::StampedTransform transform_right(
            tf::Transform(tf::Quaternion(quaternion_right.x(), quaternion_right.y(), quaternion_right.z(), quaternion_right.w()), tf::Vector3(translation_right.x(), translation_right.y(), translation_right.z())),
            
            ros::Time::now(),
            "panda_2_link0", 
            "right_dmp_tf"
        );
        broadcaster_right.sendTransform(transform_right);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
