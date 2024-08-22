#include "include/panda_kinematics.h"
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_example_node");
  ros::NodeHandle nh;

  Kinematics panda;
  PandaKinematics::setup(panda);

  tf::TransformListener listener;

  ros::Publisher leftJointStatePublisher = nh.advertise<sensor_msgs::JointState>("/panda_joint_states_left", 10);
  ros::Publisher rightJointStatePublisher = nh.advertise<sensor_msgs::JointState>("/panda_joint_states_right", 10);

  while (ros::ok())
  {
    tf::StampedTransform leftTf, rightTf;

    try
    {
      listener.waitForTransform("panda_1_link0", "LeftHandPn", ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("panda_1_link0", "LeftHandPn", ros::Time(0), leftTf);

      listener.waitForTransform("panda_2_link0", "RightHandPn", ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("panda_2_link0", "RightHandPn", ros::Time(0), rightTf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      continue;
    }

    // Extract position and angles (in angle-axis representation) from left hand tf
    tf::Vector3 leftPosition = leftTf.getOrigin();
    tf::Quaternion leftRotation = leftTf.getRotation();
    tf::Vector3 leftAngles = leftRotation.getAxis() * leftRotation.getAngle();

    // Extract position and angles (in angle-axis representation) from right hand tf
    tf::Vector3 rightPosition = rightTf.getOrigin();
    tf::Quaternion rightRotation = rightTf.getRotation();
    tf::Vector3 rightAngles = rightRotation.getAxis() * rightRotation.getAngle();

    Vec6d leftX;
    leftX << leftPosition.x(), leftPosition.y(), leftPosition.z(), leftAngles.x(), leftAngles.y(), leftAngles.z();

    Vec6d rightX;
    rightX << rightPosition.x(), rightPosition.y(), rightPosition.z(), rightAngles.x(), rightAngles.y(), rightAngles.z();

    VecXd leftQ(panda.xToQ(leftX, 0.0));
    VecXd rightQ(panda.xToQ(rightX, 0.0));

    sensor_msgs::JointState leftJointStateMsg;
    leftJointStateMsg.header.stamp = ros::Time::now();
    leftJointStateMsg.name = { "panda_1_joint1", "panda_1_joint2", "panda_1_joint3", "panda_1_joint4", "panda_1_joint5", "panda_1_joint6", "panda_1_joint7" };
    leftJointStateMsg.position = { leftQ[0], leftQ[1], leftQ[2], leftQ[3], leftQ[4], leftQ[5], leftQ[6] };

    leftJointStatePublisher.publish(leftJointStateMsg);

    sensor_msgs::JointState rightJointStateMsg;
    rightJointStateMsg.header.stamp = ros::Time::now();
    rightJointStateMsg.name = { "panda_2_joint1", "panda_2_joint2", "panda_2_joint3", "panda_2_joint4", "panda_2_joint5", "panda_2_joint6", "panda_2_joint7" };
    rightJointStateMsg.position = { rightQ[0], rightQ[1], rightQ[2], rightQ[3], rightQ[4], rightQ[5], rightQ[6] };

    rightJointStatePublisher.publish(rightJointStateMsg);

    ros::spinOnce();
  }

  return 0;
}
