#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_transformation_example");
  ros::NodeHandle nh;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  ros::Rate rate(10.0);

  while (nh.ok())
  {
    tf::StampedTransform transform1;
    tf::StampedTransform transform2;

    try
    {
      // Lookup the "panda_2_link0" to "RightHandPn" transformation
      listener.lookupTransform("panda_2_link0", "RightHandPn", ros::Time(0), transform1);

      // Create a new transform from "world" to "RightHandPn_2"
      tf::StampedTransform newTransform1;
      newTransform1.child_frame_id_ = "RightHandPn_2";
      newTransform1.frame_id_ = "world";
      newTransform1.stamp_ = ros::Time::now();
      newTransform1.setOrigin(transform1.getOrigin());
      newTransform1.setRotation(transform1.getRotation());

      // Publish the new transformation for "RightHandPn_2"
      broadcaster.sendTransform(newTransform1);


      // Lookup the "panda_1_link0" to "LeftHandPn" transformation
      listener.lookupTransform("panda_1_link0", "LeftHandPn", ros::Time(0), transform2);

      // Create a new transform from "world" to "LeftHandPn_2"
      tf::StampedTransform newTransform2;
      newTransform2.child_frame_id_ = "LeftHandPn_2";
      newTransform2.frame_id_ = "world";
      newTransform2.stamp_ = ros::Time::now();
      newTransform2.setOrigin(transform2.getOrigin());
      newTransform2.setRotation(transform2.getRotation());

      // Publish the new transformation for "LeftHandPn_2"
      broadcaster.sendTransform(newTransform2);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    rate.sleep();
  }

  return 0;
}
