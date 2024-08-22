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
    tf::StampedTransform transform;

    try
    {
      // Lookup the "panda_2_link0" to "RightHandPn" transformation
      listener.lookupTransform("panda_2_link0", "RightHandPn", ros::Time(0), transform);

      // Create a new transform from "world" to "RightHandPn_2"
      tf::StampedTransform newTransform;
      newTransform.child_frame_id_ = "RightHandPn_2";
      newTransform.frame_id_ = "world";
      newTransform.stamp_ = ros::Time::now();
      newTransform.setOrigin(transform.getOrigin());
      newTransform.setRotation(transform.getRotation());

      // Publish the new transformation
      broadcaster.sendTransform(newTransform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    rate.sleep();
  }

  return 0;
}
