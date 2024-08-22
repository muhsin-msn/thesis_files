
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <fstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_publisher_node");
    ros::NodeHandle nh;

    tf::TransformBroadcaster tf_broadcaster;

    ros::Rate rate(1000);  // Publish at 10 Hz

    std::ifstream file("rotation_values.txt");
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open rotation_values.txt");
        return -1;
    }

    double roll, pitch, yaw;
    file >> roll >> pitch >> yaw;
    file.close();

    tf::TransformListener tf_listener;

    while (ros::ok())
    {
        
        ros::Time current_time = ros::Time::now();

     
        tf::Transform transform_right_hand;
        transform_right_hand.setIdentity();

        // Apply the transformation chain from "world" to "RightHand"
        std::vector<std::string> right_hand_frame_names = {
            "panda_2_link0","world", "right_controller_2"
        };
        tf::StampedTransform transform_chain_right_hand;
        try
{
    for (int i = 0; i < right_hand_frame_names.size() - 1; ++i)
    {
        tf_listener.waitForTransform(right_hand_frame_names[i], right_hand_frame_names[i + 1], current_time, ros::Duration(1.0));
        tf_listener.lookupTransform(right_hand_frame_names[i], right_hand_frame_names[i + 1], current_time, transform_chain_right_hand);
        transform_right_hand *= transform_chain_right_hand;
    }
}
catch (tf::TransformException& ex)
{
    ROS_ERROR("%s", ex.what());
    continue;
}


/* tf::Vector3 translation = transform_right_hand.getOrigin();
translation.setX(translation.getX() + 0.1);
transform_right_hand.setOrigin(translation); */

tf::Vector3 translation = transform_right_hand.getOrigin();
translation.setX(translation.getX() ); 
translation.setY(translation.getY() );  
translation.setZ(translation.getZ()); 
transform_right_hand.setOrigin(translation);

tf::Transform rotationRightHandToEE(
    tf::Quaternion(tf::Vector3(0, 1, 0), M_PI_2),
    tf::Vector3(0, 0, 0));

transform_right_hand = transform_right_hand.operator*(rotationRightHandToEE);

tf::Transform rotationRightHandToEE2(
    tf::Quaternion(tf::Vector3(0, 0, 1), M_PI),
    tf::Vector3(0, 0, 0));

transform_right_hand = transform_right_hand.operator*(rotationRightHandToEE2);


       /*   tf::Transform rotationRightHandToEE3(
        tf::Quaternion(tf::Vector3(0, 1, 0), -M_PI_2),
        tf::Vector3(0, 0, 0));

             transform_right_hand =
        transform_right_hand.operator*(rotationRightHandToEE3); */



        // Publish the transform from "world" to "RightHand"
        tf_broadcaster.sendTransform(tf::StampedTransform(transform_right_hand, current_time, "panda_2_link0", "RightHandPn"));

        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
