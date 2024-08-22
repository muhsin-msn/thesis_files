
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <fstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_publisher_node");
    ros::NodeHandle nh;

    tf::TransformBroadcaster tf_broadcaster;

    ros::Rate rate(1000);  

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
translation.setX(translation.getX()); 
translation.setY(translation.getY());  
//translation.setZ(translation.getZ() - 0.2);  // Decrease z by 0.2
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



        
        tf_broadcaster.sendTransform(tf::StampedTransform(transform_right_hand, current_time, "panda_2_link0", "RightHandPn"));

        
        tf::Transform transform_left_hand;
        transform_left_hand.setIdentity();

        // Apply the transformation chain from "world" to "LeftHand"
        std::vector<std::string> left_hand_frame_names = {
            "panda_1_link0","world", "target_left"
        };
        tf::StampedTransform transform_chain_left_hand;
        try
        {
            for (int i = 0; i < left_hand_frame_names.size() - 1; ++i)
            {
                tf_listener.waitForTransform(left_hand_frame_names[i], left_hand_frame_names[i + 1], current_time, ros::Duration(1.0));
                tf_listener.lookupTransform(left_hand_frame_names[i], left_hand_frame_names[i + 1], current_time, transform_chain_left_hand);
                transform_left_hand *= transform_chain_left_hand;
            }
        }
         catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            continue;
        }

        tf::Vector3 translation2 = transform_left_hand.getOrigin();
        translation2.setX(translation2.getX() );  // Increase x by 0.1
        translation2.setY(translation2.getY()-0.02);  // Decrease y by 0.1
        //translation.setZ(translation.getZ() - 0.2);  // Decrease z by 0.2
        transform_left_hand.setOrigin(translation2);

        tf::Transform rotationLeftHandToEE(
        tf::Quaternion(tf::Vector3(0, 1, 0), M_PI_2),
        tf::Vector3(0, 0, 0));

     
     transform_left_hand =
        transform_left_hand.operator*(rotationLeftHandToEE);

                        tf::Transform rotationLeftHandToEE2(
        tf::Quaternion(tf::Vector3(0, 0, 1), -M_PI),
        tf::Vector3(0, 0, 0));

             transform_left_hand =
        transform_left_hand.operator*(rotationLeftHandToEE2);

        // Publish the transform from "world" to "LeftHand"
        tf_broadcaster.sendTransform(tf::StampedTransform(transform_left_hand, current_time, "panda_1_link0", "LeftHandPn"));

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
