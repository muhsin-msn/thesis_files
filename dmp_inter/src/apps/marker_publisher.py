#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf.transformations import quaternion_from_euler , quaternion_about_axis, quaternion_multiply
from tf import transformations as tfs


class TransformModifier:
    def __init__(self):
        rospy.init_node('transform_modifier_node', anonymous=True)
        
        self.target_frame = "panda_1_link0"
        self.source_frame = "world"
        
        #self.translation_offset = (0.1, -0.178, -0.163)  # 0.5 left, 0 forward, 0.1 below
        self.translation_offset = (0.09, -0.23, -0.0)
        
        self.offset_right = np.array([-0.06, 0.055, -0.015])
        self.offset_left = np.array([0.06, 0.055, -0.015])

        # self.offset_right = np.array([-0.05, -0.01, -0.00])
        # self.offset_left = np.array([0.05, -0.01, -0.00])
        self.left_rotation_matrix = tf.transformations.rotation_matrix(np.pi / 2, [0, -1, 0])
        self.right_rotation_matrix = tf.transformations.rotation_matrix(-np.pi / 2, [0, 1, 0])


        self.right_rotation_angle = -np.pi / 2  # 90 degrees around its own y-axis
        self.right_rotation_quaternion = quaternion_about_axis(-np.pi / 2, [0, 1, 0])

        self.left_rotation_angle = np.pi / 2  # 90 degrees around its own y-axis
        self.left_rotation_quaternion = quaternion_about_axis(self.left_rotation_angle, [0, 1, 0])

        self.left_dmp_frame = "left_dmp_tf"
        self.right_dmp_frame = "right_dmp_tf"



        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.modify_and_broadcast_transform()

    def modify_and_broadcast_transform(self):
        rate = rospy.Rate(10.0)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform(self.source_frame, self.target_frame, rospy.Time(0))
                (trans_aruco_to_marker, rot_aruco_to_marker) = self.listener.lookupTransform("marker", "aruco_marker_frame", rospy.Time(0))

                trans[0] += self.translation_offset[0]
                trans[1] += self.translation_offset[1]
                trans[2] += self.translation_offset[2]

                transform_msg = TransformStamped()
                transform_msg.header.frame_id = self.source_frame
                transform_msg.header.stamp = rospy.Time.now()
                transform_msg.child_frame_id = "/marker"
                transform_msg.transform.translation.x = trans[0]
                transform_msg.transform.translation.y = trans[1]
                transform_msg.transform.translation.z = trans[2]
                transform_msg.transform.rotation.x = rot[0]
                transform_msg.transform.rotation.y = rot[1]
                transform_msg.transform.rotation.z = rot[2]
                transform_msg.transform.rotation.w = rot[3]

                self.broadcaster.sendTransformMessage(transform_msg)

                #left_trans = np.dot(self.left_rotation_matrix[:3, :3], self.offset_left)

                rotation_y = quaternion_about_axis(-np.pi, [0, 1, 0])

                rotation_x = quaternion_about_axis(-np.pi/2+np.deg2rad(26) , [1, 0, 0])  # Replace 'angle_in_radians' with your desired angle

                combined_rotation = quaternion_multiply(rotation_x, rotation_y)

                rotated_left_quaternion = tf.transformations.quaternion_multiply(rot_aruco_to_marker, self.left_rotation_quaternion)
                self.broadcaster.sendTransform(

                    translation=self.offset_left,
                    #rotation=tf.transformations.quaternion_from_matrix(self.left_rotation_matrix),
                    #rotation=(0.0, 0.0, 0.0, 1.0), 
                    rotation=combined_rotation, 
                    #rotation=rotation_y,
                    time=rospy.Time.now(),
                    child= "marker_left",
                    parent="/aruco_marker_frame"
                )

                right_trans = np.dot(self.right_rotation_matrix[:3, :3], self.offset_right)
                rotated_right_quaternion = tf.transformations.quaternion_multiply(rot_aruco_to_marker, self.right_rotation_quaternion)
                
                rotation_y = quaternion_about_axis(np.pi / 2, [0, 1, 0])

                rotation_x = quaternion_about_axis(np.pi/2+ np.deg2rad(22), [1, 0, 0])  # Replace 'angle_in_radians' with your desired angle

                combined_rotation_2 = quaternion_multiply(rotation_x, rotation_y)
                
                self.broadcaster.sendTransform(
                    translation=self.offset_right,
                    #rotation=tf.transformations.quaternion_from_matrix(self.right_rotation_matrix),
                    #rotation=(0.0, 0.0, 0.0, 1.0), 
                    #rotation=combined_rotation_2,
                    rotation=rotation_x,
                    time=rospy.Time.now(),
                    child= "marker_right",
                    parent="/aruco_marker_frame"
                )


           




            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Transformation lookup failed. Retrying...")

            rate.sleep()

if __name__ == '__main__':
    try:
        transform_modifier = TransformModifier()
    except rospy.ROSInterruptException:
        pass
