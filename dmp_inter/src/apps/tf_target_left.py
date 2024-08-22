#!/usr/bin/env python

import rospy
import tf
import yaml
from geometry_msgs.msg import TransformStamped
import numpy as np

def publish_last_tf_from_yaml(yaml_file):
    rospy.init_node('tf_publisher_target')
    broadcaster = tf.TransformBroadcaster()

    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)

    last_transformation = data[-1]

    while not rospy.is_shutdown():
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = "target_left"

        transform.transform.translation.x = last_transformation['position']['x']
        transform.transform.translation.y = last_transformation['position']['y']
        transform.transform.translation.z = last_transformation['position']['z']


        # Define the rotations to apply
        rotationRightHandToEE2 = tf.transformations.quaternion_about_axis(-np.pi, (1, 0, 0))
        rotationRightHandToEE = tf.transformations.quaternion_about_axis(np.pi, (0, -1, 0))
        rotationRightHandToEE3 = tf.transformations.quaternion_about_axis(-np.pi, (1, 0, 0))

        # Convert the existing rotation to a quaternion
        existing_rotation = [
            last_transformation['orientation']['x'],
            last_transformation['orientation']['y'],
            last_transformation['orientation']['z'],
            last_transformation['orientation']['w']
        ]

        # Apply the rotations to the existing rotation
        transformed_rotation = tf.transformations.quaternion_multiply(existing_rotation, rotationRightHandToEE)
        transformed_rotation = tf.transformations.quaternion_multiply(transformed_rotation, rotationRightHandToEE2)
        # transformed_rotation = tf.transformations.quaternion_multiply(transformed_rotation, rotationRightHandToEE3)

        # Set the new rotation
        transform.transform.rotation.x = transformed_rotation[0]
        transform.transform.rotation.y = transformed_rotation[1]
        transform.transform.rotation.z = transformed_rotation[2]
        transform.transform.rotation.w = transformed_rotation[3]

        broadcaster.sendTransform(
            (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
            (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
            transform.header.stamp,
            transform.child_frame_id,
            transform.header.frame_id
        )
  # Publish every 1 second (adjust as needed)
        rospy.sleep(0.05)

        # transform.transform.rotation.x = last_transformation['orientation']['x']
        # transform.transform.rotation.y = last_transformation['orientation']['y']
        # transform.transform.rotation.z = last_transformation['orientation']['z']
        # transform.transform.rotation.w = last_transformation['orientation']['w']

        # broadcaster.sendTransform(
        #     (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
        #     (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
        #     transform.header.stamp,
        #     transform.child_frame_id,
        #     transform.header.frame_id
        # )
        # rospy.sleep(0.5)  # Publish every 1 second (adjust as needed)

if __name__ == '__main__':
    yaml_file_path = 'path/to/filedata/trimmed/moving/peg_s.yaml'  # Replace with the actual file path
    publish_last_tf_from_yaml(yaml_file_path)
