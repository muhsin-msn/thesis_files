#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from dmp_int.srv import *
from std_msgs.msg import String
import yaml

class DMPServiceClient:
    def __init__(self):
        rospy.init_node('dmp_service_client')
        self.req = DMPLoadRequest()
        self.execute_signal = ""
        self.path_index = 0
        self.weights = ["weights_1", "weights_2", "weights_3", "weights_4", "weights_5", "weights_6"]

        self.paths_left = [
            "path/to/filedata/dual_carrying/hold/recording_20240125153734_left_aic_01.yaml",
            "path/to/filedata/dual_carrying/hold/recording_20240129160535_left_aic_04.yaml",
            "path/to/filedata/dual_carrying/hold/recording_20240129140145_left_aic_03.yaml",
            "path/to/filedata/dual_carrying/hold/recording_20240129140316_left_aic_release_2.yaml",
        ]

        self.paths_right = [
            "path/to/filedata/dual_carrying/hold/recording_20240125153734_right_aic_01.yaml",
            "path/to/filedata/dual_carrying/hold/recording_20240129160535_right_aic_04.yaml",
            "path/to/filedata/dual_carrying/hold/recording_20240129140145_right_aic_03.yaml",
            "path/to/filedata/dual_carrying/hold/recording_20240129140316_right_aic_release_2.yaml",
        ]

        rospy.Subscriber("play_pause_signal", String, self.play_pause_callback)

    def reset_service(self):
        self.req = DMPLoadRequest()

    def play_pause_callback(self, msg):
        if msg.data in self.weights:
            self.execute_signal = msg.data

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            for index, weight in enumerate(self.weights, start=1):
                if self.execute_signal == weight:
                    self.path_index = index - 1  # Adjusted index
                    self.call_dmp_service()
                    self.reset_service()
                    self.execute_signal = ""
            rate.sleep()

    def call_dmp_service(self):
        poses_yaml_path_left = self.paths_left[self.path_index]
        poses_yaml_path_right = self.paths_right[self.path_index]

        print("poses_yaml_path_left", poses_yaml_path_left)
        print("poses_yaml_path_right", poses_yaml_path_right)

        with open(poses_yaml_path_left, "r") as f_left:
            poses_left = yaml.safe_load(f_left)

        with open(poses_yaml_path_right, "r") as f_right:
            poses_right = yaml.safe_load(f_right)

        self.req.header.frame_id = 'world'
        self.req.file_name = 'example2.yaml'
        self.req.dmp_name = 'square_wave'
        self.req.header.stamp = rospy.Time.now()
        self.req.n_bfs = 500  # You may adjust this value
        self.req.n_dmps = 12

        for pose_data in poses_left:
            pose_left = Pose()
            pose_left.position.x = pose_data['position']['x']
            pose_left.position.y = pose_data['position']['y']
            pose_left.position.z = pose_data['position']['z']
            pose_left.orientation.x = pose_data['orientation']['x']
            pose_left.orientation.y = pose_data['orientation']['y']
            pose_left.orientation.z = pose_data['orientation']['z']
            pose_left.orientation.w = pose_data['orientation']['w']
            self.req.poses_l.append(pose_left)

        for pose_data in poses_right:
            pose_right = Pose()
            pose_right.position.x = pose_data['position']['x']
            pose_right.position.y = pose_data['position']['y']
            pose_right.position.z = pose_data['position']['z']
            pose_right.orientation.x = pose_data['orientation']['x']
            pose_right.orientation.y = pose_data['orientation']['y']
            pose_right.orientation.z = pose_data['orientation']['z']
            pose_right.orientation.w = pose_data['orientation']['w']
            self.req.poses_r.append(pose_right)

        try:
            service_client = rospy.ServiceProxy('/learn_dynamic_motion_primitive_service_2', DMPLoad)
            response = service_client(self.req)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" % e)

if __name__ == "__main__":
    dmp_client = DMPServiceClient()
    dmp_client.run()
