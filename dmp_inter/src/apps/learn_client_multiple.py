#!/usr/bin/env python
import rospy
import numpy as np
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

        self.paths = [
            "path/to/filedata/trimmed/moving/move_29_1.yaml",
            "path/to/filedata/trimmed/moving/move_small.yaml",
            "path/to/filedata/trimmed/moving/peg_29_1.yaml",
            "path/to/filedata/extracted_poses/recording_20231031143135_right_aic_wobble_right_2.yaml",
            "path/to/filedata/extracted_poses/wiggle/round_2/recording_20231101135441_right_aic_1.yaml",
            "path/to/filedata/extracted_poses/wiggle/round_2/recording_20231101135448_right_aic_2.yaml"
        ]
        

        #legit ones   path/to/filedata/extracted_poses/wiggle/round_1/recording_20231101135246_right_aic_4.yaml
        #path/to/filedata/extracted_poses/wiggle/round_2/recording_20231101135441_right_aic_1.yaml
        #path/to/filedata/extracted_poses/wiggle/round_2/recording_20231101135448_right_aic_2.yaml
        #path/to/filedata/extracted_poses/wiggle/round_2/recording_20231101135455_right_aic_3.yaml
        #path/to/filedata/trimmed/moving/backwards_3.yaml
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
        poses_yaml_path_r = self.paths[self.path_index]
        print("poses_yaml_path_r", poses_yaml_path_r)

        with open(poses_yaml_path_r, "r") as f:
            poses_r = yaml.safe_load(f)

        self.req.header.frame_id = 'world'
        self.req.file_name = 'example2.yaml'
        self.req.dmp_name = 'square_wave'
        self.req.header.stamp = rospy.Time.now()
        self.req.n_bfs = 500
        self.req.n_dmps = 6

        for pose_data in poses_r:
            pose_r = Pose()
            pose_r.position.x = pose_data['position']['x']
            pose_r.position.y = pose_data['position']['y']
            pose_r.position.z = pose_data['position']['z']
            pose_r.orientation.x = pose_data['orientation']['x']
            pose_r.orientation.y = pose_data['orientation']['y']
            pose_r.orientation.z = pose_data['orientation']['z']
            pose_r.orientation.w = pose_data['orientation']['w']
            self.req.poses_r.append(pose_r)

        try:
            service_client = rospy.ServiceProxy('/learn_dynamic_motion_primitive_service_2', DMPLoad)
            response = service_client(self.req)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" % e)

if __name__ == "__main__":
    dmp_client = DMPServiceClient()
    dmp_client.run()
