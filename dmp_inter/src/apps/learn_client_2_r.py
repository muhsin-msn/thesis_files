#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from dmp_int.srv import *

import yaml

if __name__ == "__main__":

    rospy.init_node('learn_dmp_service_test_client')
    req = DMPLoadRequest()


    #poses_yaml_path_r = "path/to/filedata/extracted_poses/recording_20231010162800_right_aic_9.yaml"  # Replace with the actual path to your poses YAML file
    #path/to/filedata/extracted_poses/recording_20231026133650_right_aic_101.yaml
    #path/to/filedata/trimmed/moving/move_s.yaml
    #path/to/filedata/trimmed/moving/peg_s.yaml
    #path/to/filedata/extracted_poses/recording_20231026133848_right_aic_200.yaml
    
    #poses_yaml_path_r = "path/to/filedata/trimmed/moving/move_s.yaml"
    poses_yaml_path_r = "path/data/extracted_poses/recording_20231026133831_right_aic_201.yaml"
    with open(poses_yaml_path_r, "r") as f:
        poses_r = yaml.safe_load(f)


    req.header.frame_id = 'world'
    req.file_name = 'example2.yaml'
    req.dmp_name = 'square_wave'
    req.header.stamp = rospy.Time.now()
    req.n_bfs = 500
    req.n_dmps =6

    
  

    for pose_data in poses_r:
        pose_r = Pose()
        pose_r.position.x = pose_data['position']['x']
        pose_r.position.y = pose_data['position']['y']
        pose_r.position.z = pose_data['position']['z']
        pose_r.orientation.x = pose_data['orientation']['x']
        pose_r.orientation.y = pose_data['orientation']['y']
        pose_r.orientation.z = pose_data['orientation']['z']
        pose_r.orientation.w = pose_data['orientation']['w']
        req.poses_r.append(pose_r)

    try:
        service_client = rospy.ServiceProxy('/learn_dynamic_motion_primitive_service_2', DMPLoad)
        response = service_client(req)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)


