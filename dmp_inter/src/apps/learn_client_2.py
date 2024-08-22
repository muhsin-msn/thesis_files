#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from dmp_int.srv import *
import yaml

if __name__ == "__main__":

    rospy.init_node('learn_dmp_service_test_client')
    req = DMPLoadRequest()

    # Load poses from the YAML file
    #path/to/filedata/extracted_poses/tf_poses_extracted_left_f2.yaml
    
    # poses_yaml_path_l = "path/to/filedata/extracted_poses/candle/recording_20231108141838_left_aic_candle.yaml"  # Replace with the actual path to your poses YAML file
    # with open(poses_yaml_path_l, "r") as f:
    #     poses_l = yaml.safe_load(f)
    
    # #path/to/filedata/extracted_poses/tf_poses_extracted_right_f2.yaml
    # poses_yaml_path_r = "path/to/filedata/extracted_poses/candle/recording_20231108141838_right_aic_candle.yaml"  # Replace with the actual path to your poses YAML file
    # with open(poses_yaml_path_r, "r") as f:
    #     poses_r = yaml.safe_load(f)


    #  poses_yaml_path_l = "path/to/filedata/extracted_poses/candle/recording_20231109133300_final2_left_aic_final_2.yaml"  # Replace with the actual path to your poses YAML file
    # with open(poses_yaml_path_l, "r") as f:
    #     poses_l = yaml.safe_load(f)
    
    # #path/to/filedata/extracted_poses/tf_poses_extracted_right_f2.yaml
    # #poses_yaml_path_r = "path/to/filedata/extracted_poses/backwards_straight/recording_20231107183529_right_aic_7_11_7.yaml"  # Replace with the actual path to your poses YAML file
    # poses_yaml_path_r = "path/to/filedata/extracted_poses/candle/recording_20231109133300_final2_right_aic_final_2.yaml"



    #path/to/filedata/extracted_poses/backwards_straight/recording_20231107183529_left_aic_7_11_7.yaml
    #poses_yaml_path_l = "path/to/filedata/extracted_poses/backwards_straight/recording_20231107183529_left_aic_7_11_7.yaml"  # Replace with the actual path to your poses YAML file
    poses_yaml_path_l = /path/data/dual_carrying/hold/recording_20240125153734_left_aic_01.yaml"
    
    with open(poses_yaml_path_l, "r") as f:
        poses_l = yaml.safe_load(f)
    
    #path/to/filedata/extracted_poses/tf_poses_extracted_right_f2.yaml
    #poses_yaml_path_r = "path/to/filedata/extracted_poses/backwards_straight/recording_20231107183529_right_aic_7_11_7.yaml"  # Replace with the actual path to your poses YAML file
    #poses_yaml_path_r = "path/to/filedata/extracted_poses/candle/recording_20231109133300_final2_right_aic_final_2.yaml"
    poses_yaml_path_r = /path/data/dual_carrying/hold/recording_20240125153734_right_aic_01.yaml"
    
    with open(poses_yaml_path_r, "r") as f:
        poses_r = yaml.safe_load(f)


    req.header.frame_id = 'world'
    req.file_name = 'example2.yaml'
    req.dmp_name = 'square_wave'
    req.header.stamp = rospy.Time.now()
    req.n_bfs = 1000
    req.n_dmps =12

    for pose_data in poses_l:
        pose_l = Pose()
        pose_l.position.x = pose_data['position']['x']
        pose_l.position.y = pose_data['position']['y']
        pose_l.position.z = pose_data['position']['z']
        pose_l.orientation.x = pose_data['orientation']['x']
        pose_l.orientation.y = pose_data['orientation']['y']
        pose_l.orientation.z = pose_data['orientation']['z']
        pose_l.orientation.w = pose_data['orientation']['w']
        req.poses_l.append(pose_l)
    
    """ print('pose_l', req.poses_l)
    input('pose ') """

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


