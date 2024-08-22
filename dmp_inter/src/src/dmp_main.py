#!/usr/bin/env python3
import yaml

import rospy
import sys
import tf
import std_msgs
import numpy as np
from os.path import join
from geometry_msgs.msg import PoseStamped, TransformStamped,Pose
from nav_msgs.msg import Path
from dmp_int.srv import *
from tf2_ros import TransformBroadcaster
import tkinter as tk
from std_msgs.msg import String
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
from tf.transformations import euler_matrix
from visualization_msgs.msg import Marker, MarkerArray

import math
from dmp import dmp_cartesian as dmp2

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_inverse




class DmpLoad:
    def __init__(self):
        '''Ros interface for learning DMP

        Initializes the learn DMP service
        '''
        rospy.init_node("learn_dynamic_motion_primitive_service")
        service_ = rospy.Service('learn_dynamic_motion_primitive_service_2',DMPLoad, self.learn_dmp_handler)
        rospy.loginfo("Started learn DMP service")

        self.service = rospy.Service('/service_execute_1', DMPLoad, self.execute_callback_1)
        self.service4 = rospy.Service('/service_execute_3', DMPLoad, self.execute_callback_3)

        
        self.gamma = 400
        self.beta = 20.0 / math.pi

        # Publishers
        self.imitated_path_r_pub = rospy.Publisher("~imitated_path_r", Path, queue_size=1)
        self.imitated_path_l_pub = rospy.Publisher("~imitated_path_l", Path, queue_size=1)
        self.demonstrated_path_pub_l = rospy.Publisher("~demonstrated_path_l", Path, queue_size=1)
        self.demonstrated_path_pub_r = rospy.Publisher("~demonstrated_path_r", Path, queue_size=1)

        self.robot_path_pub_r = rospy.Publisher("~robot_path_r", Path, queue_size=1)
        self.robot_path_pub_l = rospy.Publisher("~robot_path_l", Path, queue_size=1)

        self.goal_marker_pub = rospy.Publisher("goal_marker", Marker, queue_size=10)
        self.obstacle_marker_pub = rospy.Publisher("obstacle_markers", MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher("dmp_path", Path, queue_size=10)

        

        
        rospy.Subscriber("play_pause_signal", String, self.play_pause_callback)
        self.listener = tf.TransformListener()
        self.tf_timer = rospy.Timer(rospy.Duration(0.2), self.tf_callback)
        self.tf_timer_3 = rospy.Timer(rospy.Duration(0.2), self.path_callback_2)
        self.tf_timer_2 = rospy.Timer(rospy.Duration(0.2), self.tf_callback_2)
        

      
        self.weights_file_path = rospy.get_param('~weights_file_path', '../../data/weights/')
        # loop_rate = rospy.get_param('~loop_rate')


        self.result = ""
        # r = rospy.Rate(loop_rate)
        self.pose_array = []
        self.new_pose_array =np.empty((0,))
        self.pose_array_2 = []
        self.pose_array_3 =[]
        self.new_goal_pose=np.empty((0,))
        self.new_pose = [] 
        self.vel = np.zeros(12)

        self.pose_array_marker_left = []
        self.pose_array_marker_right = []
        
        self.enter_signal = None
        self.initial_obstacle_diff = None
        self.increase_factor= None
        self.decrease_factor = None
        self.counter = 0

        self.path_r = None
       
        self.path_l = None
        #self.path_l.header.frame_id = "/world"
        
                                 
        
        rospy.spin()


    
    def execute_callback_1(self, req):
 
        self.execute()
        
        
        response = DMPLoadResponse()
        response.result = "True"
        #self.execute_flag = True
        return response
    
    def execute_callback_3(self, req):
      
        self.execute_3()
        
        # Return a response (if needed)
        response = DMPLoadResponse()
        response.result = "True"
        #self.execute_flag = True
        return response
    
  



    

    

    def path_callback_2(self, event):
       
            

        if  self.path_r:
            # Publish the accumulated path_r
            self.robot_path_pub_r.publish(self.path_r)
            self.robot_path_pub_l.publish(self.path_l)
            
            

            
    def tf_callback_2(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform("/world", "/right_dmp_tf", rospy.Time(0))

            (trans_2, rot_2) = self.listener.lookupTransform("/world", "/right_controller_2", rospy.Time(0))
            
            # rotationRightHandToEE2 = tf.transformations.quaternion_about_axis(np.pi/2, (0, 1, 0))
            # rotationRightHandToEE = tf.transformations.quaternion_about_axis(-np.pi, (0, 0, 1))
            # rotationRightHandToEE3 = tf.transformations.quaternion_about_axis(-np.pi, (1, 0, 0))
            
            # # Apply the rotations to the existing rotation
            # transformed_rotation = tf.transformations.quaternion_multiply(rot, rotationRightHandToEE)
            # transformed_rotation = tf.transformations.quaternion_multiply(transformed_rotation, rotationRightHandToEE2)
            # transformed_rotation = tf.transformations.quaternion_multiply(transformed_rotation, rotationRightHandToEE3)




            euler_angles = tf.transformations.euler_from_quaternion(rot)
            offset_z = -0.01  # 5 cm in meters
            offset_y = -0.02 
            offset_x = -0.09 
            # trans[1] += offset_z    #right  
            #trans[2] += offset_y 
            # trans[0] += offset_x   # front
            self.pose_array_2 = trans + list(euler_angles)  # Concatenate translation and rotation

            euler_angles_2 = tf.transformations.euler_from_quaternion(rot_2)
            self.pose_array_3 = trans_2 + list(euler_angles_2)
            #print("Pose Array:", self.pose_array)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
    def tf_callback(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform("/world", "/left_dmp_tf", rospy.Time(0))
            
            rotationRightHandToEE2 = tf.transformations.quaternion_about_axis(np.pi/2, (0, 1, 0))
            rotationRightHandToEE = tf.transformations.quaternion_about_axis(-np.pi, (0, 0, 1))
            rotationRightHandToEE3 = tf.transformations.quaternion_about_axis(-np.pi, (1, 0, 0))
            
          
            transformed_rotation = tf.transformations.quaternion_multiply(rot, rotationRightHandToEE)
            transformed_rotation = tf.transformations.quaternion_multiply(transformed_rotation, rotationRightHandToEE2)
            transformed_rotation = tf.transformations.quaternion_multiply(transformed_rotation, rotationRightHandToEE3)


            euler_angles = tf.transformations.euler_from_quaternion(transformed_rotation)
            offset_z = -0.01  # 5 cm in meters
            offset_y = -0.02 
            offset_x = -0.01 
            # trans[1] += offset_z    #right  
            # trans[2] += offset_y 
            # trans[0] += offset_x   # front
            modified_pose_array = trans + list(euler_angles) 
            
            #modified_pose_array = np.array(self.pose_array)
            rotation_matrix_i = Rotation.from_euler('xyz', modified_pose_array[3:6], degrees=False).as_matrix()

            offset_i = np.array([0.00,-0.02, -0.01]) # z  y
            #offset_i = np.array([0.00,-0.00, -0.00]) 
            modified_pose_array[:3] += np.dot(rotation_matrix_i, offset_i)
            self.pose_array = modified_pose_array # Concatenate translation and rotation
            self.publish_target_tf(self.pose_array)
        #print("Pose Array:", self.pose_array)

            (trans_marker_left, rot_marker_left) = self.listener.lookupTransform("/world", "/marker_left", rospy.Time(0))

            (trans_marker_right, rot_marker_right) = self.listener.lookupTransform("/world", "/marker_right", rospy.Time(0))

         
            self.pose_array_marker_left = trans_marker_left + list(tf.transformations.euler_from_quaternion(rot_marker_left))
            self.pose_array_marker_right = trans_marker_right + list(tf.transformations.euler_from_quaternion(rot_marker_right))
            self.array_marker_left = np.array(self.pose_array_marker_left).copy()
            self.array_marker_right = np.array(self.pose_array_marker_right).copy()




        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")

    def calculate_new_goal_pose(self, initial_pose, goal_pose, new_pose):
        #modified_pose_array = new_pose.copy()

        
        translation_initial = initial_pose[:3]
        translation_goal = goal_pose[:3]

     
        rotation_initial = initial_pose[3:]
        rotation_goal = goal_pose[3:]

        # Convert rotation vectors to rotation matrices
        r_initial = Rotation.from_euler('xyz', rotation_initial, degrees=False).as_matrix()
        r_goal = Rotation.from_euler('xyz', rotation_goal, degrees=False).as_matrix()


        T_world_to_initial = np.eye(4)
        T_world_to_initial[:3, :3] = r_initial
        T_world_to_initial[:3, 3] = translation_initial

        T_world_to_goal = np.eye(4)
        T_world_to_goal[:3, :3] = r_goal
        T_world_to_goal[:3, 3] = translation_goal

      
        T_initial_to_world_inv = np.linalg.inv(T_world_to_initial)


        T_initial_to_goal = np.dot(T_initial_to_world_inv, T_world_to_goal)

        new_translation_initial = new_pose[:3]
        new_rotation_initial = new_pose[3:]


        new_r_initial = Rotation.from_euler('xyz', new_rotation_initial, degrees=False).as_matrix()

       
        T_world_to_new_initial = np.eye(4)
        T_world_to_new_initial[:3, :3] = new_r_initial
        T_world_to_new_initial[:3, 3] = new_translation_initial

     
        T_new_initial_to_goal = np.dot(T_world_to_new_initial, T_initial_to_goal)


        new_translation_goal = T_new_initial_to_goal[:3, 3]

      
        new_r_goal = T_new_initial_to_goal[:3, :3]
        new_rotation_goal = Rotation.from_matrix(new_r_goal).as_euler('xyz', degrees=False)

    
        new_goal_pose = np.concatenate((new_translation_goal, new_rotation_goal))
        
        return new_goal_pose
    

    def calculate_new_goal_pose_dual_arm(self, initial_pose, goal_pose, new_pose):
        

 
        initial_left_arm = initial_pose[:6]
        initial_right_arm = initial_pose[6:]

        goal_left_arm = goal_pose[:6]
        goal_right_arm = goal_pose[6:]


        new_goal_left_arm = self.calculate_new_goal_pose(initial_left_arm, goal_left_arm, new_pose[:6])

       
        new_goal_right_arm = self.calculate_new_goal_pose(initial_right_arm, goal_right_arm, new_pose[6:])

        new_goal_pose = np.concatenate((new_goal_left_arm, new_goal_right_arm))

        return new_goal_pose


    def play_pause_callback(self, msg):
        if msg.data == "pause":
            self.pause = True
        elif msg.data == "play":
            self.pause = False
        elif msg.data == "enter":
            self.enter_signal  = "enter"
        


    def learn_dmp_handler(self, req):
        '''Handler for client request

        req: service request msg
        '''
        rospy.loginfo("Recieved request to learn a motion primitive")
        
        rospy.loginfo("Learning motion primitive " + req.dmp_name)

        

        
        self.learn_dmp(req.poses_l,req.poses_r, req.file_name,n_dmps=req.n_dmps,  n_bfs=req.n_bfs)
        rospy.loginfo("Successfully learned the motion primitive")
        # Return response
        response = DMPLoadResponse()
        response.result = self.result
        return response

    def learn_dmp(self, poses_l,poses_r, file_name, n_dmps=12, n_bfs=50,w=None):
      
        if poses_l:
        
            trajectory_l = np.zeros((6, len(poses_l)))

            for i in range(len(poses_l)):
                rpy = tf.transformations.euler_from_quaternion([poses_l[i].orientation.x,
                                                                poses_l[i].orientation.y,
                                                                poses_l[i].orientation.z,
                                                                poses_l[i].orientation.w])
                trajectory_l[:, i] = [poses_l[i].position.x, poses_l[i].position.y,
                                    poses_l[i].position.z, rpy[0], rpy[1], rpy[2]]
                
            print('trajectory_l', trajectory_l)
            """ print('trajectory_l', trajectory_l.shape)
            print('trajectory_l', trajectory_l)
            input('pose ') """
            self.demonstrated_trajectory_l = trajectory_l.copy()
            self.demonstrated_goal_pose_l = self.demonstrated_trajectory_l[:, -1]
            self.demonstrated_initial_pose_l = self.demonstrated_trajectory_l[:, 0]


            self.goal_pose_l =self.demonstrated_goal_pose_l
            self.initial_pose_l =self.demonstrated_initial_pose_l 
          


            


            trajectory_l -= trajectory_l[:, 0][:, None]

        trajectory_r = np.zeros((6, len(poses_r)))

        for i in range(len(poses_r)):
            rpy = tf.transformations.euler_from_quaternion([poses_r[i].orientation.x,
                                                            poses_r[i].orientation.y,
                                                            poses_r[i].orientation.z,
                                                            poses_r[i].orientation.w])
            trajectory_r[:, i] = [poses_r[i].position.x, poses_r[i].position.y,
                                poses_r[i].position.z, rpy[0], rpy[1], rpy[2]]
        #print('trajectory_r', trajectory_r)    
        """ print('trajectory_r', trajectory_r.shape)
        print('trajectory_r', trajectory_r)
        input('trajectory_r ') """
        self.demonstrated_trajectory_r = trajectory_r.copy()
        self.demonstrated_goal_pose_r = self.demonstrated_trajectory_r[:, -1]
        self.demonstrated_initial_pose_r = self.demonstrated_trajectory_r[:, 0]


        self.goal_pose_r =self.demonstrated_goal_pose_r
        self.initial_pose_r =self.demonstrated_initial_pose_r 
        trajectory_r -= trajectory_r[:, 0][:, None]


        if poses_l:
            final_trajectory = np.vstack((trajectory_l, trajectory_r))
            self.goal_pose=  np.hstack((self.goal_pose_l, self.goal_pose_r))
            self.initial_pose = np.hstack((self.initial_pose_l, self.initial_pose_r))
        else:
            final_trajectory = trajectory_r
            self.goal_pose=  self.goal_pose_r
            self.initial_pose = self.initial_pose_r


        self.initiate_and_save_dmp(final_trajectory, n_bfs, n_dmps, file_name, poses_l)
      

    
    def initiate_and_save_dmp(self, final_trajectory, n_bfs, n_dmps, file_name, poses_l):
        print('n_bfs', n_bfs)
        #self.dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=n_dmps, n_bfs=n_bfs,ay=None, dt=0.005)
        # weights = self.dmp.imitate_path(y_des=final_trajectory)

        K = 1050.0
        alpha = 4.0
        n_dim = 6
        #n_bfs = 1000
        dt = 0.001
        tol = 0.02
        self.dmp = dmp2.DMPs_cartesian(n_dmps=n_dmps, n_bfs=n_bfs, K=K, dt=dt, alpha_s=alpha, tol=tol, rescale=None)
        gamma = np.transpose(final_trajectory)
        weights =self.dmp.imitate_path(x_des=gamma)


        if poses_l:
            demonstrated_path_l = Path()
            demonstrated_path_l.header.frame_id = "/world"
            for itr in range(self.demonstrated_trajectory_l.shape[1]):
                pose_stamped_l = PoseStamped()
                pose_stamped_l.pose.position.x = self.demonstrated_trajectory_l[0, itr]
                pose_stamped_l.pose.position.y = self.demonstrated_trajectory_l[1, itr]
                pose_stamped_l.pose.position.z = self.demonstrated_trajectory_l[2, itr]
                demonstrated_path_l.poses.append(pose_stamped_l)
            self.demonstrated_path_pub_l.publish(demonstrated_path_l)

        demonstrated_path_r = Path()
        demonstrated_path_r.header.frame_id = "/world"
        for itr in range(self.demonstrated_trajectory_r.shape[1]):
            pose_stamped_r = PoseStamped()
            pose_stamped_r.pose.position.x = self.demonstrated_trajectory_r[0, itr]
            pose_stamped_r.pose.position.y = self.demonstrated_trajectory_r[1, itr]
            pose_stamped_r.pose.position.z = self.demonstrated_trajectory_r[2, itr]
            demonstrated_path_r.poses.append(pose_stamped_r)
        self.demonstrated_path_pub_r.publish(demonstrated_path_r)

        self.result = "success"
    
    
    
    def reset_paths(self):
        self.path_r.poses[0].position.x = 0.0
        self.path_r.poses[0].position.y = 0.0
        self.path_r.poses[0].position.z = 0.0
        self.path_r.poses[0].orientation.x = 0.0
        self.path_r.poses[0].orientation.y = 0.0
        self.path_r.poses[0].orientation.z = 0.0
        self.path_r.poses[0].orientation.w = 1.0

        self.path_l.poses[0].position.x = 0.0
        self.path_l.poses[0].position.y = 0.0
        self.path_l.poses[0].position.z = 0.0
        self.path_l.poses[0].orientation.x = 0.0
        self.path_l.poses[0].orientation.y = 0.0
        self.path_l.poses[0].orientation.z = 0.0
        self.path_l.poses[0].orientation.w = 1.0


    def initialize_paths(self):
        
            self.path_r = Path()
            self.path_r.header.frame_id = "/world"
            self.path_l = Path()
            self.path_l.header.frame_id = "/world"
            

    def execute(self):    

  
        self.initial_pose =np.array(self.initial_pose)
        
        # self.goal_pose=  np.hstack((array_marker_left, array_marker_right))


        # array_marker_left = np.array(self.pose_array_marker_left).copy()
        # array_marker_right = np.array(self.pose_array_marker_right).copy()

        #self.goal_pose= np.array(self.goal_pose)
        # self.goal_pose_2=  np.hstack((self.array_marker_left, self.array_marker_right))

        # decrease_values = np.array([0, -0.1, 0, 0, 0, 0, 0, -0.1, 0, 0, 0, 0])
        decrease_values = np.array([0, -0.1, 0, 0, 0, 0])

        # self.goal_pose_2 = self.goal_pose - decrease_values

        # print('goal_pose_1', np.hstack((array_marker_left, array_marker_right)))
        # print('goal_pose_2', self.goal_pose)
        # input('pos ')


        # modified_pose_array[1] -= 0.15
        # modified_pose_array[0] += 0.1
        #pos, vel, acc = self.dmp.rollout(goal=modified_pose_array, y0=self.initial_pose)

        self.dmp.x_goal = self.goal_pose
        # self.dmp.x_goal =self.goal_pose_2
        self.dmp.x_0 = self.initial_pose
        print('dmp.cs.timesteps', self.dmp.cs.timesteps)



        pos= self.dmp.rollout()[0]

        self.new_pose= pos[-1]


        """ print('pos', pos.shape)
        print('pos', pos)
        input('pos ') """
            

        if pos.shape[1]==12:
            imitated_path_l = Path()
            imitated_path_l.header.frame_id = "/world"
            imitated_path_r = Path()
            imitated_path_r.header.frame_id = "/world"
            for itr in range(pos.shape[0]):
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = pos[itr, 0]
                pose_stamped.pose.position.y = pos[itr, 1]
                pose_stamped.pose.position.z = pos[itr, 2]
                imitated_path_l.poses.append(pose_stamped)
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = pos[itr, 6]
                pose_stamped.pose.position.y = pos[itr, 7]
                pose_stamped.pose.position.z = pos[itr, 8]
                imitated_path_r.poses.append(pose_stamped)

            self.imitated_path_l_pub.publish(imitated_path_l)
            self.imitated_path_r_pub.publish(imitated_path_r)



            

        else:
            imitated_path_r = Path()
            imitated_path_r.header.frame_id = "/world"
            for itr in range(pos.shape[0]):
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = pos[itr, 0]
                pose_stamped.pose.position.y = pos[itr, 1]
                pose_stamped.pose.position.z = pos[itr, 2]
                imitated_path_r.poses.append(pose_stamped)
            self.imitated_path_r_pub.publish(imitated_path_r)



        

        



       

        # path_r = Path()
        # path_r.header.frame_id = "/world"
        # path_l = Path()
        # path_l.header.frame_id = "/world"
        
            
        # time_increment = 0.01

        
        # goal_increment = 0.02

        self.initialize_paths()

        self.dmp.reset_state()

        print("Press Enter to publish the cartesian_pose")
        print('self.enter_signal',self.enter_signal)
        while self.enter_signal != "enter":  # Wait for the "enter" signal
            rospy.sleep(0.1)
        
        self.pause = False
        #self.dmp.goal = np.array([4.5, 5, 0.5, 0.0, 0.0, 0.0])
        self.enter_signal="0"
        smoothing_factor = 0.9
        flag = False
        offset = 0.0

        deviation_data = []
        
        upper_limit = 0.05  # Adjust this value as needed
        lower_limit = -0.05
        for t in range(self.dmp.cs.timesteps):
         # Get DMP state
                
                while self.pause:
                    rospy.sleep(0.1)
             
                

                interpolation_factor = 1.075*t / (self.dmp.cs.timesteps - 1)

                # Interpolate btween the initial and new goal poses
                interpolated_goal_pose = (1 - interpolation_factor) * self.goal_pose + interpolation_factor * self.goal_pose_2



               ##########coupling
                # _,result_Cdot=self.coupling(self.dmp.x, self.dmp.dx)
                # combined_result = np.zeros(12)

                # # print('result_Cdot',result_Cdot)

                # combined_result[:3] = result_Cdot[:3]


                # combined_result[6:9] = result_Cdot[3:6]


                #pos, vel, acc = self.dmp.step(external_force=combined_result)
                
                self.dmp.x_goal = interpolated_goal_pose

            
                pos, vel, acc = self.dmp.step()

                
                # if t == 150:
                #     pos, vel, acc = self.dmp.step(external_force=combined_result)

                # else: 
                #     pos, vel, acc = self.dmp.step()
                self.new_pose= pos

             

                if len(pos)==12:
                    pose_stamped_l = PoseStamped()
                    pose_stamped_l.header.stamp = rospy.Time.now()
                    pose_stamped_l.header.frame_id = "/world"
                    pose_stamped_l.pose.position.x = pos[0]
                    pose_stamped_l.pose.position.y = pos[1]
                    pose_stamped_l.pose.position.z = pos[2]
                    quaternion = tf.transformations.quaternion_from_euler(pos[3], pos[4], pos[5])
                    pose_stamped_l.pose.orientation.x = quaternion[0]
                    pose_stamped_l.pose.orientation.y = quaternion[1]
                    pose_stamped_l.pose.orientation.z = quaternion[2]
                    pose_stamped_l.pose.orientation.w = quaternion[3]
                    self.path_l.poses.append(pose_stamped_l)

                    pose_stamped_r = PoseStamped()
                    pose_stamped_r.header.stamp = rospy.Time.now()
                    pose_stamped_r.header.frame_id = "/world"
                    pose_stamped_r.pose.position.x = pos[6]
                    pose_stamped_r.pose.position.y = pos[7]
                    pose_stamped_r.pose.position.z = pos[8]
                    quaternion = tf.transformations.quaternion_from_euler(pos[9], pos[10], pos[11])
                    pose_stamped_r.pose.orientation.x = quaternion[0]
                    pose_stamped_r.pose.orientation.y = quaternion[1]
                    pose_stamped_r.pose.orientation.z = quaternion[2]
                    pose_stamped_r.pose.orientation.w = quaternion[3]
                    self.path_r.poses.append(pose_stamped_r)    

                else:
                    pose_stamped_r = PoseStamped()
                    pose_stamped_r.header.stamp = rospy.Time.now()
                    pose_stamped_r.header.frame_id = "/world"
                    pose_stamped_r.pose.position.x = pos[0]
                    pose_stamped_r.pose.position.y = pos[1]
                    pose_stamped_r.pose.position.z = pos[2]
                    quaternion = tf.transformations.quaternion_from_euler(pos[3], pos[4], pos[5])
                    pose_stamped_r.pose.orientation.x = quaternion[0]
                    pose_stamped_r.pose.orientation.y = quaternion[1]
                    pose_stamped_r.pose.orientation.z = quaternion[2]
                    pose_stamped_r.pose.orientation.w = quaternion[3]
                    self.path_r.poses.append(pose_stamped_r)
              
                self.publish_tf(pos, vel, acc)

                # if  self.pose_array :
                #     #self.publish_target_tf(self.pose_array) 
                #     self.publish_target_tf(modified_pose_array)
                # else:
                #     self.publish_target_tf(self.goal_pose)
                
                
                # if  t==20 :
                #    self.publish_target_tf(self.goal_pose)
                # self.publish_target_tf(self.goal_pose)     
                # #print("self.goal_pose:", self.goal_pose)
                # self.robot_path_pub_r.publish(path_r)
                # self.robot_path_pub_l.publish(path_l)     

                #self.pose_array = [] 
                """ print("Position:", pos)
                print("Velocity:", vel)
                print("Acceleration:", acc) """

                # """ print('self.pose_array_marker_left', self.pose_array_marker_left)
                # print('self.pose_array_marker_right', self.pose_array_marker_right)
                # input('self.pose_array_marker_right ') """
                # rospy.sleep(0.005)
                rospy.sleep(0.05)
                #rospy.sleep(0.1)s
                
           
        
        
        # with open('path/to/filedata/limits/deviation_info.yaml', 'w') as yaml_file:
        #     yaml.dump(deviation_data, yaml_file)

        
        
        # self.pose_array = []
        # self.dmp.goal=self.goal_pose
        

    def publish_tf(self, pos, vel, acc):
        '''Publishes the current position as a TF transformation

        pos: DMP state, containing position and orientation
        '''
        
      
        

        
        if len(pos)==12:
            tf_broadcaster = TransformBroadcaster()
            parent_frame_id = "world"

            """ print('pos', pos.shape)
            print('pos', pos)
            input('pos ') """
            controller_frame_ids = ["left_controller_2", "right_controller_2"]

            for i, child_frame_id in enumerate(controller_frame_ids):
                transform_stamped = geometry_msgs.msg.TransformStamped()
                transform_stamped.header.stamp = rospy.Time.now()
                transform_stamped.header.frame_id = parent_frame_id
                transform_stamped.child_frame_id = child_frame_id

                translation_index = i * 6  
                rotation_index = 3 + translation_index

                # Set translation (position)
                transform_stamped.transform.translation.x = pos[translation_index]
                transform_stamped.transform.translation.y = pos[translation_index + 1]
                transform_stamped.transform.translation.z = pos[translation_index + 2]

                # Set rotation (orientation)
                quaternion =  tf.transformations.quaternion_from_euler(pos[rotation_index], pos[rotation_index + 1], pos[rotation_index + 2])
                transform_stamped.transform.rotation.x = quaternion[0]
                transform_stamped.transform.rotation.y = quaternion[1]
                transform_stamped.transform.rotation.z = quaternion[2]
                transform_stamped.transform.rotation.w = quaternion[3]

                tf_broadcaster.sendTransform(transform_stamped)   
        elif len(pos)==6:
                
                tf_broadcaster = TransformBroadcaster()

                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = rospy.Time.now()
                transform_stamped.header.frame_id = "/world"
                transform_stamped.child_frame_id = "/right_controller_2"  # Adjust the child frame ID as needed
                
                transform_stamped.transform.translation.x = pos[0]
                transform_stamped.transform.translation.y = pos[1]
                transform_stamped.transform.translation.z = pos[2]  # Assuming 3D position, adjust as needed
                
                quaternion = tf.transformations.quaternion_from_euler(pos[3], pos[4], pos[5])
                transform_stamped.transform.rotation.x = quaternion[0]
                transform_stamped.transform.rotation.y = quaternion[1]
                transform_stamped.transform.rotation.z = quaternion[2]
                transform_stamped.transform.rotation.w = quaternion[3]
                
                tf_broadcaster.sendTransform(transform_stamped)
    

    def euler_to_matrix(self, euler_angles):
        r = Rotation.from_euler('xyz', euler_angles, degrees=True)
        return r.as_matrix()

    def matrix_to_euler(self, matrix):
        r = Rotation.from_matrix(matrix)
        return r.as_euler('xyz', degrees=True)

    def find_transformation_matrix(self, from_pose, to_pose):
        from_matrix = self.euler_to_matrix(from_pose[3:])
        to_matrix = self.euler_to_matrix(to_pose[3:])
        transformation_matrix = np.linalg.inv(from_matrix) @ to_matrix
        return transformation_matrix

    def apply_transformation(self, transformation_matrix, new_pose):
        new_matrix = self.euler_to_matrix(new_pose[3:])
        transformed_matrix = transformation_matrix @ new_matrix
        transformed_pose = np.concatenate([new_pose[:3], self.matrix_to_euler(transformed_matrix)])
        return transformed_pose
    
    def transform_poses(self, obstacle_pose_array, current_pose_array):
        translation_initial =  current_pose_array[:3] 
        translation_goal = obstacle_pose_array[:3]

        

        rotation_initial =  current_pose_array[:3]
        rotation_goal = current_pose_array[3:]

        r_initial = Rotation.from_euler('xyz', rotation_initial, degrees=False).as_matrix()
        r_goal = Rotation.from_euler('xyz', rotation_goal, degrees=False).as_matrix()

        T_world_to_initial = np.eye(4)
        T_world_to_initial[:3, :3] = r_initial
        T_world_to_initial[:3, 3] = translation_initial

        T_world_to_goal = np.eye(4)
        T_world_to_goal[:3, :3] = r_goal
        T_world_to_goal[:3, 3] = translation_goal

        T_initial_to_world_inv = np.linalg.inv(T_world_to_initial)

        T_initial_to_goal = np.dot(T_initial_to_world_inv, T_world_to_goal)

        

        new_translation_goal = T_initial_to_goal[:3, 3]

        new_r_goal = T_initial_to_goal[:3, :3]
        new_rotation_goal = Rotation.from_matrix(new_r_goal).as_euler('xyz', degrees=False)

        new_goal_pose = np.concatenate((new_translation_goal, new_rotation_goal))

        return new_goal_pose

         
   

    
    def update_goal_pose(self,original_initial_pose, original_goal_pose, new_initial_pose):
        

        original_goal_position = original_goal_pose[:3]
        original_goal_orientation_rpy = original_goal_pose[3:]

        new_goal_position = original_goal_position + (new_initial_pose[:3] - original_initial_pose[:3])

        original_orientation = Rotation.from_euler('xyz', original_goal_orientation_rpy, degrees=False)
        new_orientation = Rotation.from_euler('xyz', new_initial_pose[3:], degrees=False)
        
        combined_rotation = new_orientation * original_orientation

        new_goal_orientation_rpy = combined_rotation.as_euler('xyz', degrees=False)

        new_goal_pose = np.concatenate((new_goal_position, new_goal_orientation_rpy))


        return new_goal_pose

  
    def create_obstacle_marker(self, obstacle, id):
        obstacle_marker = Marker()
        obstacle_marker.header.frame_id = "world"
        obstacle_marker.type = Marker.SPHERE
        obstacle_marker.action = Marker.ADD
        obstacle_marker.id = id
        obstacle_marker.pose.position.x = obstacle[0]
        obstacle_marker.pose.position.y = obstacle[1]
        obstacle_marker.pose.position.z = obstacle[2]
        obstacle_marker.scale.x = obstacle_marker.scale.y = obstacle_marker.scale.z = 0.01
        obstacle_marker.color.a = 1.0
        obstacle_marker.color.r = 1.0
        return obstacle_marker




    
    
    def coupling(self, y,y3):
        #da = y[:3] - y2[:3]

        #da =y
        desired_distance=y3

        da=np.array([0.00, 0,0])

        lf=(0.0, 1.0)
        k=1.0 
        c1=0.1
        c2=1000.0
     
        F12 = k * (-desired_distance - da)
        F21 = -F12
        C12 = c1 * F12 * lf[0]
        C21 = c1 * F21 * lf[1]
        C12dot = F12 * c2 * lf[0]
        C21dot = F21 * c2 * lf[1]
        return np.hstack([C12, C21]), np.hstack([C12dot, C21dot])
    

 


   

   
    def calculate_rotation_matrix(self, current_position, goal_position):

        
        direction_vector = goal_position[:3] - current_position[:3]

        
        
        normalized_direction = direction_vector / np.linalg.norm(direction_vector)
        
        rotation_axis = np.cross([1, 0, 0], normalized_direction)
        
        rotation_angle = np.arccos(np.dot([1, 0, 0], normalized_direction))
        
        c = np.cos(rotation_angle)
        s = np.sin(rotation_angle)
        t = 1 - c
        
        rotation_matrix = np.array([
            [t * rotation_axis[0] ** 2 + c, t * rotation_axis[0] * rotation_axis[1] - s * rotation_axis[2], t * rotation_axis[0] * rotation_axis[2] + s * rotation_axis[1]],
            [t * rotation_axis[0] * rotation_axis[1] + s * rotation_axis[2], t * rotation_axis[1] ** 2 + c, t * rotation_axis[1] * rotation_axis[2] - s * rotation_axis[0]],
            [t * rotation_axis[0] * rotation_axis[2] - s * rotation_axis[1], t * rotation_axis[1] * rotation_axis[2] + s * rotation_axis[0], t * rotation_axis[2] ** 2 + c]
        ])
        
        return rotation_matrix
    

    
    def execute_3(self):    

        self.initialize_paths()

        

       
            
        if self.new_pose.size == 12:

            self.initial_pose = np.array(self.initial_pose)
            self.goal_pose = np.array(self.goal_pose)
            self.new_pose = np.array(self.new_pose)
            modified_pose_array = self.new_pose.copy()

            new_goal_pose = self.calculate_new_goal_pose_dual_arm(self.initial_pose, self.goal_pose, modified_pose_array)
            



        else:
        
            self.initial_pose = np.array(self.initial_pose)
            self.goal_pose = np.array(self.goal_pose)
            self.new_pose = np.array(self.new_pose)
            modified_pose_array = self.new_pose.copy()

            
            #self.pose_array =  np.array(self.pose_array)  

            translation_initial = self.initial_pose[:3]
            translation_goal = self.goal_pose[:3]

            rotation_initial = self.initial_pose[3:]
            rotation_goal = self.goal_pose[3:]

            
            r_initial = Rotation.from_euler('xyz', rotation_initial, degrees=False).as_matrix()
            r_goal = Rotation.from_euler('xyz', rotation_goal, degrees=False).as_matrix()

            T_world_to_initial = np.eye(4)
            T_world_to_initial[:3, :3] = r_initial
            T_world_to_initial[:3, 3] = translation_initial

            T_world_to_goal = np.eye(4)
            T_world_to_goal[:3, :3] = r_goal
            T_world_to_goal[:3, 3] = translation_goal

            T_initial_to_world_inv = np.linalg.inv(T_world_to_initial)

            T_initial_to_goal = np.dot(T_initial_to_world_inv, T_world_to_goal)

            new_translation_initial = modified_pose_array[:3]
            new_rotation_initial = modified_pose_array[3:]

            new_r_initial = Rotation.from_euler('xyz', new_rotation_initial, degrees=False).as_matrix()

            T_world_to_new_initial = np.eye(4)
            T_world_to_new_initial[:3, :3] = new_r_initial
            T_world_to_new_initial[:3, 3] = new_translation_initial

            T_new_initial_to_goal = np.dot(T_world_to_new_initial, T_initial_to_goal)

            new_translation_goal = T_new_initial_to_goal[:3, 3]

            new_r_goal = T_new_initial_to_goal[:3, :3]
            new_rotation_goal = Rotation.from_matrix(new_r_goal).as_euler('xyz', degrees=False)

            # Combine translation and rotation to get the 6D XYZ RPY representation of the goal pose
            self.new_goal_pose = np.concatenate((new_translation_goal, new_rotation_goal))
                




       

            
        # new_goal_position=np.array(self.pose_array)    

        # self.dmp.x_goal = new_goal_position
        # self.dmp.x_0 = modified_pose_array
        # pos= self.dmp.rollout()[0]
        #input('pose')




        self.dmp.x_goal = self.goal_pose
        self.dmp.x_0 = self.initial_pose
        pos= self.dmp.rollout()[0]

        if pos.shape[1]==12:
            imitated_path_l = Path()
            imitated_path_l.header.frame_id = "/world"
            imitated_path_r = Path()
            imitated_path_r.header.frame_id = "/world"
            for itr in range(pos.shape[0]):
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = pos[itr, 0]
                pose_stamped.pose.position.y = pos[itr, 1]
                pose_stamped.pose.position.z = pos[itr, 2]
                imitated_path_l.poses.append(pose_stamped)
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = pos[itr, 6]
                pose_stamped.pose.position.y = pos[itr, 7]
                pose_stamped.pose.position.z = pos[itr, 8]
                imitated_path_r.poses.append(pose_stamped)

            self.imitated_path_l_pub.publish(imitated_path_l)
            self.imitated_path_r_pub.publish(imitated_path_r)



            

        else:

            imitated_path_r = Path()
            imitated_path_r.header.frame_id = "/world"
            for itr in range(pos.shape[0]):
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = pos[itr, 0]
                pose_stamped.pose.position.y = pos[itr, 1]
                pose_stamped.pose.position.z = pos[itr, 2]
                imitated_path_r.poses.append(pose_stamped)
            self.imitated_path_r_pub.publish(imitated_path_r)


        path_r = Path()
        path_r.header.frame_id = "/world"
        path_l = Path()
        path_l.header.frame_id = "/world"
        
            
        time_increment = 0.01

        
        goal_increment = 0.02
        self.dmp.reset_state()

       


        new_pose_array=self.dmp.x_goal - np.array([-0.01, 0, 0, 0.0, 0.0, 0.0])
       
        
        for t in range(self.dmp.cs.timesteps):
                
                while self.pause:
                    rospy.sleep(0.1)
                print("Step:", t)

              
                

                

                interpolation_factor = 1.075*t / (self.dmp.cs.timesteps - 1)

                interpolated_goal_pose = (1 - interpolation_factor) * self.new_goal_pose + interpolation_factor * new_pose_array
                
              

                desired_distance=np.array([-0.00, -0.015, 0.00])
              
                rotation_matrix_3 = Rotation.from_euler('xyz', interpolated_goal_pose[3:6], degrees=False).as_matrix()

              
                modified_pose_array_3= np.dot(rotation_matrix_3, desired_distance)



                    #  x -a,- z,y u
                original_distance=np.array([0, 0, 0])
                interpolated_goal_pose_2 = (1 - interpolation_factor) * original_distance + interpolation_factor * modified_pose_array_3[:3]

                
                
                self.dmp.x_goal = interpolated_goal_pose
               
                _,result_Cdot=self.coupling(None,interpolated_goal_pose_2)
               
                combined_result = np.zeros(6)

                print('x',self.dmp.x)

                combined_result[:3] = result_Cdot[3:6]


                #combined_result[6:9] = result_Cdot[3:6]
                #pos, vel, acc = self.dmp.step()
                pos, vel, acc= self.dmp.step(external_force=combined_result)
                pose_stamped_r = PoseStamped()
                pose_stamped_r.header.stamp = rospy.Time.now()
                pose_stamped_r.header.frame_id = "/world"
                pose_stamped_r.pose.position.x = pos[0]
                pose_stamped_r.pose.position.y = pos[1]
                pose_stamped_r.pose.position.z = pos[2]
                quaternion = tf.transformations.quaternion_from_euler(pos[3], pos[4], pos[5])
                pose_stamped_r.pose.orientation.x = quaternion[0]
                pose_stamped_r.pose.orientation.y = quaternion[1]
                pose_stamped_r.pose.orientation.z = quaternion[2]
                pose_stamped_r.pose.orientation.w = quaternion[3]
                path_r.poses.append(pose_stamped_r)
                # if t == self.dmp.cs.timesteps - 1:
                #     self.new_pose= pos
                # #     print('pos', self.new_pose)
                self.new_pose= pos
                self.publish_tf(pos, vel, acc)

              
                self.robot_path_pub_r.publish(path_r)    
                # self.publish_target_tf(self.pose_array)        
                #rospy.sleep(0.1)
                #rospy.sleep(0.02)
                rospy.sleep(0.006)
    
    def publish_target_tf(self,new_goal_position):
        tf_broadcaster = TransformBroadcaster()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = "target_left"

        transform.transform.translation.x = new_goal_position[0]
        transform.transform.translation.y = new_goal_position[1]
        transform.transform.translation.z = new_goal_position[2]


        rotationRightHandToEE2 = tf.transformations.quaternion_about_axis(-np.pi, (1, 0, 0))
        rotationRightHandToEE = tf.transformations.quaternion_about_axis(np.pi, (0, -1, 0))
        rotationRightHandToEE3 = tf.transformations.quaternion_about_axis(-np.pi, (1, 0, 0))

        quaternion = tf.transformations.quaternion_from_euler(new_goal_position[3], new_goal_position[4], new_goal_position[5])
        # existing_rotation = [
        #     new_goal_position['orientation']['x'],
        #     new_goal_position['orientation']['y'],
        #     new_goal_position['orientation']['z'],
        #     new_goal_position['orientation']['w']
        # ]

        transformed_rotation = tf.transformations.quaternion_multiply(quaternion, rotationRightHandToEE)
        transformed_rotation = tf.transformations.quaternion_multiply(transformed_rotation, rotationRightHandToEE2)
        # transformed_rotation = tf.transformations.quaternion_multiply(transformed_rotation, rotationRightHandToEE3)

        transform.transform.rotation.x = transformed_rotation[0]
        transform.transform.rotation.y = transformed_rotation[1]
        transform.transform.rotation.z = transformed_rotation[2]
        transform.transform.rotation.w = transformed_rotation[3]

        tf_broadcaster.sendTransform(transform)
        

    def load_weights_from_yaml(self, yaml_filename):
        try:
            with open(yaml_filename, 'r') as yaml_file:
                data = yaml.safe_load(yaml_file)

                x_rdata = data.get('x_r', [])
                y_rdata = data.get('y_r', [])
                z_rdata = data.get('z_r', [])
                r_rdata = data.get('r_r', [])
                p_rdata = data.get('p_r', [])
                ya_rdata = data.get('ya_r', [])
                
                x_rarray = np.array(x_rdata)
                y_rarray = np.array(y_rdata)
                z_rarray = np.array(z_rdata)
                r_rarray = np.array(r_rdata)
                p_rarray = np.array(p_rdata)
                ya_rarray = np.array(ya_rdata)
            
                                
                x_ldata = data.get('x_l', [])
                y_ldata = data.get('y_l', [])
                z_ldata = data.get('z_l', [])
                r_ldata = data.get('y_l', [])
                p_ldata = data.get('x_l', [])
                ya_ldata = data.get('ya_l', [])
                
                x_larray = np.array(x_ldata)
                y_larray = np.array(y_ldata)
                z_larray = np.array(z_ldata)
                r_larray = np.array(r_ldata)
                p_larray = np.array(p_ldata)
                ya_larray = np.array(ya_ldata)

                """ print('pos', x_larray.shape)
                print('pos', x_larray)
                input('x_rarray ') """
                
                if x_larray!=[]:
                    weights = np.vstack((x_larray, y_larray,z_larray,r_larray,p_larray,ya_larray,x_rarray, y_rarray,z_rarray,r_rarray,p_rarray,ya_rarray))
                
                else:
                    weights = np.vstack((x_rarray, y_rarray,z_rarray,r_rarray,p_rarray,ya_rarray))
                print('weights',weights)
                
                return weights

            
        except (FileNotFoundError, yaml.YAMLError):
            
            print("Error loading weights from YAML file.")
            
            
            
            
if __name__ == '__main__':
    try:
        DmpLoad()
    except rospy.ROSInterruptException:
        pass
