#!/usr/bin/env python

import rospy
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import yaml

class TFTrimmerNode(object):
    def __init__(self):
        rospy.init_node('tf_trimmer_node')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.playing = False  
        self.recording = []  
        
    
        rospy.Subscriber("play_pause_signal", String, self.play_pause_callback)

    def play_pause_callback(self, msg):
        if msg.data == "play":
            self.playing = True
            rospy.loginfo("Recording started. Press 'pause' to stop recording.")
        elif msg.data == "pause":
            self.playing = False
            rospy.loginfo("Recording paused. Press 'play' to resume recording.")
            
            self.save_recorded_data()
      
            self.recording = []

    

    def save_recorded_data(self):
        if self.recording:
            
            formatted_tf_data = []
            for tf_msg in self.recording:
                formatted_tf_msg = {
                    'orientation': {
                        'w': tf_msg.transform.rotation.w,
                        'x': tf_msg.transform.rotation.x,
                        'y': tf_msg.transform.rotation.y,
                        'z': tf_msg.transform.rotation.z
                    },
                    'position': {
                        'x': tf_msg.transform.translation.x,
                        'y': tf_msg.transform.translation.y,
                        'z': tf_msg.transform.translation.z
                    },
                    'timestamp': {
                        'secs': tf_msg.header.stamp.secs,
                        'nsecs': tf_msg.header.stamp.nsecs
                    }
                }
                formatted_tf_data.append(formatted_tf_msg)
            
      
            try:
                with open("path/to/filedata/trimmed/moving/straight_1.yaml", 'w') as yaml_file:
                    yaml.dump(formatted_tf_data, yaml_file, default_flow_style=False)
                rospy.loginfo("Saved trimmed TF data to 'trimmed_tf_data.yaml'.")
            except Exception as e:
                rospy.logerr("Error saving YAML file: %s", str(e))
    
    def record_tf_data(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            try:
                if self.playing:
                  
                    tf_msg = self.tf_buffer.lookup_transform("world", "right_controller_2", rospy.Time(0))
                    
            
                    self.recording.append(tf_msg)
                    rospy.loginfo("TF detected from 'world' to 'end_effector'.")
                    
                rate.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

if __name__ == '__main__':
    try:
        tf_trimmer_node = TFTrimmerNode()
        rospy.loginfo("Press 'play' to start recording and 'pause' to stop recording.")
        tf_trimmer_node.record_tf_data()
    except rospy.ROSInterruptException:
        pass




