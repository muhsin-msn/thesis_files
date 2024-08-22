#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from dmp_int.srv import *
import tf
import yaml
from std_msgs.msg import String

class DMPClient:

    def __init__(self):
        rospy.init_node('execute_dmp_service_client')
        self.req = DMPLoadRequest()
        self.execute_signal = ""
        
        rospy.Subscriber("play_pause_signal", String, self.play_pause_callback)

        # List of execute signals
        self.execute_signals = [
            'execute_1',
            'execute_2',
            'execute_3',
            'execute_4',
            'execute_5_2',
        ]

    def play_pause_callback(self, msg):
        if msg.data in self.execute_signals:
            self.execute_signal = msg.data

    def reset_service(self):
        # Reset the service request
        self.req = DMPLoadRequest()

    def call_dmp_service(self, execute_signal):
        # Call the service when "execute" signal is received
        try:
            service_number = execute_signal
            service_client = rospy.ServiceProxy('/service_' + str(service_number), DMPLoad)
            rospy.loginfo(service_client(self.req))
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" % e)

    def run(self):
        rate = rospy.Rate(20)  
        while not rospy.is_shutdown():
            if self.execute_signal:
                self.call_dmp_service(self.execute_signal)
                self.reset_service()
                self.execute_signal = "" 
            rate.sleep()

if __name__ == "__main__":
    dmp_client = DMPClient()
    dmp_client.run()
