#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import tkinter as tk
from std_msgs.msg import String

class PlayPauseButton:
    def __init__(self):
        self.root = tk.Tk()
        self.button_names_left = ["Play", "Pause", "Enter", "Execute_1", "Execute_2", "Execute_3", "Execute_4", "Execute_5", "Weights_1", "Weights_2", "Weights_3", "Weights_4", "Weights_5", "Weights_6", "Restart"]
        self.button_names_right = ["Execute_1_2", "Execute_2_2", "Execute_3_2", "Execute_4_2", "Execute_5_2","Ready_1","Ready_2","Ready_3","Ready_4"]

        self.pub = rospy.Publisher("play_pause_signal", String, queue_size=1)
        self.status = "0"
        self.publish_status()

        self.create_buttons()

    def create_buttons(self):
        for button_name in self.button_names_left:
            button = tk.Button(self.root, text=button_name, command=lambda name=button_name: self.publish_action(name))
            button.grid(row=self.button_names_left.index(button_name), column=0)

        for button_name in self.button_names_right:
            button = tk.Button(self.root, text=button_name, command=lambda name=button_name: self.publish_action(name))
            button.grid(row=self.button_names_right.index(button_name), column=1)

    def publish_action(self, action_name):
        self.status = action_name.lower()
        self.pub.publish(String(data=self.status))
        self.status = "0"  
    def publish_status(self):
        msg = String(data=self.status)
        self.pub.publish(msg)
        self.root.after(100, self.publish_status)  

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    rospy.init_node("play_pause_button_node")
    button_gui = PlayPauseButton()
    button_gui.run()
