#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

'''
# Team ID:          < LD-1122 >
# Theme:            < LUMINOSITY DRONE >
# Author List:      < YUVARAJ MUNEESHWARAN C, SUKESH T N, MATHINRAJ R, MUKILAN J >
# Filename:         < life_form_detector.py >
# Functions:        < >
# Global variables: < >
'''


#Import all the required pakages
import rospy
from swift_msgs.msg import *
from std_msgs.msg import *
from pid_tune.msg import PidTune
import queue


class swift:
    """docstring for swift"""
    def __init__(self):
        #initializing the node with the name 'life_form_detector'
        rospy.init_node('life_form_detection', anonymous = True)

        #set initial drone position as 0,0,0 and will be updated each time from the whycon callback function
        self.drone_position = [0.0, 0.0, 0.0]

        #set points are set manually to lift the drone high and land it 
        self.setpoint = [0.0, 0.0, 0.0]

        #Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        #initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [5.3, 10.3, 11.179991999999199]
        self.Ki = [0, 0, 0.04800401758]
        self.Kd = [10, 20, 468.87479]

        #errors for pid
        self.alt_error = [0.0, 0.0, 0.0]
        self.prev_alt_error = [0.0, 0.0, 0.0]

        #min and max values for throttle
        self.min = 1000
        self.max = 2000

        # pub;ish /drone_command and errors for analytics purpose /alt_error, /pitch_error and /roll_error
        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size = 1)
        self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size = 1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size = 1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size = 1)

        # Subscribing to /whycon/poses, (/pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll)=> to tune pid
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)

        # rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
        # rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)

        self.arm()
    

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def arm(self):
        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX4 = 1500

        self.command_pub.publish(self.cmd)	# Publishing /drone_command
    
        rospy.sleep(1)

    #whycon_callback func to  update the drone's current position subscribed from /whycon/poses    
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    # pid controller for the drone
    def pid(self):
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1590
        self.cmd.rcAUX4 = 1500

        self.command_pub.publish(self.cmd)




if __name__=='__main__':
    swift_drone = swift()
    r = rospy.Rate(30)
    # Controller started
    while not rospy.is_shutdown():
        swift_drone.pid()
        r.sleep()
    # controller ended