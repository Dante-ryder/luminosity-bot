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
import numpy as np
from sensor_msgs.msg import Image
from luminosity_drone.msg import Biolocation
import cv2
import math
from cv_bridge import CvBridge
from swift_msgs.msg import *
from std_msgs.msg import *
from imutils import contours
from skimage import measure
import numpy as np
import imutils
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64



class swift:
    """docstring for swift"""
    def __init__(self):
        #initializing the node with the name 'life_form_detector'
        rospy.init_node('life_form_detection', anonymous = True)

        #set initial drone position as 0,0,0 and will be updated each time from the whycon callback function
        self.drone_position = [0.0, 0.0, 0.0]

        #set points are set manually to lift the drone high and land it 
        self.setpoint = [0.0, 0.0, 22.0]
        self.visitedsetpoints = [[]]

# , [0, 2, 20], [0, 4, 20], [0, -2, 20], [0, -4, 20], [2, 0, 20], [2, 2, 20], 
#                          [2, 4, 20], [2, -2, 20], [2, -4, 20], [4, 0, 20], [4, 2, 20], [4, 4, 20], [4, -2, 20], 
#                          [4, -4, 20], [-2, 0, 20], [-2, 2, 20], [-2, 4, 20], [-2, -2, 20], [-2, -4, 20], 
#                          [-4, 0, 20], [-4, 2, 20], [-4, 4, 20], [-4, -2, 20], [-4, -4, 20]
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
        self.Kp = [9.3, 15.3, 11.179991999999199]
        self.Ki = [0, 0, 0.04800401758]
        self.Kd = [10, 20, 468.87479]

        #errors for pid
        self.alt_error = [0.0, 0.0, 0.0]
        self.prev_alt_error = [0.0, 0.0, 0.0]
        self.sum_alt_error = [0.0, 0.0, 0.0]

        #min and max values for throttle
        self.min = 1000
        self.max = 2000
        self.helper = 0
        self.opt = 0

        # pub;ish /drone_command and errors for analytics purpose /alt_error, /pitch_error and /roll_error
        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size = 1)
        self.location_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size = 1)
        self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size = 1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size = 1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size = 1)

        self.loc = Biolocation()
        self.bridge = CvBridge()
        self.detected_led_center=None
        self.img=Image
        

        # Subscribing to /whycon/poses, (/pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll)=> to tune pid
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)

        # rospy.Subscriber('/swift/camera_rgb/image_raw', PoseArray,self.altitude_set_pid)
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
        if self.alt_error[0] < 0.2 and self.alt_error[0] > -0.2 and self.alt_error[1] < 0.2 and self.alt_error[1] > -0.2 and self.alt_error[2] < 0.2 and self.alt_error[2] > -0.2:
            self.helper+=1
            if(self.helper==5):
                self.helper=0
                if self.opt >=4:
                    self.opt=0
                rospy.Subscriber('/swift/camera_rgb/image_raw', Image, self.detect_life_form)
                self.opt+=1  #navigate to the next setpoint
                self.changeposition()
                
        # rospy.loginfo("<------####------.")
        # rospy.loginfo("Throttle : %s",str(self.cmd.rcThrottle))
        # rospy.loginfo("Error : %s",str(self.alt_error[2]))
        # rospy.loginfo("Kp : %s",str(self.Kp[2]))
        # rospy.loginfo("Ki : %s",str(self.Ki[2]))
        # rospy.loginfo("Kd : %s",str(self.Kd[2]))
        # rospy.loginfo("<------####------.")

		# for altitude
        self.alt_error[2] = - (self.setpoint[2] - self.drone_position[2])
        self.cmd.rcThrottle = 1550 + int((self.alt_error[2] * self.Kp[2]) + ((self.alt_error[2] - self.prev_alt_error[2]) * self.Kd[2]) + (self.sum_alt_error[2] * self.Ki[2]))
        if (self.cmd.rcThrottle > 2000):
            self.cmd.rcThrottle = 2000
        if (self.cmd.rcThrottle < 1000):
            self.cmd.rcThrottle = 1000
        self.pitch_error_pub.publish(self.cmd.rcThrottle)
        self.prev_alt_error[2] = self.alt_error[2]
        self.sum_alt_error[2] += self.alt_error[2]

        # rospy.loginfo("<------####------.")
        # rospy.loginfo("pitch : %s",str(self.cmd.rcPitch))
        # rospy.loginfo("Error : %s",str(self.alt_error[1]))
        # rospy.loginfo("Kp : %s",str(self.Kp[1]))
        # rospy.loginfo("Ki : %s",str(self.Ki[1]))
        # rospy.loginfo("Kd : %s",str(self.Kd[1]))
        # rospy.loginfo("<------####------.")

		# for pitch
        self.alt_error[1] =  - (self.setpoint[1] - self.drone_position[1])
        self.cmd.rcPitch = 1500 + int((self.alt_error[1] * self.Kp[1]) + ((self.alt_error[1] - self.prev_alt_error[1]) * self.Kd[1]) + (self.sum_alt_error[1] * self.Ki[1]))
        if (self.cmd.rcPitch > 2000):
            self.cmd.rcPitch = 2000
        if (self.cmd.rcPitch < 1000):
            self.cmd.rcPitch = 1000
        self.pitch_error_pub.publish(self.cmd.rcPitch)
        self.prev_alt_error[1] = self.alt_error[1]
        self.sum_alt_error[1] += self.alt_error[1]

        # rospy.loginfo("<------####------.")
        # rospy.loginfo("Roll : %s",str(self.cmd.rcRoll))
        # rospy.loginfo("Error : %s",str(self.alt_error[0]))
        # rospy.loginfo("Kp : %s",str(self.Kp[0]))
        # rospy.loginfo("Ki : %s",str(self.Ki[0]))
        # rospy.loginfo("Kd : %s",str(self.Kd[0]))
        # rospy.loginfo("<------####------.")

        # for roll
        self.alt_error[0] =  (self.setpoint[0] - self.drone_position[0])
        self.cmd.rcRoll = 1500 + int((self.alt_error[0] * self.Kp[0]) + ((self.alt_error[0] - self.prev_alt_error[0]) * self.Kd[0]) + (self.sum_alt_error[0] * self.Ki[0]))
        if (self.cmd.rcRoll > 2000):
            self.cmd.rcRoll = 2000
        if (self.cmd.rcRoll < 1000):
            self.cmd.rcRoll = 1000
        self.pitch_error_pub.publish(self.cmd.rcRoll)
        self.prev_alt_error[0] = self.alt_error[0]
        self.sum_alt_error[0] += self.alt_error[0]
        self.command_pub.publish(self.cmd)
        self.loc.organism_type = "A"
        self.loc.whycon_x = 0
        self.loc.whycon_y = 1
        self.loc.whycon_z = 0
        self.location_pub.publish(self.loc)


    def changeposition(self):
        temp = []
        for i in self.setpoint:
            temp.append(i)
        self.visitedsetpoints.append(self.setpoint) 
        if self.opt ==1:
            temp[0] += 2.25
            if temp in self.visitedsetpoints:
                temp[0] -= 2.25
        elif self.opt == 2:
            temp[1] += 2.25
            if temp in self.visitedsetpoints:
                temp[1] -= 2.25
        elif self.opt == 3:
            temp[0] -= 2.25
            if temp in self.visitedsetpoints:
                temp[0] += 2.25
        elif self.opt == 4:
            temp[1] -= 2.25
            if temp in self.visitedsetpoints:
                temp[1] += 2.25
        print(f"temp : {temp}")
        while temp in self.visitedsetpoints:
            if self.opt <=1:
                self.opt =5
            self.opt -=1
            print(self.opt)
            if self.opt ==1:
                temp[0] += 2.25
                if temp in self.visitedsetpoints:
                    temp[0] -= 2.25
            elif self.opt == 2:
                temp[1] += 2.25
                if temp in self.visitedsetpoints:
                    temp[1] -= 2.25
            elif self.opt == 3:
                temp[0] -= 2.25
                if temp in self.visitedsetpoints:
                    temp[0] += 2.25
            elif self.opt == 4:
                temp[1] -= 2.25
                if temp in self.visitedsetpoints:
                    temp[1] += 2.25
            if temp[0] >= 6 and temp[0] <= -6 and temp[1] >= 6 and temp[1] <=-6:
                print('hi')
                if self.opt >=4:
                    self.opt-=1
                else:
                    self.opt+=1
            print(f"temp final : {temp}")
        
        self.setpoint=temp
        

    def detect_life_form(self, img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Apply a binary threshold
        except Exception as e:
            print(e)
            return
        _, binary_image = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY)

            # Find contours in the binary image
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter contours based on area (assuming LEDs are small and bright spots)
        min_contour_area = 50  # Adjust this threshold based on your specific case
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

            # Draw contours on the original image
        cv2.drawContours(cv_image, filtered_contours, -1, (0, 255, 0), 2)

        led_locations = []
        for cnt in filtered_contours:
            # Get the centroid of the contour
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                led_locations.append((cx, cy))

            # Draw contours on the original image
            cv2.drawContours(cv_image, [cnt], -1, (0, 255, 0), 2)

        # Display the result
        # cv2.imshow("Processed Image", cv_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # Print LED locations
        if led_locations:
            print("Number of white LEDs:", len(led_locations))
            print("LED locations:", led_locations)
        else:
            print("No white LEDs found.")




if __name__=='__main__':
    swift_drone = swift()
    r = rospy.Rate(30)
    # Controller started
    while not rospy.is_shutdown():
        swift_drone.pid()
        r.sleep()
    print(swift_drone.visitedsetpoints)

    # controller ended
