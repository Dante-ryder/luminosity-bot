#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from luminosity_drone.msg import Biolocation
import cv2
import math
from swift_msgs.msg import *
from cv_bridge import CvBridge  
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
        self.Kp = [7.3, 13.3, 11.179991999999199]
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
        #LED DETECTION
        self.detected_led_center=None
        self.img=Image
        self.bridge=CvBridge()





        # pub;ish /drone_command and errors for analytics purpose /alt_error, /pitch_error and /roll_error
        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size = 1)
        self.location_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size = 1)
        self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size = 1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size = 1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size = 1)

        self.loc = Biolocation()
        

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


    def detect_life_form(self,img):
        try:
            # Convert the ROS Image message to OpenCV format
            # cv_image = self.bridge.imgmsg_to_cv2(img, 'bgr8')
            cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

        except Exception as e:
            print(e)
            return

        # Perform image processing to detect the LED
        self.detected_led_center = self.detect_led(cv_image)
        if(self.detected_led_center==None):
            rospy.loginfo("Routing")
            return False
        else:
            rospy.loginfo("Detected......")
            adjust_drone_movement()
            return True
        # Adjust drone movement based on the detected LED position
        # self.adjust_drone_movement(detected_led_center)
    def detect_led(self,cv_image):

        image = cv_image.copy()

        # image = cv2.imread('image.png')
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (11, 11), 0)
        _, thresholded_image = cv2.threshold(blurred_image, 200, 255, cv2.THRESH_BINARY)


        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresholded_image, connectivity=8)

        large_components_mask = np.zeros(thresholded_image.shape, dtype=np.uint8)

        min_component_area = 100 

        for label in range(1, num_labels):
            if label == 0:
                continue

    # Construct the label mask for the current component
            label_mask = (labels == label).astype(np.uint8)

    # Count the number of pixels in the current component
            component_area = stats[label, cv2.CC_STAT_AREA]

    # Check if the component is "large" (greater than the threshold)
            if component_area > min_component_area:
        # Add the component to the mask of "large blobs"
                large_components_mask += label_mask * 255



        contours, _ = cv2.findContours(large_components_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        sorted_contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])

        centroid_list = []
        area_list = []
        circles = cv2.HoughCircles(gray_image, cv2.HOUGH_GRADIENT, 1, 20,param1=50, param2=30, minRadius=0, maxRadius=0)
        for i, contour in enumerate(sorted_contours):
    # Calculate the area of the contour
            area = cv2.contourArea(contour)
            radius=math.sqrt(area/math.pi)
    # Find the centroid of the contour
            M = cv2.moments(contour)
            centroid_x = M["m10"] / M["m00"]
            centroid_y = M["m01"] / M["m00"]
            # cv2.circle(image, (int(centroid_x), int(centroid_y)), int(radius), (0, 0, 255), 4)

            cv2.putText(image, "x1", (int(centroid_x) - 10, int(centroid_y) - 30), cv2.MARKER_CROSS, 0.5, (0, 0, 255), 2, cv2.LINE_8)

            # cv2.circle(large_components_mask, (int(centroid_x), int(centroid_y)), 5, 255, -1)

    # Append centroid coordinates and area to the respective lists
            centroid_list.append((centroid_x, centroid_y))
            area_list.append(area)


        # hsv_image=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        # mask=cv2.inRange(hsv_image,np.array([0,0,200]),np.array([255,50,255]))
        # contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # if contours:
        #    largest_contour=max(contours,key=cv2.contourArea)
        #    M=cv2.moments(largest_contour)
        #    if M["m00"] !=0:
        #     centroid_x=int(M["m10"]/M["m00"])     
        #     centroid_y=int(M["m01"]/M["m00"])  
        #     cv2.circle(image,(centroid_x,centroid_y),10,(0,255,0),-1)

        rospy.loginfo(f"No. of LEDs detected: {len(centroid_list)}\n")
        if(len(centroid_list)>=3):
            return (centroid_x,centroid_y)
        else:
            return None

    def adjust_drone_movement(self):
        if self.detected_led_center is not None:
            # Center of the image
            image_center_x = 320
            image_center_y = 240

            # Calculate the difference between the detected LED position and the image center
            delta_x = self.detected_led_center[0] - image_center_x
            delta_y = self.detected_led_center[1] - image_center_y

            # Adjust drone movement commands based on the difference
            self.cmd.rcThrottle = 1550 + int(0.1 * delta_y)  # Adjust throttle based on the y difference

            # Publish the adjusted movement commands
            self.command_pub.publish(self.cmd)
        else:
            # Stop the drone if the LED is not detected
            self.cmd.rcThrottle = 1500
            self.command_pub.publish(self.cmd)
if __name__=='__main__':
    swift_drone = swift()
    r = rospy.Rate(30)
    # Controller started
    while not rospy.is_shutdown():
        swift_drone.pid()
        # if swift_drone.detect_life_form():
        #     rospy.loginfo("Found!!!!! Rerouting")
        #     swift_drone.adjust_drone_movement()
        # else:

  
        r.sleep()
    print(swift_drone.visitedsetpoints)

    # controller ended
