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

# Importing the required libraries

import queue
from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control', anonymous = True)	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		# self.setpoint = [2,2,20] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoints = [[0, 0, 23], [2, 0, 23], [2, 2, 23], [2, 2, 25], [-5, 2, 25], [-5, -3, 25], [-5, -3, 21], [7, -3, 21], [7, 0, 21], [0, 0, 19]]
		self.pos = -1


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
		
		
		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.alt_error = [0.0,0.0,0.0]
		self.prev_alt_error = [0.0,0.0,0.0]
		self.sum_alt_error = [0.0,0.0,0.0]


		self.min_value = 1000
		self.max_value = 2000




		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)

		# self.command_pub.publish(self.cmd)

	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------


		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)


		#------------------------------------------------------------------------------------------------------------
		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		# rospy.loginfo("<------####------.")
		# rospy.loginfo(msg.poses[0].position.x)
		# rospy.loginfo("<------####------.")
		# rospy.loginfo(msg.poses[0].position.y)
		# rospy.loginfo("<------####------.")
		# rospy.loginfo(msg.poses[0].position.z)
		# rospy.loginfo("<------####------.")
	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.0008
		self.Kd[2] = alt.Kd * 0.3
		# rospy.loginfo("<------####Altitude####------>")
		# rospy.loginfo(alt)
		# rospy.loginfo("<------####------>")
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pitch.Ki * 0.0008
		self.Kd[1] = pitch.Kd * 0.3
		# rospy.loginfo("<------####------>")
		# rospy.loginfo(pitch)
		# rospy.loginfo("<------####------>")


	def roll_set_pid(self, roll):
		self.Kp[0] = roll.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = roll.Ki * 0.0008
		self.Kd[0] = roll.Kd * 0.3
		# rospy.loginfo("<------####------>")
		# rospy.loginfo(roll)
		# rospy.loginfo("<------####------>")






	#----------------------------------------------------------------------------------------------------------------------

	def pid(self, setpoint):
		if self.alt_error[0] < 0.175 and self.alt_error[0] > -0.175 and self.alt_error[1] < 0.175 and self.alt_error[1] > -0.175 and self.alt_error[2] < 0.175 and self.alt_error[2] > -0.175:
			self.pos+=1
		# for altitude
		self.alt_error[2] = - (setpoint[2] - self.drone_position[2])
		self.cmd.rcThrottle = 1550 + int((self.alt_error[2] * self.Kp[2]) + ((self.alt_error[2] - self.prev_alt_error[2]) * self.Kd[2]) + (self.sum_alt_error[2] * self.Ki[2]))
		if(self.alt_error[2] > 0.1 or self.alt_error[2]<-0.2):
			rospy.loginfo("<------####------>")
			rospy.loginfo("Throttle : %s",str(self.cmd.rcThrottle))
			rospy.loginfo("Error : %s",str(self.alt_error[2]))
			rospy.loginfo("Kp : %s",str(self.Kp[2]))
			rospy.loginfo("Ki : %s",str(self.Ki[2]))
			rospy.loginfo("Kd : %s",str(self.Kd[2]))
			rospy.loginfo("<------####------>")
		if (self.cmd.rcThrottle > 2000):
			self.cmd.rcThrottle = 2000
		if (self.cmd.rcThrottle < 1000):
			self.cmd.rcThrottle = 1000
		self.alt_error_pub.publish(self.cmd.rcThrottle-1500)
		self.pitch_error_pub.publish(self.cmd.rcThrottle)
		self.prev_alt_error[2] = self.alt_error[2]
		self.sum_alt_error[2] += self.alt_error[2]

		# for pitch
		self.alt_error[1] = - (setpoint[1] - self.drone_position[1])
		self.cmd.rcPitch = 1500 + int((self.alt_error[1] * self.Kp[1]) - ((self.alt_error[1] - self.prev_alt_error[1]) * self.Kd[1])+ (self.sum_alt_error[1] * self.Ki[1]))
		if(self.alt_error[1] > .2 or self.alt_error[1]<-.2):
			rospy.loginfo("<------####------.")
			rospy.loginfo("pitch : %s",str(self.cmd.rcPitch))
			rospy.loginfo("Error : %s",str(self.alt_error[1]))
			rospy.loginfo("Kp : %s",str(self.Kp[1]))
			rospy.loginfo("Ki : %s",str(self.Ki[1]))
			rospy.loginfo("Kd : %s",str(self.Kd[1]))
			rospy.loginfo("<------####------.")
		if (self.cmd.rcPitch > 2000):
			self.cmd.rcPitch = 2000
		if (self.cmd.rcPitch < 1000):
			self.cmd.rcPitch = 1000
		self.prev_alt_error[1] = self.alt_error[1]
		self.sum_alt_error[1] += self.alt_error[1]

		# for roll
		self.alt_error[0] =  (setpoint[0] - self.drone_position[0])
		self.cmd.rcRoll = (1500 + int((self.alt_error[0] * self.Kp[0]) - ((self.alt_error[0] - self.prev_alt_error[0]) * self.Kd[0])+ (self.sum_alt_error[0] * self.Ki[0])))
		if(self.alt_error[0] > .2 or self.alt_error[0]<-.2):
			rospy.loginfo("<------####------>")
			rospy.loginfo("Roll : %s",str(self.cmd.rcRoll))
			rospy.loginfo("Error : %s",str(self.alt_error[0]))
			rospy.loginfo("Kp : %s",str(self.Kp[0]))
			rospy.loginfo("Ki : %s",str(self.Ki[0]))
			rospy.loginfo("Kd : %s",str(self.Kd[0]))
			rospy.loginfo("<------####------>")
		if (self.cmd.rcRoll > 2000):
			self.cmd.rcRoll = 2000
		if (self.cmd.rcRoll < 1000):
			self.cmd.rcRoll = 1000
		self.prev_alt_error[0] = self.alt_error[0]
		self.sum_alt_error[0] += self.alt_error[0]

		self.command_pub.publish(self.cmd)
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum



















	#------------------------------------------------------------------------------------------------------------------------
		







if __name__ == '__main__':
	swift_drone = swift()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	rospy.loginfo("<------####------>")
	rospy.loginfo("<-------PID Started------>")
	while not rospy.is_shutdown():
		# for i in swift_drone.setpoints:
		swift_drone.pid(swift_drone.setpoints[swift_drone.pos])
		rospy.loginfo("Pos : " + str(swift_drone.pos))
		r.sleep()
	rospy.loginfo("<------PID Ended------>")
	rospy.loginfo("<------####------>")