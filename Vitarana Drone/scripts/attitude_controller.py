#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import math
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to the current orientation of eDrone in quaternion format. This value is updated each time in imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [p,r,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [p_setpoint, r_setpoint, y_setpoint,throttle]
        self.setpoint_cmd = [1500.0, 1500.0, 1500.0, 1490.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
     
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp,Kd and ki for [roll, pitch, yaw, throttle]. 
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [0.60, 0.60, 0, 0.0]
        self.Ki = [0.020, 0.020, 0.0, 0.00]
        self.Kd = [30, 30, 0, 0]
    
        # Errors in each axis
        self.error = [0,0,0]
    
        #Errors in pid terms
        self.error_propor =[0,0,0]
        self.error_sum =[0,0,0]
        self.error_diff =[0,0,0]



        # Previous errors in each axis
        self.prev_error = [0,0,0] 
    
        # Limtiting the pwm values of prop
        self.min_values = [0, 0, 0, 0] 
        self.max_values = [1024, 1024, 1024, 1024] 

        # initialise variable prev_time
		
		



	self.prev_time = time.time()

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.roll_error = rospy.Publisher('/roll_error',Float32, queue_size=1)
        self.pitch_error = rospy.Publisher('/pitch_error',Float32, queue_size=1)
        self.yaw_error = rospy.Publisher('/yaw_error',Float32, queue_size=1)
    
        # Subscribing to /drone_command, imu/data
        rospy.Subscriber('/edrone/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        

    # Imu callback function( gets executed each time when imu publishes /edrone/imu/data)
    def imu_callback(self,msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w



    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcPitch
        self.setpoint_cmd[1] = msg.rcRoll
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_cmd[3] = msg.rcThrottle

    

    def pid(self):
        
        # Converting quaternion to euler angles
       
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], 			self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll ,pitch and yaw axis
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
        
        

        # Converting the range of 1000 to 2000 to 0 to 1024 for throttle 
        self.out_throttle = self.setpoint_cmd[3]* 1.024 - 1024

        

        # Computing error in each axis	
        self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0] * 180/math.pi
        self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1] * 180/math.pi
        self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2] * 180/math.pi

        
        # current time
    	curr_time = time.time()

    	# variable that holds the time lapse
    	dt = curr_time - self.prev_time

    	# if time lapse less than the sample tiime ; exit
	if (dt< self.sample_time):
		    return

	# else ; compute pid output
	else:
		    # update previous time
		    self.prev_time = curr_time

                    # Computing the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis.
                    # Error( for proportional)
                    self.error_propor[0] = self.Kp[0] * self.error[0]
                    self.error_propor[1] = self.Kp[1] * self.error[1]
                    self.error_propor[2] = self.Kp[2] * self.error[2]

                    
                    #Change in Error (for derivative)
                    self.error_diff[0] = self.Kd[0] * (self.error [0] -self.prev_error[0] )
                    self.error_diff[1] = self.Kd[1] * (self.error [1] -self.prev_error[1] )
                    self.error_diff[2] = self.Kd[2] * (self.error [2] -self.prev_error[2] )

                    #rospy.loginfo(self.error_diff)

                    #Sum of errors (for integral)
                    self.error_sum[0] += self.Ki[0]* self.error[0]
                    self.error_sum[1] += self.Ki[1]* self.error[1]
                    self.error_sum[2] += self.Ki[2]* self.error[2]

                    
                    for x in range(3):                          
                        if self.error_sum[x] > self.max_values[x]:
		                    self.error_sum[x] = self.max_values[x]
                        elif self.error_sum[x] < self.min_values[x]:
	 	                    self.error_sum[x] = self.min_values[x]

                    
    
                    #Computing pid output for each axis
                    self.out_pitch  = self.error_propor[0] + self.error_diff[0] + self.error_sum[0]
                    self.out_roll = self.error_propor[1] + self.error_diff[1] + self.error_sum[1]
                    self.out_yaw   = self.error_propor[2] + self.error_diff[2] + self.error_sum[2]

                    
                    
                    #Computing PWM for propellers
                    self.pwm_cmd.prop1 = self.out_throttle - self.out_roll + self.out_pitch + self.out_yaw
                    self.pwm_cmd.prop2 = self.out_throttle - self.out_roll - self.out_pitch - self.out_yaw
                    self.pwm_cmd.prop3 = self.out_throttle + self.out_roll - self.out_pitch + self.out_yaw
                    self.pwm_cmd.prop4 = self.out_throttle + self.out_roll + self.out_pitch - self.out_yaw

                    
                    
    
                    #Limit the output
                    if self.pwm_cmd.prop1 > self.max_values[0]:
                        self.pwm_cmd.prop1 = self.max_values[0]		
                    elif self.pwm_cmd.prop1 < self.min_values[0]:
                        self.pwm_cmd.prop1 = self.min_values[0]

                    if self.pwm_cmd.prop2 > self.max_values[1]:
                        self.pwm_cmd.prop2 = self.max_values[1]		
                    elif self.pwm_cmd.prop2 < self.min_values[1]:
                        self.pwm_cmd.prop2 = self.min_values[1] 		
     
                    if self.pwm_cmd.prop3 > self.max_values[2]:
                        self.pwm_cmd.prop3 = self.max_values[2]		
                    elif self.pwm_cmd.prop3 < self.min_values[2]:
                        self.pwm_cmd.prop3 = self.min_values[2]

                    if self.pwm_cmd.prop4 > self.max_values[3]:
                        self.pwm_cmd.prop4 = self.max_values[3]		
                    elif self.pwm_cmd.prop4 < self.min_values[3]:
                        self.pwm_cmd.prop4 = self.min_values[3]

                    
                    # Updating the previous errors
                    self.prev_error[0] = self.error[0]
                    self.prev_error[1] = self.error[1]
                    self.prev_error[2] = self.error[2]

                    


                    
                    self.roll_error.publish(self.error[1])
                    self.pitch_error.publish(self.error[0])
                    self.yaw_error.publish(self.error[2])
                    self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(20)  
    while not rospy.is_shutdown():
        try:
            e_drone.pid()
            r.sleep()
        except:
            rospy.exceptions.ROSTimeMovedBackwordsException(time_jump)
        pass