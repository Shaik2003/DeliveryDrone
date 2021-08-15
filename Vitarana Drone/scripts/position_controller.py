#!/usr/bin/env python

'''
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  

       
        # This is the setpoint that will be published to the drone_command in the range from 1000 to 2000
        self.pub_cmd = [0.0, 0.0, 0.0, 0, 0, 0, 0, 0]
        
        # pid output values. [latitude, longitude, altitude]
        self.out= [0.0, 0.0, 0.0]

        # setpoint of final destiantion
        self.setpoint_position_gps = [19.0007046575, 71.9998955286, 24.1599967919]
        
        # initial setting of Kp,Kd and ki for [latitude, longitude, altitude]
        self.Kp = [11700000, 11700000, 700.1]
        self.Ki = [1.2, 1.2, 3]
        self.Kd = [1570000000, 1570000000, 8250]
        self.P = [0, 0, 0]
        self.I = [0, 0, 0]
        self.D = [0, 0, 0]
        self.min_values = [1000, 1000, 1000, 1000]
        self.max_values = [2000, 2000, 2000, 2000] 
        self.max_valuesI = [500, 500, 2000, 2000]
        # variables current values of gps
        self.current_pos_gps = [0.0, 0.0, 0.0]
        self.error= [0.0, 0.0, 0.0]
        self.prev_error= [0.0, 0.0, 0.0]
        
        
        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.075  # in seconds

        # Publishing /edrone/drone_command
        self.cmd_pub = rospy.Publisher('/edrone/drone_command', edrone_cmd, queue_size=1)

	

        # Subscribing to /edrone/gps
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        
    

    def gps_callback(self, msg):                #callback for live location
        self.current_pos_gps[0] = msg.latitude
        self.current_pos_gps[1] = msg.longitude
        self.current_pos_gps[2] = msg.altitude
        

      
   
    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def latitude_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 120000  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 12000
        self.Kd[0] = roll.Kd * 2000000
    def longitude_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 500000  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * 100
        self.Kd[1] = pitch.Kd * 1000000
    def altitude_set_pid(self, altitude):
        self.Kp[2] = altitude.Kp * 0.1     #4760
        self.Ki[2] = altitude.Ki * 0.03       #852
        self.Kd[2] = altitude.Kd * 1     #4607

    def path(self):
			
	    #Only if error in each axis is less then only next point from pathpoint table is accesed
	    if abs(self.error[0])<0.000004517  and  abs(self.error[1])<0.0000047487  and  abs(self.error[2])<0.2 :
                if self.current_pos_gps[0] > (19.00 - 0.000004517)  and self.current_pos_gps[0] < 19.000004517  :
                    self.setpoint_position_gps =[19.0000451704, 72.00 , 3]
                else :
                    self.setpoint_position_gps =[19.0000451704 , 72.00 ,0.31]
        
    def pid(self):

        # Computing error in each axis
        for i in range(3):
            self.error[i] = self.setpoint_position_gps[i] - self.current_pos_gps[i]
        			
	    #pid
	    
        rospy.loginfo(self.current_pos_gps)
        
        for i in range(3):                  #calculating I part of pid for each roll pitch throttle
            self.I[i] += self.Ki[i]* self.error[i]
        
        
        
        for x in range(3):                          #limiting I value
            if self.I[x] > self.max_valuesI[x]:
		        self.I[x] = self.max_valuesI[x]
            elif self.I[x] < self.max_valuesI[x]*-1:
	 	        self.I[x] = self.max_valuesI[x] * -1

        for i in range(3):                          #calculating P part of pid for each roll pitch throttle
            self.P[i]= self.Kp[i] * self.error[i]

        for i in range(3):                          #calculating D part of pid for each roll pitch throttle
            self.D[i]= self.Kd[i]* (self.error[i] - self.prev_error[i])

        for i in range(3):
	        self.out[i] = self.P[i]  + self.D[i] + self.I[i]        #calculating PID
        
        for i in range(3):
            self.prev_error[i] = self.error[i]                     #Storing error
        
        self.pub_cmd[0]= 1500  
        self.pub_cmd[1]= 1500

        if abs(self.error[0])>0.000002517:               #First making the drone roll
            self.pub_cmd[0]= 1500  + self.out[0]
            self.pub_cmd[1]= 1500 

        elif abs(self.error[1])>0.000027487:         #And then making it pitch
            self.pub_cmd[0]= 1500 
            self.pub_cmd[1]= 1500 + self.out[1]


        self.pub_cmd[2]= 1500                       #Keeping the yaw value constant
        self.pub_cmd[3]= 1500 + self.out[2]                #Assigning throttle value
        
        
        for i in range(4):                          #Limiting the output
            if self.pub_cmd[i] > self.max_values[i] :
                self.pub_cmd[i] = self.max_values[i]
            elif self.pub_cmd[i] < self.min_values[i] :
                self.pub_cmd[i] = self.min_values[i]
        
        self.cmd_pub.publish(self.pub_cmd[0],self.pub_cmd[1],self.pub_cmd[2],self.pub_cmd[3],self.pub_cmd[4],self.pub_cmd[5],self.pub_cmd[6],self.pub_cmd[7])          #Publishing 
        
        #if abs(self.error[0])<0.000002017  and  abs(self.error[1])<0.0000047487  and  abs(self.error[2])<0.1 :
            #self.path()
if __name__ == '__main__':

    e_drone = Edrone()
    #rospy.sleep(5)
    r = rospy.Rate(20)  
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
        #e_drone.sample_time