#!/usr/bin/env python
# license removed for brevity
import rospy
from vitarana_drone.msg import prop_speed

    

def Revolve():

    
    pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=10)
    rospy.init_node('game', anonymous=True)
    
    
    V=prop_speed()
    V.prop1=10000
    V.prop2=10000
    V.prop3=10000
    V.prop4=10000
    
    i=0
    while not rospy.is_shutdown(): 
        
        #pub.publish(V)
        ++i
        if i>100:
            break

    V.prop1=0000
    V.prop2=0000
    V.prop3=0000
    V.prop4=0000

    pub.publish(V)
        
        
        
    
        
        
if __name__ == '__main__':
    try:
        Revolve()
    except rospy.ROSInterruptException:
        pass




 self.I += self.Ki[0]* error[0]
 	    if self.I > self.max_values[0]:
		    self.I = self.max_values[0] 
	    else if pid_mem_roll < self.min_values[0] :
	 	    self.I = self.min_values[0]

	    roll = self.Kp[0] * error[0] + self.I + self.Kd[0]* (error[0] - self.prev_error[0])
        
        if roll > self.max_values[0] :
		    roll - self.max_values[0]
	    elif roll < self.min_values[0] :
		    roll = self.min_values[0]
	
	    
	
	    #pitch calculations
	    pid_i_pitch += self.Ki[1]* error[1]
	    if pid_i_pitch > self.max_values[1] :
		    pid_i_pitch = self.max_values[1]
	    elif pid_i_pitch < self.min_values[1] :
		    pid_i_pitch = self.min_values[1]

	    pitch = self.Kp[1] * error[1] + pid_i_pitch + self.Kd[1] * (error[1] - self.prev_error[1])
	
	    if pitch > self.max_values[1] :
		    pitch= self.max_values[1] 
	    elif pitch < self.min_values[1] : 
		    pitch = self.min_values[1]

       

	    #Yaw calculations
	    pid_i_yaw_+= self.Ki[2] * error[2] 
	    if pid_i_yaw > self.max_values[2] :
		    pid_i_yaw = self.max_values[2]
	    elif pid_i_yaw < self.min_values[2] :
    		pid_i_yaw = self.min_values[2]

	    yaw = self.Kp[2] * error[2] + pid_i_yaw + self.Kd[2] * (error[2] -self.prev_error[2])
	    
        if yaw > pid_max_yaw :
		    yaw = pid max_yaw
	    elif yaw < pid_max_yaw * -1 :
		    yaw = pid_max_yaw * -1
	    
        
        self.prev_error[0] = error[0]
        self.prev_error[1] = error[1]
        self.prev_error[2] = error[2]
        self.pub_cmd[]=[roll, pitch, yaw]




cosLat = Math.cos(self.current_pos[0] * Math.PI / 180.0)
        sinLat = Math.sin(self.current_pos[0] * Math.PI / 180.0)
        cosLon = Math.cos(self.current_pos[1] * Math.PI / 180.0)
        sinLon = Math.sin(self.current_pos[1] * Math.PI / 180.0)
        rad = 6378137.0
        f = 1.0 / 298.257224
        C = 1.0 / Math.sqrt(cosLat * cosLat + (1 - f) * (1 - f) * sinLat * sinLat)
        S = (1.0 - f) * (1.0 - f) * C
        h = 0.31
        self.setpoint_position_xyz[0] = (rad * C + h) * cosLat * cosLon
        self.setpoint_position_xyz[1]= (rad * C + h) * cosLat * sinLon
        self.setpoint_position_xyz[2]= (rad * S + h) * sinLat



    # Callback function for /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[1] = roll.Kp  *0.0004
        self.Ki[1] = roll.Ki  *0.0002
        self.Kd[1] = roll.Kd  *0.006
    def pitch_set_pid(self, pitch):
        self.Kp[0] = pitch.Kp * 0.0004  
        self.Ki[0] = pitch.Ki * 0.0002
        self.Kd[0] = pitch.Kd * 0.006
    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 0.0002  
        self.Ki[2] = yaw.Ki * 0.001
        self.Kd[2] = yaw.Kd * 0.0002
    def altitude_set_pid(self, altitude):
        self.Kp[3] = altitude.Kp * 0.0002
        self.Ki[3] = altitude.Ki * 0.001
        self.Kd[3] = altitude.Kd * 0.0002
