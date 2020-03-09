#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

import LS7366R

from time import sleep

T2R = 0 #tick to radian. needs manual calibration.

def vehicle_model(x,y,theta,other_parameters):
    """
    Vehicle motion model.
    
    returns the pose of the vehicle (x, y, theta).
    
    """

def odometry():
    """
    This node reads the encoder ticks, evaluate the odometry using the
    motion model and publishes the pose using TF.
    
    ** LS7366R usage**
        create an object by calling enc = LS7366R(CSX, CLK, BTMD) where:
        CSX is either CE0 or CE1, 
        CLK is the speed, 
        BTMD is the bytemode 1-4 the resolution of your counter
        
    """
    
    rospy.init_node("node_node", anonymous=True)
    pub = rospy.Publisher("current_speed", Float32, queue_size=1) 
    
    encoder = LS7366R.LS7366R(0, 3900000, 4)
    cur_encoder_position = 0
    pre_encoder_position = 0
    motor_speed = 0.0
    
    node_freq = 10.0
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        rospy.loginfo("Tick counter %i", encoder.readCounter())
                        
        pre_encoder_position = cur_encoder_position
        cur_encoder_position = encoder.readCounter()
        motor_speed = abs((cur_encoder_position - pre_encoder_position) / ( 1.0/node_freq))
        #rospy.loginfo("Current speed %f", motor_speed)
        
        pub.publish(motor_speed)
        
        rate.sleep()
        
    rospy.loginfo("Shutting down encoder node")

if __name__ == '__main__':
    odometry()
    


