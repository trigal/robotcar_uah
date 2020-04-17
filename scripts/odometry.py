#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import math

import LS7366R

from time import sleep

WT2M = 0.000232599 # (W)heels   tick to radian. needs manual calibration.
ST2R = 25.0 / 400.0 # (S)teering tick to radian. needs manual calibration.
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
L = 0.14 #meters

current_angle = 0.0
motor_speed   = 0.0

def vehicle_model(steering_angle, speed, dt):
    """
    Vehicle motion model.
        
    """
    
    x_ = speed * math.sin (math.pi / 2.0 - steering_angle) * dt
    y_ = speed * math.cos (math.pi / 2.0 - steering_angle) * dt
    th_= ((speed * dt) / L) * math.tan (steering_angle) 
    
    return [x_, y_, th_]

def steering_callback(angle_msg):
    """
    Receive the angle from the steering node (Float32)
    """
    global current_angle
    
    current_angle = angle_msg.data * ST2R * DEG2RAD
    #rospy.loginfo("steering callback %f", current_angle)

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
    pub_vel = rospy.Publisher("current_speed", Float32, queue_size=1) 
    pub_ticks = rospy.Publisher("ticks", Int32, queue_size=1) 
    rospy.Subscriber("current_servo", Float32, steering_callback)
    broadcaster = tf.TransformBroadcaster()
    
    encoder = LS7366R.LS7366R(0, 3900000, 4)
    cur_encoder_position = 0
    pre_encoder_position = 0
    cur_angle = 0
    pre_angle = 0
    
    x = 0.
    y = 0.
    th= 0.
    
    node_freq = 10.0
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        #rospy.loginfo("Tick counter %i", encoder.readCounter())
                        
        pre_encoder_position = cur_encoder_position
        cur_encoder_position = encoder.readCounter()
        motor_speed = -(((cur_encoder_position - pre_encoder_position) * WT2M) / ( 1.0/node_freq))
                
        pre_angle = cur_angle
        cur_angle = current_angle
        steering_speed = (cur_angle - pre_angle) / (1.0/node_freq)
        
        [delta_x, delta_y, delta_theta] = vehicle_model(current_angle, motor_speed, 1.0/node_freq)
        th=th+delta_theta
        x=x+math.cos(th)*delta_x-math.sin(th)*delta_y
        y=y+math.sin(th)*delta_x+math.cos(th)*delta_y
        
        
        broadcaster.sendTransform( (x,y,0), tf.transformations.quaternion_from_euler(0,0,th), rospy.Time.now(), "vehicle_frame","odom")
        broadcaster.sendTransform( (L,-.05,0), tf.transformations.quaternion_from_euler(0,0,current_angle), rospy.Time.now(), "left_wheel","vehicle_frame")
        broadcaster.sendTransform( (L, .05,0), tf.transformations.quaternion_from_euler(0,0,current_angle), rospy.Time.now(), "right_wheel","vehicle_frame")
        
        rospy.loginfo("Current speed %f m/s [%f km/h] -- yaw rate %f [rad/s]", motor_speed, motor_speed * 3.6, steering_speed)
        
        pub_vel.publish(motor_speed)
        pub_ticks.publish(cur_encoder_position)
        
        rate.sleep()
        
    rospy.loginfo("Shutting down encoder node")

if __name__ == '__main__':
    odometry()
    
