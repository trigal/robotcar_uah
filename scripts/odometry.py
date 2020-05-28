#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import math
import numpy as np

import LS7366R

from time import sleep

from robotcar_uah.srv import vehicleStatus, vehicleStatusResponse

WT2M = 0.000232599              # (W)heels   tick to meters. needs manual calibration.
ST2DEG = 25.0 / 400.0           # (S)teering tick to radian. needs manual calibration, ex. 25 DEGS / 400 TICKS
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
L = 0.14                        # Meters, distance between the front and rear axes
TICKS2RAD = 0.00738408600245
LR = L / 2.0
LF = L / 2.0

current_angle = 0.0
motor_speed   = 0.0

def vehicle_model(steering_angle, speed, dt):
    """
    Vehicle motion model.
        
    """
    
    x_ = speed * math.sin (math.pi / 2.0 - steering_angle) * dt
    y_ = speed * math.cos (math.pi / 2.0 - steering_angle) * dt
    th_= ((speed * dt) / L) * math.tan (steering_angle) 
 
    
    h0 = 0.0
    Beta = np.arctan((LR/(LR+LF)) * np.tan(steering_angle))
    x__ = speed * np.cos(h0+Beta)  * dt
    y__ = speed * np.sin(h0+Beta)  * dt 
    th__ = (speed/LR)* np.sin(Beta)* dt
    
    #rospy.loginfo("Motion Model, (steering_angle, speed, dt), %f %f %f", steering_angle, speed, dt)
    #rospy.loginfo("Motion Model, dx|dy|dth: %f %f %f", x_ , y_, th_)
    #rospy.loginfo("Motion Model, dx|dy|dth: %f %f %f", x__ , y__, th__)
    
    return [x__, y__, th__]    
    #return [x_, y_, th_]

def steering_callback(angle_msg):
    """
    Receive the angle from the steering node (Float32)
    """
    global current_angle
    
    current_angle = angle_msg.data * ST2DEG * DEG2RAD * 0.7    
    #rospy.loginfo("Received %f from steering sensor, evaluated steering angle is now: %f", angle_msg.data, current_angle)

def handle_vehicleStatus(req):
    """
    This service returns the last/current value in terms of speed and 
    steering value (please take care, this is the last value we set to 
    the servo, but there is no guarantee that the servo has already that
    position, maybe is still moving!
    
    The need of this service arises from the AMCL node, just to not
    change too much code.
    
    Provides the speed and the angle of the steering wheel. This service
    is inside here rather than in both dcmotor.py and steering.py to
    avoid calling two services from AMCL, here we have the integrated 
    data.
    
    """
    
    global current_angle, motor_speed
    
    rospy.loginfo("Requested status with moving average of %f" , req.average)
    rospy.loginfo("motor_speed   %f" , motor_speed)
    rospy.loginfo("current_angle %f" , current_angle)
    
    resp = vehicleStatusResponse()
    resp.header.stamp = rospy.Time.now()
    resp.header.frame_id = "vehicle_frame"
    resp.speed = motor_speed
    resp.steering = current_angle
    
    return resp

def odometry():
    """
    This node reads the encoder ticks, evaluate the odometry using the
    motion model and publishes the pose using TF.
    
    ** LS7366R usage**motor_speed
        create an object by calling enc = LS7366R(CSX, CLK, BTMD) where:
        CSX is either CE0 or CE1, 
        CLK is the speed, 
        BTMD is the bytemode 1-4 the resolution of your counter
        
    """
    
    global current_angle, motor_speed
    
    rospy.init_node("odometry_node", anonymous=True)
    pub_vel = rospy.Publisher("current_speed", Float32, queue_size=1) 
    pub_ticks = rospy.Publisher("ticks", Int32, queue_size=1) 
    pub_odometry = rospy.Publisher("odom", Odometry, queue_size=1)
    rospy.Subscriber("current_servo", Float32, steering_callback)
    broadcaster = tf.TransformBroadcaster()
    
    s = rospy.Service('/odometry_node/current_state', vehicleStatus, handle_vehicleStatus)
    
    encoder = LS7366R.LS7366R(0, 3900000, 4)
    cur_encoder_position = 0
    pre_encoder_position = 0
    cur_angle = 0
    pre_angle = 0
    
    x = 0.
    y = 0.
    th= 0.
    
    node_freq = 10.0
    
    t=0.001
    
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
        
        
        broadcaster.sendTransform( (x,   y,0.03), tf.transformations.quaternion_from_euler(0,0,th), rospy.Time.now(), "vehicle_frame","odom")
        broadcaster.sendTransform( (L,-.05,0.0), tf.transformations.quaternion_from_euler(0,0,current_angle), rospy.Time.now(), "front_left_wheel","vehicle_frame")
        broadcaster.sendTransform( (L, .05,0.0), tf.transformations.quaternion_from_euler(0,0,current_angle), rospy.Time.now(), "front_right_wheel","vehicle_frame")
        
        t=t-(cur_encoder_position - pre_encoder_position)*TICKS2RAD
        broadcaster.sendTransform( (0.,0.06,0.), tf.transformations.quaternion_from_euler(t,0,math.pi/2.0), rospy.Time.now(), "rear_left_wheel","vehicle_frame")
        broadcaster.sendTransform( (0.,-0.06,0.), tf.transformations.quaternion_from_euler(-t,0,-math.pi/2.0), rospy.Time.now(), "rear_right_wheel","vehicle_frame")
        
        q = tf.transformations.quaternion_from_euler(0,0,th)
        
        msg_odom = Odometry()
        msg_odom.header.stamp = rospy.Time.now()
        msg_odom.header.frame_id = "odom"
        msg_odom.pose.pose.position.x = x
        msg_odom.pose.pose.position.y = y
        msg_odom.pose.pose.position.z = 0.0
        msg_odom.pose.pose.orientation.x = q[0]
        msg_odom.pose.pose.orientation.y = q[1]
        msg_odom.pose.pose.orientation.z = q[2]
        msg_odom.pose.pose.orientation.w = q[3]
              
        
        #rospy.loginfo("Current speed %f m/s [%f km/h] -- yaw rate %f [rad/s]\n", motor_speed, motor_speed * 3.6, steering_speed)
        
        pub_vel.publish(motor_speed)
        pub_ticks.publish(cur_encoder_position)
        pub_odometry.publish(msg_odom)
        
        rate.sleep()
        
    rospy.loginfo("Shutting down encoder node")

if __name__ == '__main__':
    odometry()
    
