#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
import os
import sys
#from dynamic_reconfigure.server import Server
#from robotcar_uah.cfg import system_configConfig

from time import sleep

import pigpio 

pig = pigpio.pi() 
PIN_SERVO=17 
center_wheel_position = 1700

def LTRTbuttons_joystick(data):   
    """
    Designed to works with XBOX-ONE controller
    Listen to LT and RT buttons (the triggers), if both are pressed,
    shutdown ROS in safe conditions    

    Args:
        data: ROS standard JOY message. 
    
    """ 
    
    RT = -data.axes[4] 
    LT = -data.axes[5] 
    rospy.loginfo("RT %i    LT %i", RT, LT)

    if LT == 1 and RT == 1:
        rospy.loginfo("Received SHUTDOWN signal from joypad!")
        os.system("rosnode kill --all")   
        os.system("pkill -f robotcar_launcher.launch")
        #os._exit(1) #alternative way, but writes "error" on screen! 
        
        
def listener():
    """
    ROS listener routine. 
    Initializes the node and defines the two subscribers. After the spin
    command, ROS takes control.
    
    """
    
    rospy.init_node('system_node', anonymous=True)
    rospy.Subscriber("joy", Joy, LTRTbuttons_joystick)
   
    rate = rospy.Rate(10)
    broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        param_x     = rospy.get_param('~setup_x')
        param_y     = rospy.get_param('~setup_y')
        param_z     = rospy.get_param('~setup_z')
        param_roll  = rospy.get_param('~setup_roll')
        param_pitch = rospy.get_param('~setup_pitch')
        param_yaw   = rospy.get_param('~setup_yaw')
    
        broadcaster.sendTransform( (param_x,param_y,param_z), tf.transformations.quaternion_from_euler(param_roll,param_pitch,param_yaw), rospy.Time.now(), "laser_frame", "vehicle_frame")

        rate.sleep()


    rospy.loginfo("Shutting down steering_node")


if __name__ == '__main__':
    listener()
