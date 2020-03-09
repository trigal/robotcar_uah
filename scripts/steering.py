#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

from time import sleep

import pigpio 

pig = pigpio.pi() 
PIN_SERVO=17 
center_wheel_position = 1700

def steering_callback(data):
    """
    Generates a PWM for the steering servo from a standard message (keyboard)
    
    Args:
        data: ROS standard INT32 message. 
    """
    
    if data.data <= 2100 and data.data >= 1300:
        rospy.loginfo("Setting %i", data.data)
        pig.set_servo_pulsewidth(PIN_SERVO, cmd)
    else:
        rospy.loginfo("Too much! Set something between 1300 and 2100 --- %i ", cmd)

def steering_callback_joystick(data):   
    """
    Generates a PWM for the steering servo from a standard message (joystick version)
    The joystick node/driver of ros sends for each channel a number 
    within -1...+1 range. This number is then adapted for the steering
    values, within 1300 and 1700 range. These values were ad-hoc tuned
    for the UAH-ROBOTCAR vehicle of the VEHICULOS INTELIGENTES course.    
    
    Args:
        data: ROS standard JOY message. 
    
    """ 
    cmd = -data.axes[2] * 400 + 1700;
    if cmd <= 2100 and cmd >= 1300:
        rospy.loginfo("Setting %i", cmd)
        pig.set_servo_pulsewidth(PIN_SERVO, cmd)
    else:
        rospy.loginfo("Too much! Set something between 1300 and 2100 --- %i ", cmd)
    
def listener():
    """
    ROS listener routine. 
    Initializes the node and defines the two subscribers. After the spin
    command, ROS takes control.
    
    """
    
    rospy.init_node('steering_node', anonymous=True)
    rospy.Subscriber("chatter", Int32, steering_callback)
    rospy.Subscriber("joy", Joy, steering_callback_joystick)
    pig.set_servo_pulsewidth(PIN_SERVO, center_wheel_position)
    rospy.spin()
    
    rospy.loginfo("Shutting down steering_node")


if __name__ == '__main__':
    listener()
    


