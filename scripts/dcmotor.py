#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

from time import sleep
import time

import pigpio 

pig = pigpio.pi() 
PIN_MOTOR = 15
pwm_frequency = 20000 #Hz
pig.set_mode(20, pigpio.OUTPUT);
pig.set_mode(21, pigpio.OUTPUT);

forward = True

def speed_callback_keyboard(data):
    """
    Generates a PWM for the DC-MOTOR from a standard message (console)
    
    Args:
        data: ROS standard INT32 message. 
    """
    if data.data <= 255 and data.data >= 0:
        rospy.loginfo("Speed %i", data.data)
        pig.set_PWM_dutycycle(PIN_MOTOR, data.data)
    else:
        rospy.loginfo("Too much! Set something between 0 and 80")

def speed_callback_joystick(data):
    global forward
    """
    Generates a PWM for the DC-MOTOR from a standard message (joystick)
    The joystick node/driver of ros sends for each channel a number 
    within -1...+1 range. This number is then adapted for the dcmotor 
    values, within 0..255 range. The forward/backward actions are 
    controlled with the two pins in GPIO20 and GPIO21.
    
    Please notice that changing from forward/to/backward and vice-versa
    requires a "stop" to not overload the h-bridge.
    
    Args:
        data: ROS standard JOY message. 
    """
    cmd = data.axes[1] * 255
    if cmd >= 0.0:
        if not forward:
            rospy.loginfo("Waiting 0.1s, change direction required")
            pig.set_PWM_dutycycle(PIN_MOTOR, 0)
            time.sleep(0.1)                      
            forward = True
        pig.write(20, 1)
        pig.write(21, 0)
    else:
        if forward:
            rospy.loginfo("Waiting 0.1s, change direction required")
            pig.set_PWM_dutycycle(PIN_MOTOR, 0)
            time.sleep(0.1)     
            forward = False
        pig.write(20, 0)
        pig.write(21, 1)
    cmd = abs(cmd)
    if cmd <= 255 and cmd >= 0:
        if forward:
            rospy.loginfo("Speed Forward\t%i", cmd)
        else:
            rospy.loginfo("Speed Backward\t%i", cmd)            
            
        pig.set_PWM_dutycycle(PIN_MOTOR, cmd)
    else:
        rospy.loginfo("Too much! Set something between 0 and 255")
    

def listener():
    """
    ROS listener routine. 
    Initializes the node and defines the two subscribers. After the spin
    command, ROS takes control.
    
    """

    rospy.init_node('dcmotor_node', anonymous=True)
    rospy.Subscriber("chatter", Int32, speed_callback_keyboard)
    rospy.Subscriber("joy", Joy, speed_callback_joystick)    
    
    rospy.spin()
    
    rospy.loginfo("Shutting down dcmotor node")


if __name__ == '__main__':
    
    forward = True
    
    pig.set_PWM_frequency(PIN_MOTOR,pwm_frequency)
    pig.set_PWM_dutycycle(PIN_MOTOR,0)

    listener()
    


