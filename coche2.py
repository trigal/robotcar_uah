# This program will let you test your ESC and brushless motor.
# Make sure your battery is not connected if you are going to calibrate it at first.
# Since you are testing your motor, I hope you don't have your propeller attached to it otherwise you are in trouble my friend...?
# This program is made by AGT @instructable.com. DO NOT REPUBLISH THIS PROGRAM... actually the program itself is harmful                                             pssst Its not, its safe.

import os                       #importing os library so as to communicate with the system
import time                     #importing time library to make Rpi wait because its too impatient 
os.system ("sudo pigpiod")      #Launching GPIO library
time.sleep(1)                   # As i said it is too impatient and so if this delay is removed you will get an error
import json
import matplotlib.pyplot as  plt
import numpy as np
from math import *
import sys
import select
import termios

import pigpio                   # importing GPIO library
import filterpy
import LS7366R
import threading

class Thread_Keyboard(threading.Thread):
    def __init__(self, threadName):
        threading.Thread.__init__(self)
        self.threadName = threadName
        self.wheel_position = 1700
        self.speed_ref = 0
        print ("Thread created: " + self.threadName)
 
    def run(self):
        print ("Starting treading: " + self.threadName)

        while (True):
          aux = input()
          if aux == 'j':
             if self.wheel_position > 1300:
               self.wheel_position -= 1
          elif aux == 'l':
             if self.wheel_position < 2100:
               self.wheel_position += 1
          elif aux == 'i':
               self.speed_ref += 100
          elif aux == 'k':
               self.speed_ref -= 100
          elif aux == '0':
               self.speed_ref = 0
               self.wheel_position = 1700

          print (self.speed_ref)
 
    def get_wheel_position(self):
        return self.wheel_position

    def get_speed_ref(self):
        return self.speed_ref

    def stop(self):
        print ("Stoping treading: " + self.threadName)

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


integral_error = 0

def speed_pid_controller(speed_ref, speed):
    global integral_error

    kp = 0.1
    ki = 0.095

    error = speed_ref - speed
    integral_error += error

    u = kp*error + ki*integral_error
    if u < 50:
        u = 50
    elif u > 255:
        u = 255

    return u

def coche2(thread_keyboard):
    
    SERVO=14      # SERVO en el pin 14 GPIO
    MOTOR=15
    pwm_frequency = 20000 #Hz
    T = 0.01

    pig = pigpio.pi() 
    pig.set_PWM_frequency(MOTOR,pwm_frequency)

    encoder = LS7366R.LS7366R(0, 3900000, 4)
    cur_encoder_position = 0
    pre_encoder_position = 0

    while (True):   # El bucle dura 65 sec para que de tiempo a detectar todas las oscilaciones hasta que el sistema se pare completamente.
        timer0 = time.time()                          # Inicio el contador de tiempo para contar el tiempo de muestreo

        wheel_position = thread_keyboard.get_wheel_position()

        # Wheel angle 
        pig.set_servo_pulsewidth(SERVO, wheel_position)

        # Encoder 
        pre_encoder_position = cur_encoder_position
        cur_encoder_position = encoder.readCounter()
        motor_speed = abs((cur_encoder_position - pre_encoder_position)/T)

        ## Controller
        speed_ref = thread_keyboard.get_speed_ref()
        dutycycle = speed_pid_controller(speed_ref, motor_speed)
        print ('SpeedRef MotorSpeed Dutycycle WheelPosition: ' + str([speed_ref, motor_speed, dutycycle, wheel_position]))

        # Motor 
        pig.set_PWM_dutycycle(MOTOR, dutycycle)
    
        timer1 = time.time()  
        dt = timer1-timer0
        if( dt < T):          
            time.sleep(T - dt) 
        else:
            print('Sobrepasado tiempo de control')


if __name__ == '__main__':
    try:
      old_settings = termios.tcgetattr(sys.stdin)
      thread_keyboard = Thread_Keyboard(threadName='Thread Keyboard Reader')
      thread_keyboard.start()

      coche2(thread_keyboard)               
    except KeyboardInterrupt:
      #ecoder.close()
      thread_keyboard.stop()
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
      print ("All done, bye bye!!.")
            



