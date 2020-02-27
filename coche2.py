# This program will let you test your ESC and brushless motor.
# Make sure your battery is not connected if you are going to calibrate it at first.
# Since you are testing your motor, I hope you don't have your propeller attached to it otherwise you are in trouble my friend...?
# This program is made by AGT @instructable.com. DO NOT REPUBLISH THIS PROGRAM... actually the program itself is harmful                                             pssst Its not, its safe.

import os                       #importing os library so as to communicate with the system
import time                     #importing time library to make Rpi wait because its too impatient 
os.system ("sudo pigpiod")      #Launching GPIO library
time.sleep(1)                   # As i said it is too impatient and so if this delay is removed you will get an error
import pigpio                   # importing GPIO library
import json
import matplotlib.pyplot as  plt
import numpy as np
from math import *
import sys
import select
import termios

import filterpy

error_ant=0
error_integral =0
Km = 0.004198202773919557

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


            
def coche2():
    
    ESC=14      # ESC en el pin 14 GPIO
    MOTOR=15
    frequency = 100 #Hz
    T = 0.1
    f = 1.0/T

    pig_servo = pigpio.pi('servo')  
    pig_motor = pigpio.pi('motor')  
    pig_motor.set_PWM_frequency(MOTOR,frequency)

    pwm_angle = 1700

    print("Introduzca el angulo en grados entre 0 y 60 y pulse enter")
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        while (True):   # El bucle dura 65 sec para que de tiempo a detectar todas las oscilaciones hasta que el sistema se pare completamente.
            timer0 = time.time()                          # Inicio el contador de tiempo para contar el tiempo de muestreo

            # Wheel angle 
            pig_servo.set_servo_pulsewidth(ESC, 0)
            print ('Wheel angle: ' + str(pwm_angle))

            # Motor 
            dutycycle = 10
            pig_motor.set_PWM_dutycycle(MOTOR, dutycycle)
        
            timer1 = time.time()  
            dt = timer1-timer0
            if( dt < T):          
                time.sleep(T - dt) 
            else:
                print('Sobrepasado tiempo de control')

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        

if __name__ == '__main__':
    coche2()               
            



