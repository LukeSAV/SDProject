#!/usr/bin/env python2.7
import rospy
import serial
import time
from sensor_msgs import msg
from std_msgs.msg import String


ser = serial.Serial(port="/dev/micro", baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

left_speed = '0'
left_direction = '+'
right_speed = '0'
right_direction = '+'

mode_change = 0 # Signal was made to change between display modes
prev_mode_change = 0 # Previous mode change indicator (debounce)

controllerCallbackCalled = True

def controllerCallback(joyMsg):
    global left_speed
    global right_speed
    global left_direction
    global right_direction
    global mode_change
    global prev_mode_change
    left_speed = str(abs(int(joyMsg.axes[5] * 65)))
    right_speed = str(abs(int(joyMsg.axes[1] * 65)))

    if joyMsg.axes[5] < 0.0:
        left_direction = '-' 
    else:
        left_direction = '+'
    if joyMsg.axes[1] < 0.0 :
        right_direction = '-' 
    else:
        right_direction = '+'

    mode_change = joyMsg.buttons[3]
    if mode_change == 1 and prev_mode_change == 0:
        ser.write('{M}')
        prev_mode_change = 1
    if mode_change == 0:
        prev_mode_change = 0
    controllerCallbackCalled = True
            

def outputTimerCallback(event):
	ser.write('{'+ 'C' + left_direction + left_speed.zfill(3) + ' ' + right_direction + right_speed.zfill(3) + '}') #Format serial message and write it to the device on USB0


""" Synchronizes to the last character of the data sent, prevents issues related to 
incorrect serial data being sent"""
def sync():
    inc = ser.read_until('}')

    
	
def listener():
    rospy.init_node('rc_listener')
    rospy.Subscriber("joy", msg.Joy, controllerCallback)
    rospy.Timer(rospy.Duration(0.2), outputTimerCallback)
    enc_pub = rospy.Publisher("/encoder", String, queue_size=2)


    sync()
    while not rospy.core.is_shutdown():
        enc_data = ser.read(8)	 # Number of bytes from the microcontroller
        if enc_data[-1] != r'}':
            sync()
        else:
            enc_pub.publish(String(str(enc_data)))


if __name__ == '__main__':
 	listener()
	ser.close()
