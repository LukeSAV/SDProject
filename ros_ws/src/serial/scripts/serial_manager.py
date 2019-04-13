#!/usr/bin/env python2.7
import rospy
import serial
import time
from sensor_msgs import msg
from std_msgs.msg import String, Bool
from threading import Lock


ser = serial.Serial(port="/dev/micro", baudrate=19200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

left_speed = '0'
left_direction = '+'
right_speed = '0'
right_direction = '+'

mode_change = 0 # Signal was made to change between display modes
prev_mode_change = 0 # Previous mode change indicator (debounce)

controller_change = 0
prev_controller_change = 0

controllerCallbackCalled = True
lock = Lock()

def controlAlgorithmCallback(commandMsg):
    ser.write(commandMsg.data)

def deliveryCallback(deliveryRequestedMsg):
    if deliveryRequestedMsg.data == True:
        ser.write('{D}')	
    else:
        ser.write('{N}')

def controllerCallback(joyMsg):
    global controllerCallbackCalled
    global left_speed
    global right_speed
    global left_direction
    global right_direction
    global mode_change
    global prev_mode_change
    global controller_change
    global prev_controller_change
    lock.acquire()
    controllerCallbackCalled = True
    left_speed = str(abs(int(joyMsg.axes[1] * 65)))
    right_speed = str(abs(int(joyMsg.axes[5] * 65)))
    lock.release()

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

    controller_change = joyMsg.buttons[1]
    if controller_change == 1 and prev_controller_change == 0:
        ser.write('{S}')
        prev_controller_change = 1
    if controller_change == 0:
        prev_controller_change = 0
            

def outputTimerCallback(event):
    global left_speed
    global right_speed
    ser.write('{'+ 'C' + left_direction + left_speed.zfill(3) + ' ' + right_direction + right_speed.zfill(3) + '}') #Format serial message and write it to the device on USB0

def joystickTimeoutCallback(event):
    global controllerCallbackCalled
    global left_speed
    global right_speed
    if controllerCallbackCalled == False:
        lock.acquire()
        left_speed = str('0')
        right_speed = str('0')
        lock.release()
    lock.acquire()
    controllerCallbackCalled = False
    lock.release()
    

""" Synchronizes to the last character of the data sent, prevents issues related to 
incorrect serial data being sent"""
def sync():
    inc = ser.read_until('}')

    
	
def listener():
    rospy.init_node('rc_listener')
    rospy.Subscriber("joy", msg.Joy, controllerCallback)
    rospy.Subscriber("delivery_requested_status", Bool, deliveryCallback)
    rospy.Subscriber("robot_cmd", String, controlAlgorithmCallback);
    rospy.Timer(rospy.Duration(0.1), outputTimerCallback)
    rospy.Timer(rospy.Duration(1.0), joystickTimeoutCallback)
    enc_pub = rospy.Publisher("/encoder", String, queue_size=2)
    control_pub = rospy.Publisher("/goal_string", String, queue_size=10)

    sync() # Align with ending message
    while not rospy.core.is_shutdown():
        packet_indicator = ser.read(2)	 # Number of bytes from the microcontroller

        # Encoder packet from the microcontroller
        if packet_indicator == r'{E':
          enc_data = ser.read(6)
          enc_pub.publish(String(str(packet_indicator + enc_data)))

        # Control algorithm output from the microcontroller
        if packet_indicator == r'{C':
          control_data = ser.read_until(r'}') # TODO: Set this value
          control_pub.publish(String(str(packet_indicator + control_data)))

        # Failed Sync routine
        else:
          ser.read_until('}')


if __name__ == '__main__':
 	listener()
	ser.close()
