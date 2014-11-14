#!/usr/bin/env python
# license removed for brevity
import rospy
import time

from std_msgs.msg import String
from std_msgs.msg import UInt32

import serial

ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)

def callback(data):
	ser.write(str(data.data) + "\0")
    
def listener():
	pub = rospy.Publisher('logging', UInt32, queue_size=10)
	rospy.init_node('serial_com_node', anonymous=True)
	rospy.Subscriber("serial_com", String, callback)
	ser.write("1Start\0")
	time.sleep(1)
	ser.write("2Start\0")
	pub.publish(0x300)
	rospy.spin()
	ser.close()
        
if __name__ == '__main__':
	listener()
    

