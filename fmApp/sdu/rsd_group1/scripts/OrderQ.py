#!/usr/bin/env python
#roslib.load_manifest('rsd_group1')
import rospy
from std_msgs.msg import String
from rsd_group1.msg import Num



orderList = []    
trashList = []
ordersRed = 2
ordersBlue = 1
ordersYellow = 1


def talker():
    global orderList    
    global trashList
    global ordersRed
    global ordersBlue
    global ordersYellow
    pub = rospy.Publisher('nextbrick', Num, queue_size=10)
#		str = "hello world %s"%rospy.get_time()
#    rospy.loginfo(str)
    pub.publish(orderList[1])
    del orderList[1]

def callback(brick):
    global orderList    
    global trashList
    global ordersRed
    global ordersBlue
    global ordersYellow
    #print brick.color
    if brick.color == "Red" and ordersRed != 0:
        orderList.append(brick)
        ordersRed = ordersRed - 1
        print "filling orderlist red"
    if brick.color == "Blue" and ordersBlue != 0:
        orderList.append(brick)
        ordersBlue = ordersBlue - 1
        print "filling orderlist Blue"
    if brick.color == "Yellow" and ordersYellow != 0:
        orderList.append(brick)
        ordersYellow = ordersYellow - 1
        print "filling orderlist Yellow"

def listener():
		rospy.Subscriber("brick", Num, callback)#sub the bricks from the vision node.
		#print "listener"

def robocallback(brick):
    talker()		#this should be placed after/at a request from robotic node
    print "give me bricks"

def robolisten():
    rospy.Subscriber("brick2pic", Num, robocallback)#sub the bricks from the vision node.

if __name__ == '__main__':
    try:
        rospy.init_node('OrderQ', anonymous=True)
        #while not rospy.is_shutdown():
        #talker()
        listener()
        #print "hell"
        rospy.spin()
    except rospy.ROSInterruptException: pass





