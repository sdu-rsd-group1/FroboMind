#!/usr/bin/env python
#roslib.load_manifest('rsd_group1')
import rospy
from std_msgs.msg import String
from rsd_group1.msg import Num



orderList = []    
trashList = []
ordersRed = 100
ordersBlue = 100
ordersYellow = 100


def talker():
    global orderList    
    global trashList
    global ordersRed
    global ordersBlue
    global ordersYellow
#		str = "hello world %s"%rospy.get_time()
#    rospy.loginfo(str)
    pub.publish(orderList[0])
    del orderList[0]

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
        talker()
    if brick.color == "Blue" and ordersBlue != 0:
        orderList.append(brick)
        ordersBlue = ordersBlue - 1
        print "filling orderlist Blue"
        talker()
    if brick.color == "Yellow" and ordersYellow != 0:
        orderList.append(brick)
        ordersYellow = ordersYellow - 1
        print "filling orderlist Yellow"
        talker()

def listener():
		rospy.Subscriber("brick", Num, callback)#sub the bricks from the vision node.
		#print "listener"

#def robocallback(brick):
#    talker()		#this should be placed after/at a request from robotic node
#    print "give me bricks"

def robolisten():
    rospy.Subscriber("bricklost", Num, robocallback)#bricks dropped/missed by the robot

if __name__ == '__main__':
    try:
        rospy.init_node('OrderQ', anonymous=True)
        #while not rospy.is_shutdown():
        #talker()
	pub = rospy.Publisher('brick2pick', Num, queue_size=100)
        listener()
        #print "hell"
        rospy.spin()
    except rospy.ROSInterruptException: pass





