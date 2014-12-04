#!/usr/bin/env python
#roslib.load_manifest('rsd_group1')
import rospy
from std_msgs.msg import String
from rsd_group1.msg import Num



#class order:
#    def __init__(self):
#        self.orderList = []    
#        self.trashList = []
#        self.ordersRed = 2
#        self.ordersBlue = 1
#        self.ordersYellow = 1

def talker():
		pub = rospy.Publisher('chatter', String, queue_size=10)
		str = "hello world %s"%rospy.get_time()
#    rospy.loginfo(str)
		pub.publish(str)

def callback(brick):
    print brick.color
#    if brick.color == "Red" and self.ordersRed != 0:
#        self.orderList.append(brick)
#        self.ordersRed = self.ordersRed - 1

def listener():
		rospy.Subscriber("brick", Num, callback)#sub the bricks from the vision node.
				#print "listener"
		rospy.spin()



if __name__ == '__main__':
    try:
        rospy.init_node('OrderQ', anonymous=True)
#        order = order()
        while not rospy.is_shutdown():
#            r = rospy.Rate(10) # 10hz
            #talker()
            listener()
#            r.sleep()
            print "hell"
    except rospy.ROSInterruptException: pass





