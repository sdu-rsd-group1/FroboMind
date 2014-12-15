#!/usr/bin/env python
#roslib.load_manifest('rsd_group1')
import rospy
from std_msgs.msg import String
from rsd_group1.msg import Num, lego_brick, mes_order, Log



orderList = []    
trashList = []
ordersRed = 100
ordersBlue = 100
ordersYellow = 100

def log_it(level, code, logMessage):

    logEntry = Log()
    logEntry.CodeID = code
    logEntry.NodeID = 2
    logEntry.Text = logMessage

    pub3.publish(logEntry)



def send_brick():
    global orderList    
    global trashList
    global ordersRed
    global ordersBlue
    global ordersYellow
#		str = "hello world %s"%rospy.get_time()
#    rospy.loginfo(str)
    pub.publish(orderList[0])
    del orderList[0]

def order_done():
    pub2.publish("done")
    log_it(0,2,"Order is done after current bricks have been picked")

def callback(brick):
    global orderList    
    global trashList
    global ordersRed
    global ordersBlue
    global ordersYellow
    #print brick.color
    log_it(0,3,"Brick received from vision node")
    if brick.color == "Red" and ordersRed != 0:
        orderList.append(brick)
        ordersRed = ordersRed - 1
        print "filling orderlist red"
        log_it(1,4,"Sending Red brick to be picked")
        send_brick()
    if brick.color == "Blue" and ordersBlue != 0:
        orderList.append(brick)
        ordersBlue = ordersBlue - 1
        print "filling orderlist Blue"
        log_it(1,5,"Sending Blue brick to be picked")
        send_brick()
    if brick.color == "Yellow" and ordersYellow != 0:
        orderList.append(brick)
        ordersYellow = ordersYellow - 1
        print "filling orderlist Yellow"
        log_it(1,6,"Sending Yellow brick to be picked")
        send_brick()

    if ordersRed == 0 and ordersBlue == 0 and ordersYellow == 0:
        order_done()
        

def listener():
		rospy.Subscriber("brick", Num, callback)#sub the bricks from the vision node.
		#print "listener"

def callback2(b_list):
    #global orderList    
    #global trashList
    global ordersRed
    global ordersBlue
    global ordersYellow
    log_it(0,7,"New order list received")
    ordersRed = b_list.bricks[0].count #get the number of red bricks
    ordersBlue = b_list.bricks[1].count
    ordersYellow = b_list.bricks[2].count
    

def list_update():
		rospy.Subscriber("picklist", mes_order, callback2)

#def robocallback(brick):
#    talker()		#this should be placed after/at a request from robotic node
#    print "give me bricks"

#def robolisten():
#    rospy.Subscriber("bricklost", Num, robocallback)#bricks dropped/missed by the robot

if __name__ == '__main__':
    try:
        rospy.init_node('OrderQ', anonymous=True)	#log ("node started")
        #while not rospy.is_shutdown():
        #talker()
        pub = rospy.Publisher('brick2pick', Num, queue_size=100)
        pub2 = rospy.Publisher('orderdone', String, queue_size=100)
        pub3 = rospy.Publisher('logging', Log, queue_size=100)
        listener()
        list_update()
        #print "hell"
        rospy.spin()
    except rospy.ROSInterruptException: pass





