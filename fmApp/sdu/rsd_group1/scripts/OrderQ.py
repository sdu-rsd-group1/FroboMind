#!/usr/bin/env python
#roslib.load_manifest('rsd_group1')
import rospy
from std_msgs.msg import String
from rsd_group1.msg import Num, lego_brick, mes_order, Log, msg_hi, general



orderList = []    
trashList = []
ordersRed = 0
ordersBlue = 0
ordersYellow = 0

def log_it(level, code, logMessage):

    logEntry = Log()
    logEntry.CodeID = code
    logEntry.NodeID = 2
    logEntry.Text = logMessage

    pub_log.publish(logEntry)



def send_brick():
    global orderList    
    global trashList
    global ordersRed
    global ordersBlue
    global ordersYellow
#		str = "hello world %s"%rospy.get_time()
#    rospy.loginfo(str)
    pub_pick.publish(orderList[0])
    del orderList[0]

def order_done():
    pub_done.publish("done")
    log_it(0,2,"Order is done after current bricks have been picked")

def out_of_bricks():
    message = general()
    message.general = 2			# time
    pub_time.publish(message)

def callback(brick):
    global orderList    
    global trashList
    global ordersRed
    global ordersBlue
    global ordersYellow
    global then						# time
    then = rospy.Time.now()	# time


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
    
def callback_time(data):				#callback time
    global now
    now = rospy.Time.now()
    print now.secs
    print then.secs
    if now.secs-then.secs>20 and (ordersRed != 0 or ordersBlue != 0 or ordersYellow != 0):
        out_of_bricks()
        log_it(0,8,"Out of bricks")


def list_update():
		rospy.Subscriber("/mes/picklist", mes_order, callback2)

def time_check():             #time_check time
		rospy.Subscriber("/mes/hi_topic",msg_hi , callback_time)

#def robocallback(brick):
#    talker()		#this should be placed after/at a request from robotic node
#    print "give me bricks"

#def robolisten():
#    rospy.Subscriber("bricklost", Num, robocallback)#bricks dropped/missed by the robot

if __name__ == '__main__':
    try:
        rospy.init_node('OrderQ', anonymous=True)	#log ("node started")
        now = rospy.Time.now()				#timecheck time counter
        then = rospy.Time.now()
        #while not rospy.is_shutdown():
        #talker()
        pub_pick = rospy.Publisher('brick2pick', Num, queue_size=100)
        pub_done = rospy.Publisher('orderdone', String, queue_size=100)
        pub_log = rospy.Publisher('logging', Log, queue_size=100)
        pub_time = rospy.Publisher('/mes/incoming', general, queue_size=100)		# time
        listener()
        list_update()
        time_check()
        #print "hell"
        rospy.spin()
    except rospy.ROSInterruptException: pass













