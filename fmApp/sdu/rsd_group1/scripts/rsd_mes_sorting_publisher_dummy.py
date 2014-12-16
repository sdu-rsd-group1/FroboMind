#!/usr/bin/env python
from std_msgs.msg import Int8
import rospy
from rsd_group1.msg import mes_sorting_command, mes_sorting_status, lego_brick, mes_order

def publish_loop():
    rospy.init_node('sortingdummy', anonymous=True)
    msg_status = mes_sorting_status()
    msg_status.header.stamp = rospy.Time.now()
    msg_status.version_id = 3
    msg_status.state = msg_status.STATE_SORTING
    msg_status.done_pct = 0
    msg_status.status = 'alora'

    msg_command = mes_sorting_command()
    msg_command.header.stamp = rospy.Time.now()
    msg_command.command = msg_command.COMMAND_SORTBRICKS
    msg_command.order.order_id = 1
    msg_command.order.bricks.append(lego_brick(color=lego_brick.COLOR_RED, size=4, count=2))        
    msg_command.order.bricks.append(lego_brick(color=lego_brick.COLOR_BLUE, size=6, count=5))
    msg_command.order.bricks.append(lego_brick(color=lego_brick.COLOR_YELLOW,size=8, count =2))
    pub2 = rospy.Publisher('/mes/command', mes_sorting_command,queue_size=1)
    #pub = rospy.Publisher('/mes/incoming', mes_sorting_command, queue_size=1)
    
    while not rospy.is_shutdown():
        #pub.publish(1)
        pub2.publish(msg_command)
        response = raw_input('Input something')
        print response

    res = rospy.Subscriber('/mes/status')
    res2 = rospy.Subscriber('/mes/command')

if __name__ == '__main__':
    try:
        publish_loop()
    except rospy.ROSInterruptException:
        print 'damn'
        pass
