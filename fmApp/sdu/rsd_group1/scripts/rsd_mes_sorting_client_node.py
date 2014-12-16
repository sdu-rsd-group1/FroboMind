#!/usr/bin/env python

import rospy
import xmlrpclib
import datetime
from rsd_mes_client_node import RSDMesClientNode
from rsd_group1.msg import mes_sorting_command, mes_sorting_status, mes_order, lego_brick, Log, general, msg_hi
 
class RSDMesSortingClientNode(RSDMesClientNode):
    def __init__(self):
        RSDMesClientNode.__init__(self)
        return

    def initMsg(self):  
        self.ros_msg_command = mes_sorting_command()
        self.ros_msg_status = mes_sorting_status()
         
    
    def initNode(self):
        self.rosnode = rospy.init_node('rsd_mes_node')
    
    def initTopic(self):
        self.logDataToTopic(0,0,"MES client initialized")
        self.mes_command_publisher = rospy.Publisher(self.mes_command_topic, mes_sorting_command, queue_size=1)
        rospy.Subscriber(self.mes_status_topic, mes_sorting_status, self.callbackStatus)
        self.mes_outgoing_publisher = rospy.Publisher(self.mes_outgoing_topic,general , queue_size=1)
        self.mes_order_publisher=rospy.Publisher(self.mes_order_topic, mes_order ,queue_size=1)
        rospy.Subscriber(self.mes_incoming_topic, general, self.callbackNewStatus)
        self.hi_topic_publisher = rospy.Publisher("hi_topic", msg_hi , queue_size=1)
        
    def initVars(self):
        self.command_dict = {
            'COMMAND_WAIT': mes_sorting_command.COMMAND_WAIT,
            'COMMAND_LOADBRICKS': mes_sorting_command.COMMAND_LOADBRICKS,
            'COMMAND_SORTBRICKS' : mes_sorting_command.COMMAND_SORTBRICKS,
            'COMMAND_ABORT' : mes_sorting_command.COMMAND_ABORT
        }
        self.command_error = mes_sorting_command.COMMAND_WAIT
        self.state_dict = {
            mes_sorting_status.STATE_FREE : 'STATE_FREE',
            mes_sorting_status.STATE_LOADING : 'STATE_LOADING',
            mes_sorting_status.STATE_ORDERSORTED : 'STATE_ORDERSORTED',
            mes_sorting_status.STATE_OUTOFBRICKS : 'STATE_OUTOFBRICKS',
            mes_sorting_status.STATE_SORTING : 'STATE_SORTING',
            mes_sorting_status.STATE_ERROR : 'STATE_ERROR'
        }
        self.state_error = 'STATE_ERROR'     
     
    def getStatus(self):
        rospy.loginfo("state "+str(self.new_state)) 
        self.logDataToTopic(1,4,"Updating state")
        ns=""
        if self.new_state is 0: #"free":
            ns=0
        elif self.new_state is 1: #"sorting":
            ns=1
        elif self.new_state is 2: #"outofbricks":
            ns=2
        elif self.new_state is 3: #"ordersorted":
            ns=3
        elif self.new_state is 4: #"loading":
            ns=4
        elif self.new_state is 5: #"error":
            ns=5
        
        status = {
            'version_id': self.version_id,
            'robot_id': self.robot_id,
            'state': ns,
            'time': str(datetime.datetime.fromtimestamp(self.ros_msg_status.header.stamp.to_time())),
            'done_pct': self.ros_msg_status.done_pct,
            'status': self.ros_msg_status.status
        }

        return status 
                
    def setCommand(self,command):
        self.logDataToTopic(1,2,"New command received")
	gnrl=general()
 	gnrl.general=self.convertCommand(command['command'])
        self.mes_outgoing_publisher.publish(gnrl)
        self.logDataToTopic(1,3,"New command published to topic")
        rospy.loginfo("command "+str(self.convertCommand(command['command'])))
        self.ros_msg_command.command = self.convertCommand(command['command'])
        
        if (command.has_key("order")):
            self.logDataToTopic(1,4,"New order received")
            self.msg_command.order.order_id = command['order']['order_id']
            self.msg=command['order'] # added
            self.mes_order_publisher.publish(self.msg)
            self.logDataToTopic(1,5,"New order published to Topic")
            legos = command['order']['bricks']
            self.msg_command.order.bricks = []
            for i in range(len(legos)):
                self.msg_command.order.bricks.append(lego_brick())
                self.msg_command.order.bricks[i].color = legos[i]['color']
                self.msg_command.order.bricks[i].size = legos[i]['size']
                self.msg_command.order.bricks[i].count = legos[i]['count']
           
        else:
            self.logDataToTopic(1,6,"No order received initaialize default")
            self.ros_msg_command.order.order_id = 0
            self.ros_msg_command.order.bricks = []
        return
    
    def fillDefaultData(self):
        self.ros_msg_command.command = self.ros_msg_command.COMMAND_WAIT
        self.ros_msg_command.order.order_id = 0
        self.ros_msg_command.order.bricks = []
    
    def publishCommand(self):        
        try:
 
	    hi_o=msg_hi()
	    hi_o.hi= "hi"
            self.hi_topic_publisher.publish(hi_o)
        except:
            rospy.logerr("Publisher/messages not initialized correct!")
    
    def serverInfoExchange(self):
        # try:
        self.logDataToTopic(1,1,"Comunicating with server")
        status = self.getStatus()
        command = (self.server_connection.cell_status(status))
        self.setCommand(command)            
        # except:
        #     rospy.logerr("Communication with server failed")
        #     self.online = False

if __name__ == '__main__':
    try:
        node_class = RSDMesSortingClientNode()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

