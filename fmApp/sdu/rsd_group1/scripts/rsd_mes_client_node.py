#!/usr/bin/env python

import rospy
import xmlrpclib
from rsd_group1.msg import mes_sorting_command, mes_sorting_status, mes_order, lego_brick, Log
from std_msgs.msg import String
class RSDMesClientNode():
    new_state=0
    def __init__(self):  # initaialize all the functions
      
        self.version_id = 1        
        self.initNode()
        self.getParams()
        self.initMsg()
        self.initTopic()
        self.initVars()
        self.initTimer()
        self.status_valid = False
        self.new_state=0
        
        if (len(self.server_address) > 0): #check for online
            self.online = True
            self.fillDefaultData()
            self.initCommunication()
            rospy.loginfo("Server: " + self.server_address)
            rospy.loginfo("Starting in online mode")
            rospy.loginfo("Communication will be initialized when first valid status is received")
        else:
            self.online = False
            self.fillDummyData()
            rospy.loginfo("Starting in offline mode")
                    
    def initNode(self):
        self.rosnode = rospy.init_node('rsd_mes_generic_client')
        
    def getParams(self):
        
        self.mes_command_topic = rospy.get_param("~mes_command_topic", 'mes_command_topic') 
        self.mes_status_topic = rospy.get_param("~mes_status_topic", 'mes_status_topic')
        self.mes_incoming_topic = rospy.get_param("~mes_incoming_topic", 'mes_incomig_topic')
        self.mes_outgoing_topic = rospy.get_param("~mes_outgoing_topic","mes_outgoing_topic")
        self.mes_order_topic = rospy.get_param("~mes_order_topic",'mes_order_topic')
        self.update_duration = rospy.get_param("~update_duration", 1.0)
        self.server_address = rospy.get_param("~server_address", '')
        self.robot_id = rospy.get_param("~robot_id", 3)
        self.log_topic = rospy.Publisher("logging", Log, queue_size=1)
        self.logDataToTopic(0,1,'Initaialising topics')

    def initMsg(self):
        return
        
    def initTopic(self):
        return

    def initVars(self):
        self.command_dict = {}
        self.command_error = 0
        self.state_dict = {}
        self.state_error = ''

    def getStatus(self):    
        status = {
            'version_id': self.version_id
        }
        return status
        
    def convertCommand(self,command):
        if self.command_dict.has_key(command):
            return self.command_dict[command]
        else:
            rospy.logerr("Invalid command received")
            return self.command_error

    def convertState(self,status):
        if self.state_dict.has_key(status):
            return self.state_dict[status]
        else:
            rospy.logerr("Invalid status received")
            return self.state_error

    def initTimer(self):
        rospy.Timer(rospy.Duration(self.update_duration), self.update)  
        
    def initCommunication(self):
        self.server_connection = xmlrpclib.ServerProxy(self.server_address, use_datetime=True)
        
    def fillDummyData(self):
        return        
    
    def fillDefaultData(self):
        return
    
    def callbackStatus(self,status):
        self.ros_msg_status = status
        if not (self.status_valid):
            self.status_valid = True
            rospy.loginfo("First valid status received")

    def callbackNewStatus(self,state):   
        #if data is not new_state:
        self.new_state = state

    def serverInfoExchange(self):
        return

    def publishCommand(self):        
        try:
            self.ros_msg_command.header.stamp = rospy.Time.now()
            self.mes_command_publisher.publish(self.ros_msg_command)
        except:
            rospy.logerr("Publisher/messages not initialized correct!")
    
    def update(self, event):
        if (self.online and self.status_valid):
            self.serverInfoExchange()
        self.publishCommand()
    
    def logDataToTopic(self, level, code, data):
        log_data=Log()
        log_data.NodeID = 3
        log_data.CodeID = code
        log_data.Level = level
        log_data.Text = data
        self.log_topic.publish(log_data)

if __name__ == '__main__':  
    print("This file is not intended to run as standalone")
