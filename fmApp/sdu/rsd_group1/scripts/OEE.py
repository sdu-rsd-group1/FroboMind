#!/usr/bin/python
import roslib
roslib.load_manifest('rsd_group1')
import sys
import rospy
from std_msgs.msg import UInt32
from rsd_group1.msg import Log
from rsd_group1.msg import OEEmsg
import xmlsettings


def publishToLog(level, code, logMessage):
    #-------------------publish list of bricks---------------------------------------------------------------------
    #Node ID = 2
    #level = 0 = debug, 1 = operation
    logEntry = Log()
    logEntry.CodeID = code
    logEntry.NodeID = 5
    logEntry.Text = logMessage

    if level == "operation":
        logEntry.Level = 1
    elif level == "debug":
        logEntry.Level = 0
    else:
        print "Invalid loglevel:"
        print level
        return

    pubLog.publish(logEntry)

    return


class OEE:
    def __init__(self):

        #self.currentModule = null
        print "init start"
        self.timeStamp = 0
        self.prevType = "null"
        self.currentType = "null"
        global OEE_xml
        OEE_xml = xmlsettings.XMLSettings('/home/robot/roswork/src/fmApp/sdu/rsd_group1/scripts/OEE.xml')

        self.down_time = 0
        self.speed_loss = 0
        self.quality_loss = 0
        self.planned_operation = 0
        self.operation = 0

        self.down_time = float(OEE_xml.get('OEE_Stats/down_time',0.0))
        self.speed_loss = float(OEE_xml.get('OEE_Stats/speed_loss',0.0))
        self.quality_loss = float(OEE_xml.get('OEE_Stats/quality_loss',0.0))
        self.planned_operation = float(OEE_xml.get('OEE_Stats/planned_operation',0.0))
        self.operation = float(OEE_xml.get('OEE_Stats/operation',0.0))

	#Enumerated defines for states, first is 0
        #STOP,START,READY,EXECUTE,SUSPENDED,GO_TO_UPPER_BRICK,GO_TO_LOWER_BRICK,BRICK_TO_MIDDLE,MIDDLE_TO_BOX,
        #BOX_TO_MIDDLE,COMPLETED,GRASP_BRICK,RELEASE_BRICK,OPEN_GRIP,ABORT,RESET,SECURITY

        #"defines" for states
        self.STOP = 0
        self.START = 1
        self.READY = 2
        self.EXECUTE = 3
        self.SUSPENDED = 4
        self.GO_TO_UPPER_BRICK = 5
        self.GO_TO_LOWER_BRICK = 6
        self.BRICK_TO_MIDDLE = 7
        self.MIDDLE_TO_BOX = 8
        self.BOX_TO_MIDDLE = 9
        self.COMPLETED = 10
        self.GRASP_BRICK = 11
        self.RELEASE_BRICK = 12
        self.OPEN_GRIP = 13
	self.ABORT = 14
	self.RESET = 15
	self.SECURITY = 16

        rospy.Subscriber("robot_states", UInt32, self.OEECallback)

        return

    def OEECallback(self, state):

        #Get the time between state changes
        tempTime = rospy.get_time() - self.timeStamp
        self.timeStamp = rospy.get_time()

        if self.currentType == "null":
            tempTime = 0

        #Get add the time to the correct type
        if self.currentType == "down_time":
            self.down_time = float(self.down_time) + tempTime
        elif self.currentType == "speed_loss":
            self.speed_loss = float(self.speed_loss) + tempTime
        elif self.currentType == "quality_loss":
            self.quality_loss = float(self.quality_loss) + tempTime
        elif self.currentType == "operation":
            self.operation = float(self.operation) + tempTime
        else:
            pass

        #Update prev state type before current is changed
        self.prevType = self.currentType


	#Enumerated defines for states, first is 0
        #STOP,START,READY,EXECUTE,SUSPENDED,GO_TO_UPPER_BRICK,GO_TO_LOWER_BRICK,BRICK_TO_MIDDLE,MIDDLE_TO_BOX,
        #BOX_TO_MIDDLE,COMPLETED,GRASP_BRICK,RELEASE_BRICK,OPEN_GRIP,ABORT,RESET,SECURITY


        #Get the type of state, determines if time is downtime, speed loss, etc.
        if (state.data == self.STOP) or (state.data == self.RESET) or (state.data == self.START) or (state.data == self.ABORT):
            self.currentType = "down_time"
        elif (state.data == self.READY) or (state.data == self.EXECUTE):
            self.currentType = "speed_loss"
        elif 0:
            self.currentType = "quality_loss"
        else:
            self.currentType = "operation"

       


        if self.prevType != "null":
            publishToLog("debug", 0, "Current state of type: "+self.currentType+" ended after "+str(tempTime)+"s")
            publishToLog("debug", 0, "New state of type: "+self.currentType+" started")
            self.publishOEE()

        return

    def publishOEE(self):

        #Calculate planned operationg time, operationg time, and availability
        operating_time = self.operation + self.speed_loss + self.quality_loss
        planned_operating_time = operating_time + self.down_time
        availability = operating_time / (planned_operating_time+0.000000001)

        #Calculate performance
        net_operating_time = operating_time-self.speed_loss
        performance = net_operating_time / (operating_time+0.000000001)

        #Calculate quality
        fully_productive_operating_time = net_operating_time - self.quality_loss
        quality = fully_productive_operating_time / (net_operating_time+0.000000001)

        OEE_value = availability * performance * quality

        OEE_xml.put('OEE_Stats/down_time', float(self.down_time))
        OEE_xml.put('OEE_Stats/speed_loss', float(self.speed_loss))
        OEE_xml.put('OEE_Stats/quality_loss', float(self.quality_loss))
        OEE_xml.put('OEE_Stats/operation', float(self.operation))

        OEE_xml.save()

        publishToLog("debug", 1, "New OEE stats saved in OEE.xml")

        oeeStats = OEEmsg()

        oeeStats.planned_operating_time = planned_operating_time
        oeeStats.operating_time = operating_time
        oeeStats.availability = availability

        oeeStats.net_operating_time = net_operating_time
        oeeStats.performance = performance

        oeeStats.fully_productive_operating_time = fully_productive_operating_time
        oeeStats.quality = quality
        oeeStats.OEE = OEE_value

	oeeStats.down_time = self.down_time
        oeeStats.speed_loss = self.speed_loss
        oeeStats.quality_loss = self.quality_loss

        pubOEE.publish(oeeStats)

        publishToLog("operation", 2, "New OEE stats calculated and published; new OEE: "+str(OEE_value))

        return


def main(args):
    oee_inst = OEE()
    rospy.init_node('OEE', anonymous=True)
    publishToLog("operation", 3, "OEE node sucessfully initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    publishToLog("operation", 4, "OEE node shutting down")


if __name__ == '__main__':
    #Make a publisher for publishing the OEE stats
    print "main"
    pubOEE = rospy.Publisher('OEE_stats', OEEmsg, queue_size=100)
    pubLog = rospy.Publisher('logging', Log, queue_size=100)
    main(sys.argv)


