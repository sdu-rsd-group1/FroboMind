#!/usr/bin/env python
#/****************************************************************************
# FroboMind cmd_vel_converter.py
# Copyright (c) 2011-2013, Leon Bonde Larsen <leon@bondelarsen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
import rospy, tf
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float64

class Updater():
    """
        Converter 
    """
    def __init__(self):       
        # Init parameters
        self.sensor_width = rospy.get_param("~sensor_width",2.0)
        self.sensor_length = rospy.get_param("~sensor_length",1.0)
        self.sensor_offset_x = rospy.get_param("~sensor_offset_x",0.6)
        self.sensor_offset_y = rospy.get_param("~sensor_offset_y",0)
        self.sensor_outerrange = rospy.get_param("~sensor_outerrange",0.1)
        self.sensor_map_frame = rospy.get_param("~sensor_map_frame","/map")
        self.sensor_map_topic = rospy.get_param("~sensor_map_topic","/fmKnowledge/sensor_map")
        self.sensor_value_topic = rospy.get_param("~sensor_value_topic","/fmInformation/wads")
        self.sensor_odom_frame = rospy.get_param("~sensor_odom_frame","/odom")
        self.resolution = rospy.get_param("~resolution",0.05)
        self.period = rospy.get_param("~period",0.2)
        self.model_threshold = rospy.get_param("~model_threshold",50)
        self.tf_offset_x = self.sensor_offset_x - self.sensor_outerrange
        self.tf_offset_y = (self.sensor_width/2) + self.sensor_outerrange
        self.trans_interval = rospy.get_param("~trans_interval",0.05)
        self.angle_interval = rospy.get_param("~angle_interval",0.087)
        self.current_position_x = 0
        self.current_position_y = 0
        self.current_angle = 0
        self.last_position_x = self.current_position_x
        self.last_position_y = self.current_position_y
        self.last_angle = self.current_angle
        
        
        # Init topics and transforms
        self.map_pub = rospy.Publisher(self.sensor_map_topic, OccupancyGrid)
        self.sensor_sub = rospy.Subscriber(self.sensor_value_topic, Float64, self.onSensor )
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        
        # Wait for TF data to avoid TF failing
        #try :
        #    self.listener.waitForTransform(self.sensor_odom_frame, self.sensor_map_frame, rospy.Time(0), rospy.Duration(10))
        #except tf.Exception as e:
        #    rospy.loginfo("Could not look up tf " + self.sensor_odom_frame + " to " + self.sensor_map_frame  )

        # Init timers
        #self.timer = rospy.Timer(rospy.Duration(self.period), self.publishTransform)
        self.timer = rospy.Timer(rospy.Duration(self.period), self.onTimer)

        # Map
        self.sensor_value = 0
        self.map = OccupancyGrid()     
        self.map.header.frame_id = self.sensor_odom_frame
        self.map.info.map_load_time = rospy.Time.now() # The time at which the map was loaded
        self.map.info.resolution = self.resolution # The map resolution [m/cell]
        self.map.info.width = int( np.ceil(( self.sensor_width + (2*self.sensor_outerrange) ) / self.resolution )) # Map width [cells]
        self.map.info.height = int( np.ceil(( self.sensor_length + (2*self.sensor_outerrange) ) / self.resolution ))  # Map height [cells]

        # Sensor model
        self.sensormodel = OccupancyGrid()
        self.sensormodel.info.resolution = self.resolution # The map resolution [m/cell]
        self.sensormodel.info.width = self.map.info.width
        self.sensormodel.info.height = self.map.info.height
        self.initSensormodel()

         
    def initSensormodel(self):
        thresh = 1
        center_x = self.sensormodel.info.width / 2
        center_y = self.sensormodel.info.height / 2

        #self.sensormodel.data = [self.sensor_value] * (self.map.info.width * self.map.info.height)

        for y in range(self.sensormodel.info.height) :
            for x in range(self.sensormodel.info.width) :
                dy = (x - center_x)*self.resolution
                dx = (y - center_y)*self.resolution
                sens_gain = 43.6719 - 4.5*dx - 50.625*dx*dx - 11.25*dy + 3.825*dx*dy - 20.3906*dy*dy
                sens_gain *= 2
                if sens_gain > 100:
                    sens_gain = 100
                if sens_gain < 0:
                    sens_gain = 0
                mappos = self.sensormodel.info.width*y+x
                #if sens_gain > thresh:
                self.sensormodel.data.append(int(sens_gain))
                #else:
                #    self.sensormodel.data.append(0)

    #def publishTransform(self,event):
        # Broadcast static transform from baselink to map
    #    self.br.sendTransform((self.tf_offset_x , self.tf_offset_y , 0),
    #                 tf.transformations.quaternion_from_euler(0, 0, - np.pi/2),
    #                 rospy.Time.now(),
    #                 "/map",
    #                 "/base_link")
        
    def onSensor(self,msg):
        # Save sensor value
        self.sensor_value = int( msg.data * 20 )
        
    
    def onTimer(self,event):
        # Position from tf
        try :      
            if not (self.listener.frameExists(self.sensor_odom_frame) and self.listener.frameExists(self.sensor_map_frame)):
                rospy.logerr("TF transform error from %s to %s will retry", self.sensor_odom_frame, self.sensor_map_frame)
                return
                
            #time_stamp=self.listener.getLatestCommonTime(self.sensor_odom_frame, self.sensor_map_frame) # This solves the bug in tf when using bagged data
            #(trans,rot) = self.listener.lookupTransform(self.sensor_odom_frame, self.sensor_map_frame, time_stamp)
            (trans,rot) = self.listener.lookupTransform(self.sensor_odom_frame, self.sensor_map_frame, rospy.Time(0)) 
            self.current_position_x = trans[0] 
            self.current_position_y = trans[1]

            # Set map information to current position
            self.map.info.origin.position.x = trans[0] 
            self.map.info.origin.position.y = trans[1]
            self.map.info.origin.orientation.x = rot[0]
            self.map.info.origin.orientation.y = rot[1]
            self.map.info.origin.orientation.z = rot[2]
            self.map.info.origin.orientation.w = rot[3]
            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(rot)
            self.current_angle = yaw
        except tf.LookupException as e:
            rospy.loginfo("LookupException: Could not look up tf " + self.sensor_odom_frame + " to " + self.sensor_map_frame  )
            pass
        except tf.ConnectivityException as e:
            rospy.loginfo("ConnectivityException: Could not look up tf " + self.sensor_odom_frame + " to " + self.sensor_map_frame)
            pass
        except tf.ExtrapolationException as e:
            rospy.loginfo("ExtrapolationException: Could not look up tf " + self.sensor_odom_frame + " to " + self.sensor_map_frame)
            pass

        # ..
        d_x = np.abs(self.current_position_x - self.last_position_x)
        d_y = np.abs(self.current_position_y - self.last_position_y)
        d_distance = np.sqrt(d_x*d_x + d_y*d_y)        
        d_angle = np.abs(self.current_angle - self.last_angle)
        if d_distance > self.trans_interval or d_angle > self.angle_interval:
            self.last_position_x = self.current_position_x
            self.last_position_y = self.current_position_y
            self.last_angle = self.current_angle
            self.publishMap()
                
    def publishMap(self):
        # Time stamp the map
        self.map.header.stamp = rospy.Time.now() 
        
        # Generate map from sensor model
        self.map.data = [-1] * (self.map.info.width * self.map.info.height)
        for x in range(self.map.info.width) :
            for y in range(self.map.info.height) :
                model_value =  self.sensormodel.data[self.map.info.width*y+x]
                if (model_value < self.model_threshold):
                    value = -1
                else:
                    value = self.sensor_value

                if (value > 100):
                    value = 100
                elif (value < -1):
                    value = 0
                self.map.data[self.map.info.width*y+x] = value 
                #if (value < 1):
                #    self.map.data[self.map.info.width*y+x] = -1
                #else:
                #    self.map.data[self.map.info.width*y+x] = value
#        self.map.data = [self.sensor_value] * (self.map.info.width * self.map.info.height)
#        for x in range(self.map.info.width) :
#            self.map.data[x] = self.sensor_value
#            self.map.data[self.map.info.width+x] = self.sensor_value
#            self.map.data[self.map.info.width*(self.map.info.height-1)+x] = self.sensor_value
#            self.map.data[self.map.info.width*(self.map.info.height-2)+x] = self.sensor_value
#        for y in range(self.map.info.height) :
#            self.map.data[self.map.info.width*y] = self.sensor_value
#            self.map.data[self.map.info.width*y+1] = self.sensor_value
#            self.map.data[self.map.info.width*y+self.map.info.width-1] = self.sensor_value
#            self.map.data[self.map.info.width*y+self.map.info.width-2] = self.sensor_value
        
        # Publish local map
        self.map_pub.publish(self.map)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_converter')
    node = Updater()
    rospy.spin()
    



    
