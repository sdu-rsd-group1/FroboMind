#!/usr/bin/env python
import roslib
roslib.load_manifest('rsd_group1')
import sys
import rospy
import cv2
from cv2 import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from Lego_Brick import LegoBrick
from rsd_group1.msg import Num



def getBricks(contours, color):
    #Get a list of contours and extracts the bricks
    bricks = []
    tempList = []

    areaReq = 0
    if color == "Blue":
        areaReqMin = 300
        areaReqMax = 1000
    elif color == "Red":
        areaReqMin = 500
        areaReqMax = 2000
    elif color == "Yellow":
        areaReqMin = 700
        areaReqMax = 3000

    for cnt in contours:
        if contourArea(cnt) > areaReqMin and contourArea(cnt) < areaReqMax:
            #Rectangle data is (x, y) (width, height) (angle)
            rect = minAreaRect(cnt)
            bricks.append(rect)
            sort(rect[1], False)

            #print "Width: " + str(rect[1][0]) + "height: " + str(rect[1][1])
            #print "Aspect " + str(aspect)
            #Thresholds the brick


            # print ".................."
            # print "Brick type is: " + size + " " + str(color)
            # print "Brick pose is:"
            # print "x: " + str(rect[0][0]) + " y: " + str(rect[0][1])
            tempAngle = rect[2]

            tempList.append(LegoBrick(rect[0][0],rect[0][1],tempAngle,color,rospy.get_time()))

    return bricks, tempList


class image_converter:

  def __init__(self):
    
    self.leaveThres = 50
    self.enterThres = 230

    #Dimenstions in meter
    self.widthM = 0.15
    self.lengthM = 0.30

    #Dimenstions in pixel
    self.widthP = 205
    self.leftBorder = 3
    self.rightBorder = self.leftBorder + self.widthP

    self.orderList = []
    self.pBrickList = []
    self.brickList = []
    self.trashList = []
    self.ordersRed = 2
    self.ordersBlue = 1
    self.ordersYellow = 1
    self.avgSpeed = 0
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self, data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cv_image.shape
  

#------------------------------------------------

    print "Printing length of bricklist: "
    print str(len(self.brickList))
    test = False



    img = cv_image	# imread('frame0001.jpg', CV_32FC1)  # ("src", img)

    colorRange = 10

    crop_img = img[0:300, 220:460]
    img = crop_img
    img = GaussianBlur(img, (5, 5), 0)

    redHue = 0
    blueHue = 110
    yellowHue = 26
    intencity = 70
    hsv = cvtColor(img, COLOR_BGR2HSV)  # convert to hsv

    #Color thresholds in HSV space
    lower_thrs_red = np.array([redHue,100,intencity-10])
    upper_thrs_red = np.array([redHue+colorRange,255,255])

    lower_thrs_blue = np.array([blueHue-colorRange,100,intencity])
    upper_thrs_blue = np.array([blueHue+colorRange,255,255])


    lower_thrs_yellow = np.array([yellowHue-colorRange,100,intencity])
    upper_thrs_yellow= np.array([yellowHue+colorRange,255,255])

    #Get binary images
    maskRed = inRange(hsv, lower_thrs_red, upper_thrs_red)
    maskBlue = inRange(hsv, lower_thrs_blue, upper_thrs_blue)
    maskYellow = inRange(hsv, lower_thrs_yellow, upper_thrs_yellow)

    mask = maskRed + maskBlue + maskYellow
    #imshow("mask", mask)


    contoursRed, hierarchy = findContours(maskRed, RETR_LIST, CHAIN_APPROX_SIMPLE)
    contoursBlue, hierarchy = findContours(maskBlue, RETR_LIST, CHAIN_APPROX_SIMPLE)
    contoursYellow, hierarchy = findContours(maskYellow, RETR_LIST, CHAIN_APPROX_SIMPLE)

    allContours = contoursBlue + contoursRed + contoursYellow

    drawContours(mask, allContours, -1, (255, 0, 255), 2)
    #imshow("Contours", mask)

    #Get the recrangles
   # print "Red contours"
    rectRed, bricksRed = getBricks(contoursRed, "Red")
   # print "Blue contours"
    rectBlue, bricksBlue = getBricks(contoursBlue, "Blue")
   # print "Yellow contours"
    rectYellow, bricksYellow = getBricks(contoursYellow, "Yellow")


    for rect in rectRed:
        box = cv.BoxPoints(rect)
        box = np.int0(box)
        drawContours(img, [box], -1, (0, 0, 255), 2)
    for rect in rectBlue:
        box = cv.BoxPoints(rect)
        box = np.int0(box)
        drawContours(img, [box], -1, (255, 0, 0), 2)
    for rect in rectYellow:
        box = cv.BoxPoints(rect)
        box = np.int0(box)
        drawContours(img, [box], -1, (0, 255, 255), 2)
  
##    imshow("Show", img)

    #Horisontal lines
    line(img, (0, self.enterThres), (240, self.enterThres),(255,0,0),1)
    line(img, (0, self.leaveThres), (240, self.leaveThres),(255,0,0),1)

    #Vertical lines
    line(img, (self.leftBorder, 0), (self.leftBorder, 300),(0,255,0),1)
    line(img, (self.rightBorder, 0), (self.rightBorder, 300),(0,255,0),1)

    self.pBrickList = bricksRed + bricksBlue + bricksYellow

    #Find matches
    for brick in self.brickList:
        for pBrick in self.pBrickList:
            tempX = abs(brick.x-pBrick.x)
            tempY = abs(brick.y - pBrick.y)
            if tempX < 10 and tempY < 10:
                brick.addPos(pBrick.x, pBrick.y)
                self.pBrickList.remove(pBrick)


    #Find new bricks
    for pBrick in self.pBrickList:
        if pBrick.y > self.enterThres:
            self.brickList.append(pBrick)

    #Add bricks to orderList or trashList
    for brick in self.brickList:
        if brick.y < self.leaveThres:
            brick.timeEnd = rospy.get_time()
            brick.setEndPos(brick.x-self.leftBorder, brick.y)
            brick.calcSpeed(self.enterThres-self.leaveThres, self.lengthM)
            brick.setXPos(self.widthP, self.widthM)

            if brick.color == "Red" and self.ordersRed != 0:
                self.orderList.append(brick)
                self.ordersRed = self.ordersRed - 1
                print "Adding brick to orderlist"
            elif brick.color == "Blue" and self.ordersBlue != 0:
                self.orderList.append(brick)
                self.ordersBlue = self.ordersBlue - 1
                print "Adding brick to orderlist"
            elif brick.color == "Yellow" and self.ordersYellow != 0:
                self.orderList.append(brick)
                self.ordersYellow = self.ordersYellow - 1
                print "Adding brick to orderlist"
            else:
                self.trashList.append(brick)
                print "Adding brick to trashList"

            self.brickList.remove(brick)

#    for brick in brickList:
 #       print brick.getInfo()

    pBrickList = []
    print "Printing order list: "
    for brick in self.orderList:
        print brick.getInfo()

    print "Printing trash list: "
    for brick in self.trashList:
        print brick.getInfo() + "\n"

    #Make a temp list that contains both order and trashlist
    tempList = self.trashList + self.orderList
    speedList = []
    if len(tempList) != 0:
        tempSpeed = 0
        for brick in tempList:
            speedList.append(brick.getSpeed())
            tempSpeed = tempSpeed + brick.getSpeed()
        self.avgSpeed = tempSpeed/len(tempList)

    print "Avg speed: " + str(self.avgSpeed)
    if len(speedList) != 0:
        speedList.sort()
        medianSpeed = speedList[len(speedList)/2]
        print "Median speed: " + str(medianSpeed)
#------------------------------------------------

    cv2.imshow("Image window", img)
    cv2.waitKey(3)
#-------------------publish list of bricks--------------------------------------------------------------------------------

    #b = LegoBrick()

    pub = rospy.Publisher("brick", Num)#, queue_size=10
    a = Num()
    a.x = 12
    a.info = "hej"
    pub.publish(a)

#-------------------publish list of bricks--------------------------------------------------------------------------------


    try:
#      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


