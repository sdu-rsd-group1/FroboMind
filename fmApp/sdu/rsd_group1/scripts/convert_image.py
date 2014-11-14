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






def getBricks(contours, color):
    #Get a list of contours and extracts the bricks
    bricks = []
    tempList = []
    for cnt in contours:
        if contourArea(cnt) > 300:
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
            # print "Angle: " + str(rect[2]) + " Deg"

            tempList.append(LegoBrick(rect,color,1))

    return bricks, tempList


class image_converter:

  def __init__(self):
    self.orderList = []
    self.pBrickList = []
    self.brickList = []
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
        if pBrick.y > 100:
            self.brickList.append(pBrick)

    #Add bricks to orderList
    for brick in self.brickList:
        if brick.y < 50:
            brick.timeEnd = brick.timeStart + 1
            brick.setEndPos(brick.x, brick.y)
            brick.calcSpeed()
            self.orderList.append(brick)
            print "Adding brick to order list: "
            print str(brick.getInfo())
            self.brickList.remove(brick)

#    for brick in brickList:
 #       print brick.getInfo()

    pBrickList = []
    print "Printing order list: "
    for brick in self.orderList:
        print brick.getInfo()


#------------------------------------------------
    cv2.imshow("Image window", img)
    cv2.waitKey(3)

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

