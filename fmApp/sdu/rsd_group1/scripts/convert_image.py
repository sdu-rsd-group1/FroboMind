#!/usr/bin/env python
import roslib
roslib.load_manifest('rsd_group1')
import sys
import rospy
from cv2 import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from Lego_Brick import LegoBrick
from rsd_group1.msg import Num
import xmlsettings



def publishBrick(brick):
    #-------------------publish list of bricks---------------------------------------------------------------------

    pub = rospy.Publisher("brick", Num)
    a = Num()

    a.color = brick.color
    a.time = brick.timeEnd
    a.angle = brick.angle
    a.x = brick.xPosOnBelt
    a.speed = brick.speed

    pub.publish(a)

    print "Following brick published:"
    print brick.getInfo()

    return
#-------------------publish list of bricks------------------------------------------------------------------------


def callBack(args):

    return 0

def getBricks(contours, color):
    #Get a list of contours and extracts the bricks
    bricks = []
    tempList = []

    areaReq = 0
    if color == "Blue":
        areaReqMin = 300
        areaReqMax = 1000000
    elif color == "Red":
        areaReqMin = 300
        areaReqMax = 1000000
    elif color == "Yellow":
        areaReqMin = 300
        areaReqMax = 1000000
    else:
        areaReqMin = 300
        areaReqMax = 1000000

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

def on_colorbar(args):

    global redHue
    global blueHue
    global yellowHue
    global redSaturation
    global blueSaturation
    global yellowSaturation
    global colorRange

    global redIntensity
    global blueIntensity
    global yellowIntensity

    redHue = getTrackbarPos('redHue', 'hsvBar')
    blueHue = getTrackbarPos('blueHue', 'hsvBar')
    yellowHue = getTrackbarPos('yellowHue', 'hsvBar')
    redSaturation = getTrackbarPos('redSaturation', 'hsvBar')
    blueSaturation = getTrackbarPos('blueSaturation', 'hsvBar')
    yellowSaturation = getTrackbarPos('yellowSaturation', 'hsvBar')
    redIntensity = getTrackbarPos('redIntensity', 'hsvBar')
    blueIntensity = getTrackbarPos('blueIntensity', 'hsvBar')
    yellowIntensity = getTrackbarPos('yellowIntensity', 'hsvBar')
    colorRange = getTrackbarPos('colorRange', 'hsvBar')

    config.put('hsvSettings/redHue', redHue)
    config.put('hsvSettings/blueHue', blueHue)
    config.put('hsvSettings/yellowHue', yellowHue)
    config.put('hsvSettings/redSaturation', redSaturation)
    config.put('hsvSettings/blueSaturation', blueSaturation)
    config.put('hsvSettings/yellowSaturation', yellowSaturation)
    config.put('hsvSettings/colorRange', colorRange)

    config.put('hsvSettings/redIntensity', redIntensity)
    config.put('hsvSettings/blueIntensity', blueIntensity)
    config.put('hsvSettings/yellowIntensity', yellowIntensity)

    config.save()
    return

def on_trackbar(args):
    #crop settings
    global leftCrop
    global rightCrop
    global topCrop
    global botCrop

    global leftBorder
    global rightBorder
    global enterThres
    global leaveThres


    global config

    leftBorder = getTrackbarPos('leftBorder', 'trackbars')
    rightBorder = getTrackbarPos('rightBorder', 'trackbars')
    enterThres = getTrackbarPos('enterThres', 'trackbars')
    leaveThres = getTrackbarPos('leaveThres', 'trackbars')

    leftCrop = getTrackbarPos('LeftCrop', 'trackbars')
    rightCrop = getTrackbarPos('RightCrop', 'trackbars')
    topCrop = getTrackbarPos('TopCrop', 'trackbars')
    botCrop = getTrackbarPos('BotCrop', 'trackbars')


    config.put('imageBorders/leftBorder', leftBorder)
    config.put('imageBorders/rightBorder', rightBorder)
    config.put('imageBorders/enterThres', enterThres)
    config.put('imageBorders/leaveThres', leaveThres)

    config.put('imageCrop/leftCrop', leftCrop)
    config.put('imageCrop/rightCrop', rightCrop)
    config.put('imageCrop/topCrop', topCrop)
    config.put('imageCrop/botCrop', botCrop)

    config.save()

    return

class image_converter:

  def __init__(self):

    global config
    config = xmlsettings.XMLSettings('/home/ditlev/roswork/src/fmApp/sdu/rsd_group1/scripts/config.xml')


    global leftBorder
    global rightBorder
    global enterThres
    global leaveThres


    leftBorder = config.get('imageBorders/leftBorder', 200)
    rightBorder = config.get('imageBorders/rightBorder', 500)
    enterThres = config.get('imageBorders/enterThres', 250)
    leaveThres = config.get('imageBorders/leaveThres', 50)


    #Dimenstions in meter
    self.widthM = 0.15
    self.lengthM = 0.09

    #Dimenstions in pixel
    self.widthP = leftBorder - rightBorder

    #crop settings
    global leftCrop
    global rightCrop
    global topCrop
    global botCrop

    leftCrop = config.get('imageCrop/leftCrop', 0)
    rightCrop = config.get('imageCrop/rightCrop', 1000)
    topCrop = config.get('imageCrop/topCrop', 0)
    botCrop = config.get('imageCrop/botCrop', 800)

    global redHue
    global blueHue
    global yellowHue
    global redSaturation
    global blueSaturation
    global yellowSaturation
    global colorRange
    global redIntensity
    global blueIntensity
    global yellowIntensity


    redHue = config.get('hsvSettings/redHue', 0)
    blueHue = config.get('hsvSettings/blueHue', 110)
    yellowHue = config.get('hsvSettings/yellowHue', 26)
    redSaturation = config.get('hsvSettings/redSaturation', 100)
    blueSaturation = config.get('hsvSettings/blueSaturation', 100)
    yellowSaturation = config.get('hsvSettings/yellowSaturation', 100)
    colorRange = config.get('hsvSettings/colorRange', 10)
    redIntensity = config.get('hsvSettings/redIntensity', 104)
    blueIntensity = config.get('hsvSettings/blueIntensity', 50)
    yellowIntensity = config.get('hsvSettings/yellowIntensity', 104)


    self.pBrickList = []
    self.brickList = []

    self.avgSpeed = 0
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    #windows
    namedWindow("trackbars", 0)
    namedWindow("hsvBar", 0)
    namedWindow("Show", WINDOW_NORMAL)
    namedWindow("Red mask", WINDOW_NORMAL)
    namedWindow("Blue mask", WINDOW_NORMAL)
    namedWindow("Yellow mask", WINDOW_NORMAL)


    #Adjustment of hsv
    createTrackbar('redHue', 'hsvBar', redHue, 255, on_colorbar)
    createTrackbar("blueHue", 'hsvBar', blueHue, 255, on_colorbar)
    createTrackbar('yellowHue', 'hsvBar', yellowHue, 255, on_colorbar)
    createTrackbar('redSaturation', 'hsvBar', redSaturation, 255, on_colorbar)
    createTrackbar('blueSaturation', 'hsvBar', blueSaturation, 255, on_colorbar)
    createTrackbar('yellowSaturation', 'hsvBar', yellowSaturation, 255, on_colorbar)
    createTrackbar('redIntensity', 'hsvBar', redIntensity, 255, on_colorbar)
    createTrackbar('blueIntensity', 'hsvBar', blueIntensity, 255, on_colorbar)
    createTrackbar('yellowIntensity', 'hsvBar', yellowIntensity, 255, on_colorbar)
    createTrackbar('colorRange', 'hsvBar', colorRange, 255, on_colorbar)

    #Adjustment of lines
    createTrackbar('leaveThres', 'trackbars', leaveThres, enterThres, on_trackbar)
    createTrackbar('enterThres', 'trackbars', enterThres, botCrop-topCrop, on_trackbar)
    createTrackbar('leftBorder', 'trackbars', leftBorder, rightBorder, on_trackbar)
    createTrackbar('rightBorder', 'trackbars', rightBorder, rightCrop - leftCrop, on_trackbar)

    #Adjustment of Crop
    createTrackbar('LeftCrop', 'trackbars', leftCrop, rightCrop, on_trackbar)
    createTrackbar("RightCrop", 'trackbars', rightCrop, 1080, on_trackbar)
    createTrackbar('TopCrop', 'trackbars', topCrop, botCrop, on_trackbar)
    createTrackbar('BotCrop', 'trackbars', botCrop, 1920, on_trackbar)

  def callback(self, data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

#-----------------------------------------------
    try:
        img = transpose(cv_image)
    finally:
        pass
    flip(img, 0, img)

    #crop_img = img[self.topCrop:self.botCrop, self.leftCrop:self.rightCrop]
    crop_img = img[topCrop:botCrop, leftCrop:rightCrop]
    img = crop_img
    img = GaussianBlur(img, (9, 9), 5)

    hsv = cvtColor(img, COLOR_BGR2HSV)  # convert to hsv

    #Color thresholds in HSV space

    lower_thrs_red = np.array([0,redSaturation,redIntensity])
    upper_thrs_red = np.array([redHue+colorRange,255,255])

    lower_thrs_blue = np.array([blueHue-colorRange,blueSaturation,blueIntensity])
    upper_thrs_blue = np.array([blueHue+colorRange,255,255])


    lower_thrs_yellow = np.array([yellowHue-colorRange,yellowSaturation,yellowIntensity])
    upper_thrs_yellow= np.array([yellowHue+colorRange,255,255])

    #Get binary images


    maskRed = inRange(hsv, lower_thrs_red, upper_thrs_red)
    maskBlue = inRange(hsv, lower_thrs_blue, upper_thrs_blue)
    maskYellow = inRange(hsv, lower_thrs_yellow, upper_thrs_yellow)

    kernel = np.ones((7,7), np.int8)
    numOfIter = 5

    maskRed = morphologyEx(maskRed, MORPH_OPEN, kernel, iterations=numOfIter)
    maskBlue = morphologyEx(maskBlue, MORPH_OPEN, kernel, iterations=numOfIter)
    maskYellow = morphologyEx(maskYellow, MORPH_OPEN, kernel, iterations=numOfIter)

    maskRed = morphologyEx(maskRed, MORPH_ERODE, kernel, iterations=numOfIter)
    maskBlue = morphologyEx(maskBlue, MORPH_ERODE, kernel, iterations=numOfIter)
    maskYellow = morphologyEx(maskYellow, MORPH_ERODE, kernel, iterations=numOfIter)



    mask = maskRed + maskBlue + maskYellow
    imshow("Red mask", maskRed)
    imshow("Blue mask", maskBlue)
    imshow("Yellow mask", maskYellow)
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


    #Horisontal lines
    line(img, (0, enterThres), (rightCrop - leftCrop, enterThres),(255,0,0),1)
    line(img, (0, leaveThres), (rightCrop - leftCrop, leaveThres),(255,0,0),1)

    #Vertical lines
    line(img, (leftBorder, 0), (leftBorder, botCrop - topCrop),(0,255,0),1)
    line(img, (rightBorder, 0), (rightBorder, botCrop - topCrop),(0,255,0),1)

    imshow("Show", img)
    self.pBrickList = bricksRed + bricksBlue + bricksYellow

    #Find matches
    for brick in self.brickList:
        for pBrick in self.pBrickList:
            if pBrick.color == brick.color:
                tempX = abs(brick.x-pBrick.x)
                tempY = abs(brick.y - pBrick.y)
                if tempX < 20 and tempY < 40:
                    brick.addPos(pBrick.x, pBrick.y)
                    brick.updateTime(rospy.get_time())
                    self.pBrickList.remove(pBrick)

        #Timeout, if a tracked brick haven't been updated in 2 sec it is discarded
        if brick.getTimeSinceUpdate(rospy.get_time()) > 2:
            print "Following brick removed by timeout:"
            print brick.getInfo()
            self.brickList.remove(brick)


    #Find new bricks
    for pBrick in self.pBrickList:
        if pBrick.y > enterThres:
            self.brickList.append(pBrick)

    #Publish found bricks
    for brick in self.brickList:
        if brick.y < leaveThres:
            print "Brick found!"
            brick.timeEnd = rospy.get_time()
            brick.setEndPos(brick.x-leftBorder, brick.y)
            brick.calcSpeed(enterThres-leaveThres, self.lengthM, self.widthP, self.widthM)
            brick.setXPos(self.widthP, self.widthM)
            publishBrick(brick)
            self.brickList.remove(brick)

    waitKey(3)



    try:
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
    destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


