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
from rsd_group1.msg import Log
import xmlsettings



def publishToLog(level, code, logMessage):
    #-------------------publish list of bricks---------------------------------------------------------------------
    #Node ID = 2
    #level = 0 = debug, 1 = operation
    logEntry = Log()
    logEntry.CodeID = code
    logEntry.NodeID = 2
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

def publishBrick(brick):
    #-------------------publish list of bricks---------------------------------------------------------------------

    a = Num()

    a.color = brick.color
    a.time = brick.timeEnd
    a.angle = brick.angle
    a.x = brick.xPosOnBelt
    a.speed = brick.speed

    pub.publish(a)
    publishToLog("operation", 11, brick.color.lower()+" brick published")
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

    #Area requirements for the different colored bricks
    if color == "Blue":
        areaReqMin = blueMinArea
        areaReqMax = blueMaxArea
    elif color == "Red":
        areaReqMin = redMinArea
        areaReqMax = redMaxArea
    elif color == "Yellow":
        areaReqMin = yellowMinArea
        areaReqMax = yellowMaxArea
    else:
        areaReqMin = blueMinArea
        areaReqMax = yellowMaxArea

    for cnt in contours:
        #Check if the bricks area is plausible
        if contourArea(cnt) > areaReqMin and contourArea(cnt) < areaReqMax:
            #Rectangle data is (x, y) (width, height) (angle)
            rect = minAreaRect(cnt)
            bricks.append(rect)
            sort(rect[1], False)

            publishToLog("debug", 12, color+" brick detected, it's area is "+str(contourArea(cnt)))


            #Check if width is larger than length
            #if so, rotate 90 degrees
            if (rect[1][0] > rect[1][1]):
                tempAngle = rect[2] + 90
            else:
                tempAngle = rect[2]

            tempList.append(LegoBrick(rect[0][0],rect[0][1],tempAngle,color,rospy.get_time(), rect[1][0], rect[1][1]))
        else:
            pass
            publishToLog("debug", 12, "Potential "+color+" brick detected, it's area "+str(contourArea(cnt))+" not in range of " + str(areaReqMin)+ " to " + str(areaReqMax))
    return bricks, tempList

def on_dimensionbar(args):

    global lengthM
    global widthM

    lengthM = getTrackbarPos('length', 'dimBar')/1000
    widthM = getTrackbarPos('width', 'dimBar')/1000

    config.put('dimensions/length', redHue)
    config.put('dimensions/width', blueHue)

    config.save()
    return

def on_areabar(args):

    global blueMinArea
    global blueMaxArea

    global yellowMinArea
    global yellowMaxArea

    global redMinArea
    global redMaxArea

    blueMaxArea = getTrackbarPos('blueMaxArea', 'Area Bar')
    blueMinArea = getTrackbarPos('blueMinArea', 'Area Bar')

    yellowMaxArea = getTrackbarPos('yellowMaxArea', 'Area Bar')
    yellowMinArea = getTrackbarPos('yellowMinArea', 'Area Bar')


    redMaxArea = getTrackbarPos('redMaxArea', 'Area Bar')
    redMinArea = getTrackbarPos('redMinArea', 'Area Bar')


    config.put('brickAreas/blueMinArea', blueMinArea)
    config.put('brickAreas/blueMaxArea', blueMaxArea)

    config.put('brickAreas/yellowMinArea', yellowMinArea)
    config.put('brickAreas/yellowMaxArea', yellowMaxArea)

    config.put('brickAreas/redMinArea', redMinArea)
    config.put('brickAreas/redMaxArea', redMaxArea)

    config.save()
    return

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
    config = xmlsettings.XMLSettings('/home/robot/roswork/src/fmApp/sdu/rsd_group1/scripts/config.xml')


    #initialize border values
    global leftBorder
    global rightBorder
    global enterThres
    global leaveThres

    leftBorder = config.get('imageBorders/leftBorder', 200)
    rightBorder = config.get('imageBorders/rightBorder', 500)
    enterThres = config.get('imageBorders/enterThres', 250)
    leaveThres = config.get('imageBorders/leaveThres', 50)

    #Dimenstions in meter
    global widthM
    global lengthM

    widthM = config.get('dimensions/width', 0.15)
    lengthM = config.get('dimensions/length', 0.09)

    #crop settings
    global leftCrop
    global rightCrop
    global topCrop
    global botCrop

    leftCrop = config.get('imageCrop/leftCrop', 0)
    rightCrop = config.get('imageCrop/rightCrop', 1000)
    topCrop = config.get('imageCrop/topCrop', 0)
    botCrop = config.get('imageCrop/botCrop', 800)

    #Hue settings
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

    #brick areas
    global blueMinArea
    global blueMaxArea
    global yellowMinArea
    global yellowMaxArea
    global redMinArea
    global redMaxArea

    blueMinArea = config.get('brickAreas/blueMinArea', 2000)
    blueMaxArea = config.get('brickAreas/blueMaxArea', 6500)
    redMinArea = config.get('brickAreas/redMinArea', 6000)
    redMaxArea = config.get('brickAreas/redMaxArea', 10000)
    yellowMinArea = config.get('brickAreas/yellowMinArea', 10000)
    yellowMaxArea = config.get('brickAreas/yellowMaxArea', 24000)

    self.pBrickList = []
    self.brickList = []

    self.avgSpeed = 0
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    rospy.Subscriber("vision_config", String, self.logSubCallback)
    self.config_mode = "false"
    #windows

  def logSubCallback(self, conf):
    print "config callback called, conf recieved is:"
    print conf.data

    if conf.data == "true":
        self.config_mode = conf.data

        publishToLog("operation", 13, "Entering configuration mode")

        namedWindow("trackbars", WINDOW_NORMAL)
        namedWindow("hsvBar", WINDOW_NORMAL)
        namedWindow("Show", WINDOW_NORMAL)
        namedWindow("Red mask", WINDOW_NORMAL)
        namedWindow("Blue mask", WINDOW_NORMAL)
        namedWindow("Yellow mask", WINDOW_NORMAL)
        namedWindow("dimBar", WINDOW_NORMAL)
        namedWindow("Area bar", WINDOW_NORMAL)

        #Adjustment of brick area
        createTrackbar('blueMinArea', 'Area bar', int(blueMinArea), 6000, on_areabar)
        createTrackbar("blueMaxArea", 'Area bar', int(blueMaxArea), 9000, on_areabar)
        createTrackbar('redMinArea', 'Area bar', int(redMinArea), 12000, on_areabar)
        createTrackbar("redMaxArea", 'Area bar', int(redMaxArea), 20000, on_areabar)
        createTrackbar('yellowMinArea', 'Area bar', int(yellowMinArea), 20000, on_areabar)
        createTrackbar("yellowMaxArea", 'Area bar', int(yellowMaxArea), 30000, on_areabar)

        #Adjustment of dimensions
        createTrackbar('length', 'dimBar', int(lengthM*1000), 2000, on_dimensionbar)
        createTrackbar("width", 'dimBar', int(widthM*1000), 2000, on_dimensionbar)

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

    elif conf.data == "false":
        publishToLog("operation", 13, "Exiting configuration mode")
        self.config_mode = conf.data
        destroyAllWindows()
    else:
        pass
    return

  def callback(self, data):

    #convert the image to opencv format
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        publishToLog("operation", 14, "An error occoured while reading image from camera: "+str(e))
        print e

#-----------------------------------------------

    #rotate image 90 degrees by transposing and flipping
    try:
        img = transpose(cv_image)
    finally:
        pass
    flip(img, 0, img)

    #Crop image according to borders
    crop_img = img[topCrop:botCrop, leftCrop:rightCrop]
    img = crop_img

    #Blur image, 5 iterations with 9x9 mask
    img = GaussianBlur(img, (9, 9), 5)

    hsv = cvtColor(img, COLOR_BGR2HSV)  # convert to hsv

    #Color thresholds in HSV space
    #Parameters comes from trackbar / XML file
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

    #Make kernel for use in morhpology filter
    kernel = np.ones((7,7), np.int8)

    #Set number of iterations for morph
    numOfIter = 5

    #Apply open filter, enhances gaps without completely eroding smaller bricks
    maskRed = morphologyEx(maskRed, MORPH_OPEN, kernel, iterations=numOfIter)
    maskBlue = morphologyEx(maskBlue, MORPH_OPEN, kernel, iterations=numOfIter)
    maskYellow = morphologyEx(maskYellow, MORPH_OPEN, kernel, iterations=numOfIter)

    #Final erosion to fully seperate bricks
    maskRed = morphologyEx(maskRed, MORPH_ERODE, kernel, iterations=numOfIter)
    maskBlue = morphologyEx(maskBlue, MORPH_ERODE, kernel, iterations=numOfIter)
    maskYellow = morphologyEx(maskYellow, MORPH_ERODE, kernel, iterations=numOfIter)

    #If config mode is true, show the individually masked images
    if self.config_mode == "true":
        imshow("Red mask", maskRed)
        imshow("Blue mask", maskBlue)
        imshow("Yellow mask", maskYellow)
    else:
        pass

    #Find contours
    contoursRed, hierarchy = findContours(maskRed, RETR_LIST, CHAIN_APPROX_SIMPLE)
    contoursBlue, hierarchy = findContours(maskBlue, RETR_LIST, CHAIN_APPROX_SIMPLE)
    contoursYellow, hierarchy = findContours(maskYellow, RETR_LIST, CHAIN_APPROX_SIMPLE)

    #Find minimum area rectangels on the contours
    rectRed, bricksRed = getBricks(contoursRed, "Red")
    rectBlue, bricksBlue = getBricks(contoursBlue, "Blue")
    rectYellow, bricksYellow = getBricks(contoursYellow, "Yellow")



    #Draw the contours and lines
    for rect in rectRed:
        box = cv.BoxPoints(rect)
        box = np.int0(box)
        drawContours(img, [box], -1, (0, 0, 255), 5)
    for rect in rectBlue:
        box = cv.BoxPoints(rect)
        box = np.int0(box)
        drawContours(img, [box], -1, (255, 0, 0), 5)
    for rect in rectYellow:
        box = cv.BoxPoints(rect)
        box = np.int0(box)
        drawContours(img, [box], -1, (0, 255, 255), 5)

    #Horisontal lines
    line(img, (0, enterThres), (rightCrop - leftCrop, enterThres),(255,0,0),1)
    line(img, (0, leaveThres), (rightCrop - leftCrop, leaveThres),(255,0,0),1)

    #Vertical lines
    line(img, (leftBorder, 0), (leftBorder, botCrop - topCrop),(0,255,0),1)
    line(img, (rightBorder, 0), (rightBorder, botCrop - topCrop),(0,255,0),1)

    #If the config mode is true, show the rectangles
    if self.config_mode == "true":
        imshow("Show", img)
    else:
        pass

    #Add the found bricks to a preliminary bricklist
    self.pBrickList = bricksRed + bricksBlue + bricksYellow

    #Find matches between already found bricks and preliminary bricks
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
            publishToLog("operation",15, brick.color.lower()+" brick removed by timeout")
            print "Following brick removed by timeout:"
            print brick.getInfo()
            self.brickList.remove(brick)

    #Find new bricks
    #New brick is found if a preliminary brick haven't been matched and is located between the enter and leave thresholds
    for pBrick in self.pBrickList:
        if pBrick.y < enterThres and pBrick.y > leaveThres:
            publishToLog("operation", 16, pBrick.color.lower()+" brick detected")
            self.brickList.append(pBrick)

    #Publish found bricks when they leave the leaveThreshold
    for brick in self.brickList:
        if brick.y < leaveThres:
            print "Brick found!"
            brick.timeEnd = rospy.get_time()
            brick.setEndPos(brick.x-leftBorder, brick.y)
            brick.calcSpeed(enterThres-leaveThres, lengthM, leftBorder - rightBorder, widthM)
            brick.setXPos(leftBorder - rightBorder, widthM)
            publishBrick(brick)
            self.brickList.remove(brick)

    #Publish the image with drawn contours and bricks
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError, e:
      print e

    waitKey(3)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    publishToLog("operation", 17, "Vision node sucessfully initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        publishToLog("operation", 18, "Vision node shutting down")
    destroyAllWindows()


if __name__ == '__main__':
    pub = rospy.Publisher('brick', Num, queue_size=100)
    pubLog = rospy.Publisher('logging', Log, queue_size=100)
    main(sys.argv)


