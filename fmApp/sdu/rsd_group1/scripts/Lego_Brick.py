__author__ = 'ditlev'

class LegoBrick:


    listOfPos = []
    timeStart = 0
    timeEnd = 0
    startX = 0
    startY = 0
    endX = 100
    endY = 100
    x = 0
    y = 0

    def updateTime(self, time):
        self.lastUpdateTime = time
        return

    def getTimeSinceUpdate(self, currentTime):
        return currentTime - self.lastUpdateTime

    def addPos(self, x, y):
        self.x = x
        self.y = y
        self.listOfPos.append([x, y])
        return

    def calcSpeed(self, lengthInPixel, lengthInM, widthInPixel, widthInM):
        self.speed = (((((self.startX-self.endX)/widthInPixel)*widthInM)**2 + (((self.startY-self.endY)/lengthInPixel)*lengthInM)**2)**0.5) /(self.timeEnd-self.timeStart)
        return

    def setXPos(self, xWidthP, xWidthM,):
        self.xPosOnBelt = (self.endX/xWidthP)*xWidthM
        return

    def setEndPos(self, x, y,):
        self.endX = x
        self.endY = y
        return

    def getSpeed(self):
        return self.speed
        return

    def __init__(self, xinp, yinp, anginp, color, time, width, height):
        self.speed = 0
        self.color = color
        self.timeStart = time
        self.x = xinp
        self.y = yinp
        self.startX = xinp
        self.startY = yinp
        self.addPos(self.x, self.y)
        self.angle = anginp
        self.xPosOnBelt = 0
        self.lastUpdateTime = time
        self.width = width
        self.height = height
        return

    def getInfo(self):
        infoAsString = "Color: " + str(self.color) + "\n StartX: " + str(self.startX) + \
                    "\n StartY: " + str(self.startY) + "\n EndX: " + str(self.endX) + \
                    "\n EndY: " + str(self.endY) + "\n angle: " + str(self.angle) + \
                    "\n Timestamp: " + str(self.timeStart) + \
                    "\n Speed: " + str(self.speed) + "\n X on belt: " + str(self.xPosOnBelt) + \
                    "\n Width: " + str(self.width) + "\n Height: " + str(self.height)
        return infoAsString