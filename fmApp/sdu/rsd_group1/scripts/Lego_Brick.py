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

    def addPos(self, x, y):
        self.x = x
        self.y = y
        self.listOfPos.append([x, y])

    def calcSpeed(self, lengthInPixel, lengthInM):
        self.speed = (((self.startY-self.endY)/lengthInPixel)*lengthInM) /(self.timeEnd-self.timeStart)


    def setXPos(self, xWidthP, xWidthM,):
        self.xPosOnBelt = (self.endX/xWidthP)*xWidthM

    def setEndPos(self, x, y,):
        self.endX = x
        self.endY = y

    def getSpeed(self):
        return self.speed

    def __init__(self, xinp, yinp, anginp, color, time):
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


    def getInfo(self):
        return "Color: " + str(self.color) + "\n StartX: " + str(self.startX) + \
                    "\n StartY: " + str(self.startY) + "\n EndX: " + str(self.endX) + \
                    "\n EndY: " + str(self.endY) + "\n angle: " + str(self.angle) + \
                    "\n Timestamp: " + str(self.timeStart) + \
                    "\n Speed: " + str(self.speed) + "\n X on belt: " + str(self.xPosOnBelt)
