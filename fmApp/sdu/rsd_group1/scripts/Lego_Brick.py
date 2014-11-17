__author__ = 'ditlev'

class LegoBrick:


    listOfPos = []
    speed = 0
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

    def calcSpeed(self):
        self.speed = ((pow(self.startX-self.endX, 2) + pow(self.startY-self.endY, 2))**0.5) /(self.timeEnd-self.timeStart)

    def setEndPos(self, x, y,):
        self.endX = x
        self.endY = y

    def __init__(self, xinp, yinp, anginp, color, time):
        self.color = color
        self.timeStart = time
        self.x = xinp
        self.y = yinp
        self.startX = xinp
        self.startY = yinp
        self.addPos(self.x, self.y)
        self.angle = anginp
        print "Adding angle to brick: " + str(self.angle)

    def getInfo(self):
        return "Color: " + str(self.color) + "\n StartX: " + str(self.startX) + \
                    "\n StartY: " + str(self.startY) + "\n EndX: " + str(self.endX) + \
                    "\n EndY: " + str(self.endY) + "\n angle: " + str(self.angle) + \
                    "\n Timestamp: " + str(self.timeStart) + \
                    "\n Speed: " + str(self.speed)
