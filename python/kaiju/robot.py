import numpy
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from descartes import PolygonPatch

AlphaArmLength = 7.4 #mm
AlphaRange = [0, 360]
BetaRange = [0,180]
BetaArmLength = 13 #mm (15 is quoted length but we're placing the fiber 2 mm from end)
BetaArmWidth = 4 #mm
MinTargSeparation = 8 #mm
# length mm along beta for which a collision cant happen
BetaArmLengthSafe = BetaArmLength / 2.0
Pitch = 22.4 # mm fudge the 01 for neighbors
MinReach = BetaArmLength - AlphaArmLength
MaxReach = BetaArmLength + AlphaArmLength

# https://internal.sdss.org/trac/as4/wiki/FPSLayout


def hexFromDia(nDia):
    """
    inputs:
    nDia: is points along the equator of the hex, must be odd

    returns:
    xArr, yArr: both nd arrays of x and y coordinates for each
    position in the hex. Units are mm and the center of hex is 0,0
    """
    # first determine the xy positions for the center line
    nRad = int((nDia+1) / 2)
    xPosDia = numpy.arange(-nRad+1, nRad)*Pitch
    yPosDia = numpy.zeros(nDia)

    vertShift = numpy.sin(numpy.radians(60.0))*Pitch
    horizShift = numpy.cos(numpy.radians(60.0))*Pitch
    # populate the top half of the hex
    # determine number of rows above the diameter row
    xUpperHalf = []
    yUpperHalf = []
    for row in range(1,nRad):
        xPos = xPosDia[:-1*row] + row*horizShift
        yPos = yPosDia[:-1*row] + row*vertShift
        xUpperHalf.extend(xPos)
        yUpperHalf.extend(yPos)
    xUpperHalf = numpy.asarray(xUpperHalf).flatten()
    yUpperHalf = numpy.asarray(yUpperHalf).flatten()

    xLowerHalf = xUpperHalf[:]
    yLowerHalf = -1*yUpperHalf[:]

    xAll = numpy.hstack((xLowerHalf, xPosDia, xUpperHalf)).flatten()
    yAll = numpy.hstack((yLowerHalf, yPosDia, yUpperHalf)).flatten()
    # return arrays in a sorted order, rasterized. 1st position is lower left
    # last positon is top right
    ind = numpy.lexsort((xAll, yAll))
    return xAll[ind], yAll[ind]

class SketchyNeighbors(object):
    def __init__(self, robotA, robotB):
        """Provide a link between two robots with potential to collide
        """
        self.robotA = robotA
        self.robotB = robotB
        self._intersection = None
        self.swapTried = False
        self.id = set([robotA.id, robotB.id])

    @property
    def isCollided(self):
        return bool(self.intersection)

    @property
    def intersection(self):
        """return True if robots are collided
        """
        # first perform a quick check by drawing boxes
        # around the collision zone and see if they
        # are non-overlapping
        if self._intersection is None:
            # line intersection
            line1 = LineString([self.robotA.xySafeBetaFocal, self.robotA.xyFiberFocal]).buffer(BetaArmWidth/2.0, cap_style=3)
            line2 = LineString([self.robotB.xySafeBetaFocal, self.robotB.xyFiberFocal]).buffer(BetaArmWidth/2.0, cap_style=3)
            self._intersection = line1.intersection(line2)
            self.robotA._collisionBox = line1
            self.robotB._collisionBox = line2
        return self._intersection

    def fiberDist(self):
        """distance between fibers (end of beta arm)
        """
        return numpy.linalg.norm(self.robotA.xyFiber-self.robotB.xyFiber)

    def alphaDist(self):
        """distance between ends of alpha arms
        """
        return numpy.linalg.norm(self.robotA.xyAlpha-self.robotB.xyAlpha)

    def checkSwap(self):
        if self.intersection:
            x1,y1 = self.robotA.xyFiberFocal
            x2,y2 = self.robotB.xyFiberFocal
            self.swapTried = True
            # check that they can reach
            try:
                a1,b1 = self.robotB.getAlphaBetaFromFocalXY(x1,y1)
                a2,b2 = self.robotA.getAlphaBetaFromFocalXY(x2,y2)
                self.robotB.setAlphaBeta(a1,b1)
                self.robotA.setAlphaBeta(a2,b2)
                if not self.intersection:
                    print("swap worked")
                else:
                    print("swap still intersects")
                # self._intersection = None intersection is cleared in
                # set alpha beta
                # print('swap successful')
            except:
                pass
                # print("swap not valid")



class Robot(object):
    def __init__(self, robotID, alphaAng=0, betaAng=180, xFocal=None, yFocal=None):
        """
        robotID integer, unique to each robot
        Angles in degrees
        xy in mm
        """
        self.id = robotID
        ###############
        # store often computed stuff as underscored vars
        # populated by setter methods
        self._alphaRad = None
        self._betaRad = None
        self._cosAlpha = None
        self._sinAlpha = None
        self._cosAlphaBeta = None
        self._sinAlphaBeta = None
        self._xyFiberLocal = None
        self._xyAlphaLocal = None
        self._xySafeBetaLocal = None
        self.sketchyNeighbors = []
        self._collisionBox = None # shapely geom populated by neighbor
        ##################
        self.xyFocal = None
        if not None in [xFocal, yFocal]:
            self.setXYFocal(xFocal, yFocal)
        if not None in [alphaAng, betaAng]:
            self.setAlphaBeta(alphaAng, betaAng)

    def addSkectchyNeighbor(self, sketchyNeighbor):
        # maybe enforce that this neighbor doesn't already
        # exist?
        self.sketchyNeighbors.append(sketchyNeighbor)

    @property
    def alphaBeta(self):
        return numpy.array([numpy.degrees(self._alphaRad), numpy.degrees(self._betaRad)])

    def _preComputeTrig(self):
        # pre compute some stuff...
        self._cosAlpha = numpy.cos(self._alphaRad)
        self._sinAlpha = numpy.sin(self._alphaRad)
        self._cosAlphaBeta = numpy.cos(self._alphaRad+self._betaRad)
        self._sinAlphaBeta = numpy.sin(self._alphaRad+self._betaRad)

    def setAlphaBeta(self, alphaAng, betaAng):
        """angle are in degrees
        """
        assert AlphaRange[0]<=alphaAng<=AlphaRange[1], "alpha out of range"
        assert BetaRange[0] <=betaAng <= BetaRange[1], "beta out of range"
        self._alphaRad = numpy.radians(alphaAng)
        self._betaRad = numpy.radians(betaAng)
        self._preComputeTrig()
        # clear cached positions, if any
        self._xyAlphaLocal = None
        self._xySafeBetaLocal = None
        self._xyFiberLocal = None
        # clear any precomputed collisions
        self._collisionBox = None
        for n in self.sketchyNeighbors:
            n._intersection = None

    def setXYFocal(self, xFocal, yFocal):
        """position in focal plane mm
        """
        self.xyFocal = numpy.asarray([xFocal, yFocal])
        # clear cached positions, if any
        self._xyAlphaLocal = None
        self._xySafeBetaLocal = None
        self._xyFiberLocal = None

    def checkLocalReach(self,xLocal,yLocal):
        """return true if the robot can reach this position xy mm local coords
        """
        dist = numpy.sqrt(xLocal**2 + yLocal**2)
        return MinReach <= dist <= MaxReach

    def checkFocalReach(self,xFocal,yFocal):
        xLocal = self.xyFocal[0] - xFocal
        yLocal = self.xyFocal[1] - yFocal
        self.checkLocalReach(xLocal, yLocal)

    def setAlphaBetaRand(self):
        """Choose random alpha and beta angles
        such that the robot's workspace (xy) is uniformly sampled.

        random anululs sampling:
        https://ridlow.wordpress.com/2014/10/22/uniform-random-points-in-disk-annulus-ring-cylinder-and-sphere/
        """
        # pick r between rmin and rmax
        rPick = numpy.sqrt((MaxReach**2-MinReach**2)*numpy.random.sample() + MinReach**2)
        thetaPick = numpy.random.sample()*2*numpy.pi
        xLocal = rPick * numpy.cos(thetaPick)
        yLocal = rPick * numpy.sin(thetaPick)
        alphaAng, betaAng = self.getAlphaBetaFromLocalXY(xLocal,yLocal)
        self.setAlphaBeta(alphaAng, betaAng)

    def getAlphaBetaFromLocalXY(self,xLocal,yLocal):
        """x, y are mm, the fiber's local reference frame
        use law of cosines to solve for alpha beta
        http://mathworld.wolfram.com/LawofCosines.html
        """
        x,y = xLocal, yLocal
        # check that the xy pos is reachable
        assert self.checkLocalReach(x,y), "xy local is out of reach"

        # note: we only have right handed robots
        # because Beta is limited to 0-180
        # strategy: solve for angles using a side
        # with mag(xy) along x axis, then rotate
        # alpha by atan2(y,x) to get solution
        xyMag = numpy.linalg.norm([x,y])
        alphaAngRad = numpy.arccos((-BetaArmLength**2 + AlphaArmLength**2 + \
            xyMag**2)/(2*AlphaArmLength*xyMag))
        gammaAngRad = numpy.arccos((-xyMag**2 + AlphaArmLength**2 + \
            BetaArmLength**2)/(2*AlphaArmLength*BetaArmLength))
        # we only have right handed robot so use the -alphaAng
        alphaAngRad = -alphaAngRad
        betaAngRad = numpy.pi - gammaAngRad
        betaAngDeg = numpy.degrees(betaAngRad)
        # next rotate alpha angle from x axis to xy
        rotAng = numpy.arctan2(y,x) # defined -180 to 180
        alphaAngRad = alphaAngRad + rotAng
        # wrap to be between 0, 360 degrees
        alphaAngDeg = numpy.degrees(alphaAngRad)
        while alphaAngDeg < 0:
            alphaAngDeg += 360
        return alphaAngDeg, betaAngDeg

    def getAlphaBetaFromFocalXY(self,xFocal,yFocal):
        """x,y are mm with orgin at center of focal plane (or hex)
        """
        xLocal = xFocal - self.xyFocal[0]
        yLocal = yFocal - self.xyFocal[1]
        return self.getAlphaBetaFromLocalXY(xLocal, yLocal)

    def setAlphaBetaFromLocalXY(self,xLocal,yLocal):
        """x,y are mm with the origin being the robot's center
        """
        a,b = self.getAlphaBetaFromLocalXY(xLocal,yLocal)
        self.setAlphaBeta(a,b)

    def setAlphaBetaFromFocalXY(self,xFocal,yFocal):
        """x,y are mm with origin at center of focal plane
        """
        alpha,beta = self.getAlphaBetaFromFocalXY(xFocal,yFocal)
        self.setAlphaBeta(alpha,beta)


    @property
    def xyFiberLocal(self):
        # assumed at the end of the beta arm local coords
        if self._xyFiberLocal is None:
            # compute it now
            xA, yA = self.xyAlphaLocal
            x = xA + self._cosAlphaBeta*BetaArmLength
            y = yA + self._sinAlphaBeta*BetaArmLength
            self._xyFiberLocal = numpy.array([x,y])
            x = xA + self._cosAlphaBeta*BetaArmLengthSafe
            y = yA + self._sinAlphaBeta*BetaArmLengthSafe
            self._xySafeBetaLocal = numpy.array([x,y])
        return self._xyFiberLocal

    @property
    def xyFiberFocal(self):
        # assumed at the end of the beta arm focal coords
        return self.xyFocal + self.xyFiberLocal

    @property
    def xyAlphaLocal(self):
        # end of alpha arm
        if self._xyAlphaLocal is None:
            # compute it now
            x = self._cosAlpha*AlphaArmLength
            y = self._sinAlpha*AlphaArmLength
            self._xyAlphaLocal = numpy.array([x,y])
        return self._xyAlphaLocal

    @property
    def xySafeBetaLocal(self):
        # xy position of beginning of save zone
        if self._xySafeBetaLocal is None:
            # make it, shit this is terrible
            self.xyFiberLocal
        return self._xySafeBetaLocal

    @property
    def xySafeBetaFocal(self):
        return self.xyFocal + self.xySafeBetaLocal



    @property
    def xyAlphaFocal(self):
        return self.xyFocal + self.xyAlphaLocal

    def check2xy(self):
        xLocal,yLocal = self.xyFiberLocal
        alpha, beta = self.alphaBeta
        a, b = self.getAlphaBetaFromLocalXY(xLocal,yLocal)
        self.setAlphaBetaFromLocalXY(xLocal,yLocal)
        xLocal2,yLocal2 = self.xyFiberLocal
        print("alpha,beta err:", alpha-a, beta-b)
        print("xyLocal err:", xLocal-xLocal2, yLocal-yLocal2)



class RobotGrid(object):
    def __init__(self, nDia):
        """create a hex-packed grid of robots with a certain pitch.
        nDia describes the number of robots along y=0 axis.
        """
        self.xAll, self.yAll = hexFromDia(nDia)
        self.buildRobotList() # sets self.robotList
        self.buildNeighborList() # sets self.sketchyNeighborList and connects robots to their neighbors
        self.enforceMinTargSeparation()
        # self.swapIter()

    def buildRobotList(self):
        # create the list of robots
        self.robotList = []
        nRobots = len(self.xAll)
        for robotID, xFocal, yFocal in zip(range(nRobots), self.xAll, self.yAll):
            robot = Robot(robotID, xFocal=xFocal, yFocal=yFocal)
            robot.setAlphaBetaRand()
            # robot.check2xy()
            self.robotList.append(robot)

    def buildNeighborList(self):
        # for each robot in list find neighbors
        self.sketchyNeighborList = []
        for robot in self.robotList:
            for otherRobot in self.robotList:
                if robot.id == otherRobot.id:
                    # don't self compare
                    continue
                roboDist = numpy.linalg.norm(robot.xyFocal - otherRobot.xyFocal)
                # print('roboDist', roboDist)
                neighborExists = False
                if roboDist < (2*MaxReach):
                    # these robots have potential to colide
                    # first verify that they aren't already
                    # in the neighborList
                    for n in self.sketchyNeighborList:
                        if set([robot.id, otherRobot.id]) == n.id:
                            # this set was already added
                            neighborExists = True
                            break
                    if not neighborExists:
                        sketchNeighbor = SketchyNeighbors(robot, otherRobot)
                        self.sketchyNeighborList.append(sketchNeighbor)
                        robot.addSkectchyNeighbor(sketchNeighbor)
                        otherRobot.addSkectchyNeighbor(sketchNeighbor)

    def enforceMinTargSeparation(self):
        # loop over all robot targets and make sure they are separated
        # by a minimum distance, replace them until separation in achieved
        # neighboring robots can have nearby targets
        for robot in self.robotList:
            thisID = robot.id
            otherTargs = []
            for n in robot.sketchyNeighbors:
                if n.robotA.id == thisID:
                    otherTargs.append(n.robotB.xyFiberFocal)
                else:
                    otherTargs.append(n.robotA.xyFiberFocal)
            otherTargs = numpy.asarray(otherTargs)
            nOther = len(otherTargs)
            while True:
                # replace this target until there is a minimum separation
                thisTarg = numpy.tile(robot.xyFiberFocal, nOther).reshape(nOther,2)
                mindist = numpy.min(numpy.linalg.norm(otherTargs - thisTarg, axis=1))
                if mindist < MinTargSeparation:
                    print("replacing target for robot id %i"%robot.id)
                    robot.setAlphaBetaRand()
                else:
                    break

    def swapIter(self):
        for i in range(1000):
            nCollide = numpy.sum([n.isCollided for n in self.sketchyNeighborList])
            print("iter: %i, collisions: %i"%(i, nCollide))
            if nCollide == 0:
                break
            else:
                for n in self.sketchyNeighborList:
                    n.checkSwap()




    def plotGrid(self):
        fig = plt.figure(figsize=(9,9))
        ax = plt.gca()
        # ax = fig.add_subplot(111)
        plt.scatter(self.xAll, self.yAll)
        for robot in self.robotList:
            ax.plot(
                [robot.xyFocal[0], robot.xyAlphaFocal[0]],
                [robot.xyFocal[1], robot.xyAlphaFocal[1]],
                '-k.', linewidth=3,
                zorder=8,
            )
            color = "blue"
            # check for collisions, if so plot as red
            for n in robot.sketchyNeighbors:
                if n.intersection:
                    color='red'
            # ax.plot(
            #     [robot.xyAlphaFocal[0], robot.xyFiberFocal[0]],
            #     [robot.xyAlphaFocal[1], robot.xyFiberFocal[1]],
            #     '-', color=color, linewidth=2,
            # )
            # safe zone
            ax.plot(
                [robot.xyAlphaFocal[0], robot.xySafeBetaFocal[0]],
                [robot.xyAlphaFocal[1], robot.xySafeBetaFocal[1]],
                '-c', linewidth=3,
                zorder=9,
            )
            # collision zone
            patch = PolygonPatch(robot._collisionBox, fc=color, ec=color, alpha=0.5, zorder=10)
            ax.add_patch(patch)
        plt.axis('equal')
        # plt.xlim([-150,150])
        # plt.ylim([-150,150])


if __name__ == "__main__":
    nRobots = 500
    nc = int(numpy.sqrt((nRobots*4-1)/3))
    rg = RobotGrid(nc)
    rg.plotGrid()
    rg.swapIter()
    rg.plotGrid()
    plt.show()





