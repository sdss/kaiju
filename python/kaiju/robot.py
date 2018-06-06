import numpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from descartes import PolygonPatch
import copy
from networkx import DiGraph, shortest_path, draw, find_cycle, simple_cycles, shortest_simple_paths, has_path
from multiprocessing import Pool
import math
import sys

AlphaArmLength = 7.4 #mm
AlphaRange = [0, 360]
BetaRange = [0,180]
BetaArmLength = 15 #mm, distance to fiber
BetaArmWidth = 5 #mm
MinTargSeparation = 8 #mm
# length mm along beta for which a collision cant happen
BetaTopCollide = [8.187,16]# box from length 8.187mm to 16mm
BetaBottomCollide = [0,10.689] #box from 0 to 10.689
BetaArmLengthSafe = BetaArmLength / 2.0
Pitch = 22.4 # mm fudge the 01 for neighbors
MinReach = BetaArmLength - AlphaArmLength
MaxReach = BetaArmLength + AlphaArmLength

# https://internal.sdss.org/trac/as4/wiki/FPSLayout


PlateScale = 217.7358 / 3600.0 # plug plate scale (mm/arcsecond)

def k_sin(x,k=10):
    # series expand sin
    sin = 0
    for kk in range(k):
        sin += ((-1)**kk)*x**(2*kk+1) / float(math.factorial(2*kk + 1))
    return sin

def k_cos(x,k=10):
    # series expand cosine
    cos = 0
    for kk in range(k):
        cos += ((-1)**kk)*x**(2*kk) / float(math.factorial(2*kk))
    return cos

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
        self._topIntersect = None
        self._bottomIntersect = None
        self.swapTried = False
        self.id = set([robotA.id, robotB.id])

    def getNeighbor(self, robot):
        """return the other robot
        """
        assert robot.id in self.id, "this robot isn't a neighbor!"
        if robot.id == self.robotA.id:
            return self.robotB
        else:
            return self.robotA

    @property
    def isCollided(self):
        tc = bool(self.topIntersect)
        bc = bool(self.bottomIntersect)
        return bc or tc

    @property
    def topIntersect(self):
        # do the top of the beta arms collide?
        if self._topIntersect is None:
            self._topIntersect = self.robotA.topCollideLine.intersection(self.robotB.topCollideLine)
        return self._topIntersect

    @property
    def bottomIntersect(self):
        # do the top of the beta arms collide?
        if self._bottomIntersect is None:
            self._bottomIntersect = self.robotA.bottomCollideLine.intersection(self.robotB.bottomCollideLine)
        return self._bottomIntersect

    def fiberDist(self):
        """distance between fibers (end of beta arm)
        """
        return numpy.linalg.norm(self.robotA.xyFiber-self.robotB.xyFiber)

    def alphaDist(self):
        """distance between ends of alpha arms
        """
        return numpy.linalg.norm(self.robotA.xyAlpha-self.robotB.xyAlpha)

    def checkSwap(self):
        if self.isCollided:
            x1,y1 = self.robotA.xyFiberFocal
            x2,y2 = self.robotB.xyFiberFocal
            self.swapTried = True
            # check that they can reach
            try:
                a1,b1 = self.robotB.getAlphaBetaFromFocalXY(x1,y1)
                a2,b2 = self.robotA.getAlphaBetaFromFocalXY(x2,y2)
                self.robotB.setAlphaBeta(a1,b1)
                self.robotA.setAlphaBeta(a2,b2)
                if not self.isCollided:
                    print("swap worked")
                else:
                    print("swap still intersects")
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
        self.replacementsTried = 0
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
        self._topCollideLine = None
        self._bottomCollideLine = None
        self.sketchyNeighbors = []
        self._collisionBox = None # shapely geom populated by neighbor
        self.threwAway = False
        self.deadLocked = False
        ##################
        self.xyFocal = None
        self.betaTarg = None
        self.betaStep = 0.25 # each step is 0.25 degrees
        if not None in [xFocal, yFocal]:
            self.setXYFocal(xFocal, yFocal)
        if not None in [alphaAng, betaAng]:
            self.setAlphaBeta(alphaAng, betaAng)


    def __repr__(self):
        return ("Robot(id=%i)"%self.id)

    def _preComputeTrig(self):
        # pre compute some stuff...
        self._cosAlpha = numpy.cos(self._alphaRad)
        self._sinAlpha = numpy.sin(self._alphaRad)
        self._cosAlphaBeta = numpy.cos(self._alphaRad+self._betaRad)
        self._sinAlphaBeta = numpy.sin(self._alphaRad+self._betaRad)

    @property
    def onTarget(self):
        return self.betaTarg == self.alphaBeta[1]

    @property
    def distToTarget(self):
        currAlpha, currBeta = self.alphaBeta
        betaDist = self.betaTarg - currBeta
        return betaDist

    @property
    def isCollided(self):
        collided = False
        for n in self.sketchyNeighbors:
            if n.isCollided:
                collided = True
                break
        return collided

    @property
    def alphaBeta(self):
        return numpy.array([numpy.degrees(self._alphaRad), numpy.degrees(self._betaRad)])

    def stepTowardTarg(self):
        # return False if step created collision
        # return True if step acquired target
        # return None otherwise
        stepDir = 1
        currAlpha, currBeta = self.alphaBeta
        betaDist = self.betaTarg - currBeta
        self.deadLocked = False
        if betaDist == 0:
            assert False, 'error!'
        if betaDist < 0:
            stepDir = -1
        if numpy.abs(betaDist) < self.betaStep:
            self.setAlphaBeta(currAlpha, self.betaTarg)
        else:
            self.setAlphaBeta(currAlpha, currBeta+stepDir*self.betaStep)
        # if this step caused a collision, move back
        if self.isCollided:
            self.setAlphaBeta(currAlpha, currBeta)
            self.deadLocked = True
            return False
        if self.onTarget:
            return True
        else:
            return None

    def stepAwayTarg(self):
        pass

    def addSkectchyNeighbor(self, sketchyNeighbor):
        # maybe enforce that this neighbor doesn't already
        # exist?
        self.sketchyNeighbors.append(sketchyNeighbor)

    # def swapList(self):
    #     """Return list of robots that can reach my target (for swap potential)
    #     """
    #     roboList = []
    #     for n in self.sketchyNeighbors:
    #         otherRobot = n.getNeighbor(self)
    #         canReach = otherRobot.checkFocalReach(*self.xyFiberFocal)
    #         if canReach:
    #             roboList.append(otherRobot)
    #     return roboList

    def swapList(self):
        """Return list of robots whose target I can reach (for swap potential)
        """
        roboList = []
        for n in self.sketchyNeighbors:
            otherRobot = n.getNeighbor(self)
            canReach = self.checkFocalReach(*otherRobot.xyFiberFocal)
            if canReach:
                roboList.append(otherRobot)
        return roboList

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
        self._topCollideLine = None
        self._bottomCollideLine = None
        self._xyFiberLocal = None
        # clear any precomputed collisions
        for n in self.sketchyNeighbors:
            n._topIntersect = None
            n._bottomIntersect = None

    def setXYFocal(self, xFocal, yFocal):
        """position in focal plane mm
        """
        self.xyFocal = numpy.asarray([xFocal, yFocal])
        # clear cached positions, if any
        self._xyAlphaLocal = None
        self._topCollideLine = None
        self._bottomCollideLine = None
        self._xyFiberLocal = None

    def checkLocalReach(self,xLocal,yLocal):
        """return true if the robot can reach this position xy mm local coords
        """
        dist = numpy.sqrt(xLocal**2 + yLocal**2)
        return MinReach <= dist <= MaxReach

    def checkFocalReach(self,xFocal,yFocal):
        xLocal = self.xyFocal[0] - xFocal
        yLocal = self.xyFocal[1] - yFocal
        return self.checkLocalReach(xLocal, yLocal)

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
        return self._xyFiberLocal

    @property
    def xyFiberFocal(self):
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
    def bottomCollideLine(self):
        if self._bottomCollideLine is None:
            lineBuffer = BetaArmWidth/2.0
            x,y = self.xyFiberFocal - self.xyAlphaFocal
            fTheta = numpy.arctan2(y,x)
            x1 = numpy.cos(fTheta)*(BetaBottomCollide[0]+lineBuffer) + self.xyAlphaFocal[0]
            y1 = numpy.sin(fTheta)*(BetaBottomCollide[0]+lineBuffer) + self.xyAlphaFocal[1]

            x2 = numpy.cos(fTheta)*(BetaBottomCollide[1]-lineBuffer) + self.xyAlphaFocal[0]
            y2 = numpy.sin(fTheta)*(BetaBottomCollide[1]-lineBuffer) + self.xyAlphaFocal[1]
            self._bottomCollideLine = LineString(
                [[x1,y1], [x2,y2]]
                ).buffer(lineBuffer, cap_style=3)
        return self._bottomCollideLine

    @property
    def topCollideLine(self):
        if self._topCollideLine is None:
            lineBuffer = BetaArmWidth/2.0
            x,y = self.xyFiberFocal - self.xyAlphaFocal
            fTheta = numpy.arctan2(y,x)
            x1 = numpy.cos(fTheta)*(BetaTopCollide[0]+lineBuffer) + self.xyAlphaFocal[0]
            y1 = numpy.sin(fTheta)*(BetaTopCollide[0]+lineBuffer) + self.xyAlphaFocal[1]

            x2 = numpy.cos(fTheta)*(BetaTopCollide[1]-lineBuffer) + self.xyAlphaFocal[0]
            y2 = numpy.sin(fTheta)*(BetaTopCollide[1]-lineBuffer) + self.xyAlphaFocal[1]
            self._topCollideLine = LineString(
                [[x1,y1], [x2,y2]]
                ).buffer(lineBuffer, cap_style=3)
        return self._topCollideLine


    @property
    def xyAlphaFocal(self):
        return self.xyFocal + self.xyAlphaLocal

    def check2xy(self):
        xLocal,yLocal = self.xyFiberLocal
        alpha, beta = self.alphaBeta
        a, b = self.getAlphaBetaFromLocalXY(xLocal,yLocal)
        self.setAlphaBetaFromLocalXY(xLocal,yLocal)
        xLocal2,yLocal2 = self.xyFiberLocal



class RobotGrid(object):
    def __init__(self, nDia, minTargSeparation=MinTargSeparation):
        """create a hex-packed grid of robots with a certain pitch.
        nDia describes the number of robots along y=0 axis.
        """
        self.xAll, self.yAll = hexFromDia(nDia)
        self.buildRobotList() # sets self.robotList
        self.buildNeighborList() # sets self.sketchyNeighborList and connects robots to their neighbors
        self.minTargSeparation = minTargSeparation
        self.enforceMinTargSeparation()
        self._cacheList = []
        # self.swapIter()

    @property
    def nCollisions(self):
        return numpy.sum([n.isCollided for n in self.sketchyNeighborList])

    @property
    def nThrownAway(self):
        return numpy.sum([r.threwAway for r in self.robotList])

    def buildRobotList(self):
        # create the list of robots
        self.robotList = []
        nRobots = len(self.xAll)
        for robotID, xFocal, yFocal in zip(range(nRobots), self.xAll, self.yAll):
            robot = Robot(robotID, xFocal=xFocal, yFocal=yFocal)
            robot.setAlphaBetaRand()
            # robot.check2xy()
            self.robotList.append(robot)

    def cachePositions(self):
        """Save a current list of positions (for trying new ones and potentially
        reverting)
        """
        self._cacheList = []
        for robot in self.robotList:
            self._cacheList.append(robot.xyFiberFocal)
        return copy.deepcopy(self._cacheList) # copy for paranoia!

    def setPosFromCache(self, cacheList=None):
        """Reset all robots to cached position
        if cacheList is None, use position in cache
        """
        if cacheList is None:
            cacheList = self._cacheList
        for xyFiberFocal, robot in zip(cacheList,self.robotList):
            robot.setAlphaBetaFromFocalXY(xyFiberFocal[0], xyFiberFocal[1])

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
                if mindist < self.minTargSeparation:
                    #print("replacing target for robot id %i"%robot.id)
                    robot.setAlphaBetaRand()
                    robot.replacementsTried += 1
                    if robot.replacementsTried > 2000:
                        robot.threwAway = True
                        while True:
                            robot.setAlphaBetaRand()
                            print("finding throw away position in enforcing targ separation")
                            if not robot.isCollided:
                                break
                        break
                else:
                    break


    def swapIter(self):
        print("begining collisions: %i"%self.nCollisions)
        for n in self.sketchyNeighborList:
            n.checkSwap()
        print("swap 1 collisions: %i"%self.nCollisions)



    def plotGrid(self, title=None):
        fig = plt.figure(figsize=(10,10))
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
            topcolor = "blue"
            bottomcolor = "blue"
            # check for collisions, if so plot as red
            for n in robot.sketchyNeighbors:
                if n.bottomIntersect:
                    bottomcolor='red'
                if n.topIntersect:
                    topcolor="orange"
            if robot.threwAway:
                topcolor, bottomcolor = "magenta", "magenta"
            if robot.deadLocked:
                topcolor, bottomcolor = "green", "green"
            ax.plot(
                [robot.xyAlphaFocal[0], robot.xyFiberFocal[0]],
                [robot.xyAlphaFocal[1], robot.xyFiberFocal[1]],
                '-', color="green", linewidth=2,
            )
            patch = PolygonPatch(robot._topCollideLine, fc=topcolor, ec=topcolor, alpha=0.5, zorder=10)
            ax.add_patch(patch)
            patch = PolygonPatch(robot._bottomCollideLine, fc=bottomcolor, ec=bottomcolor, alpha=0.5, zorder=10)
            ax.add_patch(patch)
        plt.axis('equal')
        if title is not None:
            plt.title(title)
        # plt.xlim([-150,150])
        # plt.ylim([-150,150])

    def getDirectedGraph(self):
        """return a directed graph representation for robots that can
        reach their neighbor's targets, for figuring out circular swaps
        """
        dg = DiGraph()
        for robot in self.robotList:
            for otherRobot in robot.swapList():
                dg.add_edge(robot.id, otherRobot.id, label="%i->%i"%(robot.id, otherRobot.id))
        return dg


def swapSearch(robotGrid, neighbor="a"):
    """search for orientations that reduce number of collisions (3 way swaps etc)
    """
    bestCache = robotGrid.cachePositions()
    bestCollide = robotGrid.nCollisions
    print("best Collide", bestCollide)
    sadNeighbors = [n for n in robotGrid.sketchyNeighborList if n.isCollided]
    for sadNeighbor in sadNeighbors:
        robotGrid.setPosFromCache(bestCache)
        if neighbor == "a":
            robot = sadNeighbor.robotA
        else:
            robot = sadNeighbor.robotB
        dg = robotGrid.getDirectedGraph()
        # remove any edges to rule out a simple swap solution to the
        # shortest path solution
        for otherRobot in robot.swapList():
            # reset cache if we're trying path from next robot
            # the grid has been modified by swapping positions
            # in the previous loop iteration
            robotGrid.setPosFromCache(bestCache)
            try:
                dg.remove_edge(otherRobot.id, robot.id)
            except:
                pass
            else:
                print("removed back ref edge")
            try:
                # print("has path?", has_path(dg, otherRobot.id, robot.id))
                pathTo = shortest_path(dg, otherRobot.id, robot.id)
            except:
                print("swap path not found")
                continue

            pathFrom = numpy.roll(pathTo,1)
            # begin swapping positions
            firstPos = robot.xyFiberFocal
            for fromRobotID, toRobotID in zip(pathFrom, pathTo):
                fromRobot = robotGrid.robotList[fromRobotID]
                toRobot = robotGrid.robotList[toRobotID]
                print(fromRobotID, toRobotID)
                print("%i-->%i"%(fromRobot.id,toRobot.id))
                if toRobot.id==robot.id:
                    xyFiberFocal = firstPos
                else:
                    xyFiberFocal = toRobot.xyFiberFocal
                try:
                    fromRobot.setAlphaBetaFromFocalXY(*xyFiberFocal)
                except:
                    print("shit!!!")
                    raise RuntimeError("something is fucked!")

            print("collisions now", robotGrid.nCollisions)
            robotGrid.swapIter()
            print("collisions after simple swap", robotGrid.nCollisions)
            if robotGrid.nCollisions < bestCollide:
                bestCache = robotGrid.cachePositions()
                bestCollide = robotGrid.nCollisions
                print("new best orientation")
                print("collisions now", bestCollide)
                break # don't try other paths if we have reduced number of collisions

    robotGrid.setPosFromCache(bestCache)
    print("best collide", robotGrid.nCollisions)


def throwAway(robotGrid):
    robotAs = []
    for n in robotGrid.sketchyNeighborList:
        if n.isCollided:
            robotAs.append(n.robotA)
    for robot in robotAs:
        while True:
            robot.setAlphaBetaRand()
            robot.threwAway = True
            if not robot.isCollided:
                break

def simpleRun():
    initialCollisions = []
    simpleSwapCollisions = []
    cycleSwapCollisions = []
    # cycleSwapCollisions2 = []
    thrownAway = []
    for ii in range(1):
        nRobots = 500
        nc = int(numpy.sqrt((nRobots*4-1)/3))
        rg = RobotGrid(nc)
        initialCollisions.append(rg.nCollisions)
        rg.plotGrid("Naive Assignment")
        plt.savefig("naive.png")
        rg.swapIter()
        simpleSwapCollisions.append(rg.nCollisions)
        rg.plotGrid("Pairwise Swap")
        plt.savefig("pairwise.png")
        swapSearch(rg)
        cycleSwapCollisions.append(rg.nCollisions)
        rg.plotGrid("Circular Swap")
        plt.savefig("circular.png")
        # swapSearch(rg)
        # cycleSwapCollisions2.append(rg.nCollisions)
        # rg.plotGrid()
        throwAway(rg)
        thrownAway.append(rg.nThrownAway)
        rg.plotGrid("Thrown Away")
        plt.savefig("thrown.png")
        print("threw away %i robots"%rg.nThrownAway)
        # plt.show()
    print(initialCollisions)
    print(simpleSwapCollisions)
    print(cycleSwapCollisions)
    # print(cycleSwapCollisions2)
    print(thrownAway)
    # print("throw away collisions: %i"%robotGrid.nCollisions)

def run1grid(minSeparation):
    nRobots = 500
    nc = int(numpy.sqrt((nRobots*4-1)/3))
    rg = RobotGrid(nc, minSeparation)
    failedMinSep = numpy.sum([1 for x in rg.robotList if x.replacementsTried > 2000])
    initialCollisions = rg.nCollisions
    rg.swapIter()
    pairSwapCollisions = rg.nCollisions
    swapSearch(rg)
    cycleSwapCollisions = rg.nCollisions
    throwAway(rg)
    nThrowAway = rg.nThrownAway
    return failedMinSep, initialCollisions, pairSwapCollisions, cycleSwapCollisions, nThrowAway

def runGridSeries(minSeparation):
    results = []
    for ii in range(100):
        results.append(run1grid(minSeparation))
    results = numpy.asarray(results)
    numpy.savetxt("separation_%.2f.txt"%minSeparation, results, fmt="%i")

def explodeAndExplore():
    # run a series of grids at various minimum
    # spacings and save the collision results
    minSpacings = numpy.linspace(4,20,20)
    p = Pool(20)
    p.map(runGridSeries, minSpacings)
    print("explode complete!!!")

def motionPlan():
    minSeparation = 14
    # nRobots = 255
    # nc = int(numpy.sqrt((nRobots*4-1)/3))
    nc = 11
    rg = RobotGrid(nc, minSeparation)
    throwAway(rg)
    return rg

def separateMoves(doSort=False):
    rg = motionPlan()
    for robot in rg.robotList:
        if robot.threwAway:
            robot.threwAway = False
    rg.plotGrid("target")
    plt.savefig("target.png")
    for robot in rg.robotList:
        a,b = robot.alphaBeta
        robot.betaTarg = b
        robot.setAlphaBeta(a,180)
    rg.plotGrid("start")
    plt.savefig("start.png")
    ii = 0
    prevRobotSet = set([])
    while True:
        ii+=1
        robotsToMove = [robot for robot in rg.robotList if not robot.onTarget]
        if doSort:
            robotsToMove = sorted(robotsToMove, key=lambda robot: (numpy.linalg.norm(robot.xyFocal), robot.distToTarget))
        robotSet = set([robot.id for robot in robotsToMove])
        if robotSet == prevRobotSet:
            break # we arent getting anywhere
        # print("%i robots to move on iter %i"%(len(robotsToMove),ii))
        for robot in robotsToMove:
            while True:
                res = robot.stepTowardTarg()
                if res is None:
                    continue # continue stepping
                if res==True:
                    # print("robot %i achieved target"%robot.id)
                    break
                else:
                    # print("robot %i stopped before colliding"%(robot.id))
                    break
        figStr = "fig%s.png"%(("%i"%ii).zfill(4))
        rg.plotGrid(figStr)
        plt.savefig(figStr)
        prevRobotSet = robotSet

    rg.plotGrid("end")
    plt.savefig("end.png")

    deadlocks = sum([robot.deadLocked for robot in rg.robotList])
    total = len(rg.robotList)
    perc = (total-deadlocks)/float(total)
    # print("percent success %.2f"%perc)
    return perc

if __name__ == "__main__":
    doSort = False
    if len(sys.argv) > 1:
        print("do sort")
        doSort = True
    p = Pool(10)
    percents = p.map(separateMoves(doSort=doSort), range(100))
    print("found: %.2f (%.2f)"%(numpy.mean(percents), numpy.std(percents)))











