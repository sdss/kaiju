#!/usr/bin/env python
# -*- coding:utf-8 -*-

# @Filename: robotGrid.py
# @License: BSD 3-clause (http://www.opensource.org/licenses/BSD-3-Clause)


import os
import json
import pickle
import numpy as np
import fitsio
import matplotlib.pyplot as plt
import kaiju
import kaiju.cKaiju
from descartes import PolygonPatch
from shapely.geometry import LineString, Point


__all__ = ['RobotGrid', 'RobotGridFilledHex']


# Create look-up dictionary for types to strings conversion
str2FiberType = {'ApogeeFiber': kaiju.cKaiju.ApogeeFiber,
                 'BossFiber': kaiju.cKaiju.BossFiber}
fiberType2Str = dict()
for k in str2FiberType.keys():
    fiberType2Str[str2FiberType[k]] = k

class RobotGrid(kaiju.cKaiju.RobotGrid):
    """Basic python subclass of cKaiju.RobotGrid

    Parameters:
    ----------

    stepSize : float, np.float32
        step size for paths (degrees), default 1.

    collisionBuffer : float, np.float32
        half-width of beta arm, including buffer (mm), default 2.0

    Attributes:
    ----------

    stepSize : float, np.float32
        step size for paths (degrees). A maximum perturbation allowed for either
        alpha or beta axes

    collisionBuffer : float, np.float32
        half-width of beta arm, including buffer (mm)

    epsilon : float
        smoothing parameter used in the RDP path simplification.  Smaller values
        mean smaller deviations from the rough path are allowed.

    seed : int
        seed for random number generator when used

    robotDict : dictionary of Robot class objects
        all robots

    nRobots : int
        number of robots

    fiducialDict : dictionary of Fiducial objects
        positions of fiducials

    targetDict : dictionary of Target class objects
        targets in field, with ID as keys

    smoothCollisions : int
        number of collisions detected after attempted path smoothing, should
        be zero when things work out

    didFail : bool
        path generation failed, not all robots reached target

    nSteps : int
        steps taken to

    """
    def __init__(self, stepSize=1., collisionBuffer=2.0, epsilon=None, seed=0):
        self.stepSize = stepSize
        self.collisionBuffer = collisionBuffer
        if epsilon is None:
            self.epsilon = stepSize * 2.2
        else:
            self.epsilon = epsilon
        self.seed = seed
        super().__init__(self.stepSize, self.collisionBuffer,
                         self.epsilon, self.seed)
        # self._load_grid()
        return

    def plotRobotCore(self, ax=None, robotID=None, isCollided=False):
        """Basic plotting for a single robot

        Parameters:
        ----------

        ax : matplotlib axis object
            axis object to add robot to

        robotID : int, np.int32
            robotID of robot to plot

        isCollided : bool
            is this robot collided?

        Comments:
        --------

        Dark blue are assigned robots.
        Red are collided robots
        Light blue are unassigned robots
        """
        internalBuffer = 1.5

        robot = self.robotDict[robotID]
        alphaPoint = robot.betaCollisionSegment[0]
        betaPoint = robot.betaCollisionSegment[1]
        alphaX = alphaPoint[0]
        alphaY = alphaPoint[1]
        betaX = betaPoint[0]
        betaY = betaPoint[1]
        plt.plot([robot.xPos, alphaX], [robot.yPos, alphaY],
                 color='black', linewidth=2, alpha=0.5)

        topCollideLine = LineString([(alphaX, alphaY), (betaX, betaY)]
                                    ).buffer(internalBuffer, cap_style=1)
        topcolor = 'blue'
        edgecolor = 'black'
        if isCollided:
            topcolor = "red"
        if not robot.isAssigned():
            topcolor = "skyblue"
        patch = PolygonPatch(topCollideLine, fc=topcolor, ec=edgecolor,
                             alpha=0.5, zorder=10)
        ax.add_patch(patch)

        return

    def plotAllRobots(self):
        """Plots all the robots

        Comments:
        --------

        Dark blue are assigned robots.
        Red are collided robots
        Light blue are unassigned robots
"""
        plt.figure(figsize=(10, 10))
        ax = plt.gca()

        for robotID in self.robotDict.keys():
            isCollided = self.isCollided(robotID)
            self.plotRobotCore(ax=ax, robotID=robotID, isCollided=isCollided)

        rr = 340.
        plt.xlim(np.array([-1., 1.]) * rr)
        plt.ylim(np.array([-1., 1.]) * rr)
        return


    def tofits(self, filename=None):
        """Write robot information to FITS file

        Parameters:
        ----------

        filename : str
            FITS file name to write to (clobbers)

        Comments:
        --------

        HDU1 contains the robot information
        HDU2 contains the target information
"""
        robot_array = self.robot_array()
        target_array = self.target_array()
        fitsio.write(filename, robot_array, clobber=True)
        fitsio.write(filename, target_array, clobber=False)
        return

    def fromfits(self, filename=None):
        """Read robot information from FITS file

        Parameters:
        ----------

        filename : str
            FITS file name to read from

        Comments:
        --------

        Assumes the format written out by tofits()
"""
        robot_array = fitsio.read(filename, ext=1)
        target_array = fitsio.read(filename, ext=2)
        self.clearTargetList()
        self.target_fromarray(target_array)
        self.robot_fromarray(robot_array)
        return

    def singleRobotDict(self, robot, downsample=None):
        """Dictionary for a single robot

        Parameters:
        -----------
        robot : cKaiju.Robot
            robot instance to scrape

        downsample : int or None
            Down sample the paths by this factor to decrease
            data size, last step is always included

        Returns:
        -------------
        result : a dict

        Comments:
        ---------
        note: robot_dict is dictionary for all robots
        note: should get pybind to give this automatically
        using __dict__ but haven't figured it out yet
        note: need to figure out interp downsampling
        """
        if downsample is None:
            downsample = 1

        r = {}

        r["id"] = robot.id
        r["nDecollide"] = robot.nDecollide
        r["lastStepNum"] = robot.lastStepNum
        r["assignedTargetID"] = robot.assignedTargetID
        r["hasApogee"] = robot.hasApogee
        r["hasBoss"] = robot.hasBoss
        r["xPos"] = robot.xPos
        r["yPos"] = robot.yPos
        r["alpha"] = robot.alpha
        r["beta"] = robot.beta
        r["destinationAlpha"] = robot.destinationAlpha
        r["desstinationBeta"] = robot.destinationBeta
        r["angStep"] = robot.angStep
        r["collisionBuffer"] = robot.collisionBuffer
        r["alphaVel"] = robot.alphaVel
        r["betaVel"] = robot.betaVel

        # convert from numpy arrays to lists
        r["metFiberPos"] = list(robot.metFiberPos)
        r["bossFiberPos"] = list(robot.bossFiberPos)
        r["apFiberPos"] = list(robot.apFiberPos)
        r["onTargetVec"] = robot.onTargetVec

        # sequences
        # convert from numpy arrays to lists
        downsampleMe = [
            "alphaPath",
            "betaPath",
            "roughAlphaX",
            "roughBetaX",
            "roughAlphaY",
            "roughBetaY"
        ]
        for item in downsampleMe:
            # these are all lists of numpy pairs
            l = [list(x) for x in getattr(robot, item)]
            lastElement = l[-1]
            l = l[::downsample] + [lastElement]
            r[item] = l

        # not sure how to handle downsampling for these yet
        r["smoothAlphaVel"] = robot.smoothAlphaVel
        r["smoothBetaVel"] = robot.smoothBetaVel
        # r["alphaPath"] = [list(x) for x in robot.alphaPath]
        # r["betaPath"] = [list(x) for x in robot.betaPath]
        r["smoothedAlphaPath"] = [list(x) for x in robot.smoothedAlphaPath]
        r["smoothedBetaPath"] = [list(x) for x in robot.smoothedBetaPath]
        r["simplifiedAlphaPath"] = [list(x) for x in robot.simplifiedAlphaPath]
        r["simplifiedBetaPath "] = [list(x) for x in robot.simplifiedBetaPath]
        r["interpSimplifiedAlphaPath"] = [list(x) for x in robot.interpSimplifiedAlphaPath]
        r["interpSimplifiedBetaPath "] = [list(x) for x in robot.interpSimplifiedBetaPath]
        r["interpAlphaX"] = [list(x) for x in robot.interpAlphaX]
        r["interpAlphaY"] = [list(x) for x in robot.interpAlphaY]
        r["interpBetaX"] = [list(x) for x in robot.interpBetaX]
        r["interpBetaY "] = [list(x) for x in robot.interpBetaY]
        # r["roughAlphaX"] = [list(x) for x in robot.roughAlphaX]
        # r["roughAlphaY"] = [list(x) for x in robot.roughAlphaY]
        # r["roughBetaX"] = [list(x) for x in robot.roughBetaX]
        # r["roughBetaY "] = [list(x) for x in robot.roughBetaY]
        r["interpCollisions "] = [list(x) for x in robot.interpCollisions]

        r["robotNeighbors"] = robot.robotNeighbors
        r["fiducialNeighbors"] = robot.fiducialNeighbors
        r["validTargetIDs"] = robot.validTargetIDs

        return r

    def robotGridSummaryDict(self):
        """Dictionary summary for this RobotGrid, contains
        less info and thus faster read/write than a full dict

        Returns:
        -------------
        result : a dict

        Comments:
        ---------
        note: should get pybind to give this automatically
        using __dict__ but haven't figured it out yet
        """
        rgd = self.robotGridDict(incRobotDict=False)
        robotDict = {}
        for rid, robot in self.robotDict.items():
            r = {}
            r["id"] = robot.id
            r["nDecollide"] = robot.nDecollide
            r["lastStepNum"] = robot.lastStepNum
            r["assignedTargetID"] = robot.assignedTargetID
            r["hasApogee"] = robot.hasApogee
            r["hasBoss"] = robot.hasBoss
            r["xPos"] = robot.xPos
            r["yPos"] = robot.yPos
            r["alpha"] = robot.alpha
            r["beta"] = robot.beta
            r["destinationAlpha"] = robot.destinationAlpha
            r["desstinationBeta"] = robot.destinationBeta
            r["angStep"] = robot.angStep
            r["collisionBuffer"] = robot.collisionBuffer

            # convert from numpy arrays to lists
            r["metFiberPos"] = list(robot.metFiberPos)
            r["bossFiberPos"] = list(robot.bossFiberPos)
            r["apFiberPos"] = list(robot.apFiberPos)
            r["roughAlphaX"] = robot.roughAlphaX[-1][-1]
            r["roughAlphaY"] = robot.roughAlphaY[-1][-1]
            r["roughBetaX"] = robot.roughBetaX[-1][-1]
            r["roughBetaY"] = robot.roughBetaY[-1][-1]
            # find the step at which this
            # guy found its target
            otv = np.array(robot.onTargetVec)
            # find last False (after which must be true)
            # +1 because step numbers start from 1 and indices start from 0
            lastFalse = np.argwhere(otv==False).flatten()[-1] + 1
            if lastFalse == len(otv):
                # last step was false robot never got there
                r["onTargetVec"] = -1 # -1 incicates never got to target
            else:
                r["onTargetVec"] = lastFalse

        rgd["robotDict"] = robotDict
        return rgd

    def robotGridDict(self, downsample=None, incRobotDict=True):
        """Dictionary for this RobotGrid

        Parameters:
        ------------
        downsample : int or None
            Down sample the paths by this factor to decrease
            data size, last step is always included

        incRobotDict : Bool
            If false, don't include the robot dict

        Returns:
        -------------
        result : a dict

        Comments:
        ---------
        note: should get pybind to give this automatically
        using __dict__ but haven't figured it out yet
        """

        r = {}
        r["nRobots"] = self.nRobots
        r["epsilon"] = self.epsilon
        r["collisionBuffer"] = self.collisionBuffer
        r["angStep"] = self.angStep
        r["didFail"] = self.didFail
        r["nSteps"] = self.nSteps
        r["maxPathSteps"] = self.maxPathSteps
        r["smoothCollisions"] = self.smoothCollisions
        r["maxDisplacement"] = self.maxDisplacement
        r["deadlockedRobots"] = self.deadlockedRobots()
        r["greed"] = self.greed
        r["phobia"] = self.phobia
        if incRobotDict:
            robotDict = {}
            for rid, robot in self.robotDict.items():
                robotDict[rid] = self.singleRobotDict(robot, downsample)
            r["robotDict"] = robotDict
        return r

    def fullPickle(self, filename, downsample=None):
        """Pickle the output of self.robotGridDict

        Parameters:
        --------------
        filename : filename for the pickle

        downsample : int or None
            Down sample the paths by this factor to decrease
            data size, last step is always included

        """
        d = self.robotGridDict(downsample)
        with open(filename, "wb") as f:
            pickle.dump(d, f)

    def fullJSON(self, filename=None, downsample=None):
        """Format robotGridDict() output as json

        Parameters:
        ------------
        filename : string or None
            If not none, write json to file, otherwise
        downsample : int or None
            Down sample the paths by this factor to decrease
            data size, last step is always included

        Returns:
        -------------
        result : a json string or None

        """
        d = self.robotGridDict(downsample)
        if filename is not None:
            with open(filename, "w") as f:
                json.dump(d, f, separators=(',', ':'))
        else:
            return json.dumps(d)

    def summaryPickle(self, filename, downsample=None):
        """Pickle the output of self.robotGridDict

        Parameters:
        --------------
        filename : filename for the pickle

        downsample : int or None
            Down sample the paths by this factor to decrease
            data size, last step is always included

        """
        d = self.robotGridSummaryDict()
        with open(filename, "wb") as f:
            pickle.dump(d, f)

    def summaryJSON(self, filename=None):
        """Format robotGridDict() output as json

        Parameters:
        ------------
        filename : string or None
            If not none, write json to file, otherwise
        downsample : int or None
            Down sample the paths by this factor to decrease
            data size, last step is always included

        Returns:
        -------------
        result : a json string or None

        """
        d = self.robotGridSummaryDict()
        if filename is not None:
            with open(filename, "w") as f:
                json.dump(d, f, separators=(',', ':'))
        else:
            return json.dumps(d)


    def robot_dict(self):
        """Create dictionary with robot information

        Comments:
        --------

        Each value in dictionary contains a list with an element per robot.
        Fiducial information is not included.
"""
        robot_dict = dict()
        rd = self.robotDict
        ks = rd.keys()
        robot_dict['id'] = [int(rd[k].id) for k in ks]
        robot_dict['xPos'] = [float(rd[k].xPos) for k in ks]
        robot_dict['yPos'] = [float(rd[k].yPos) for k in ks]
        robot_dict['alpha'] = [float(rd[k].alpha) for k in ks]
        robot_dict['beta'] = [float(rd[k].beta) for k in ks]
        robot_dict['validTargetIDs'] = [[y for y in rd[k].validTargetIDs]
                                        for k in ks]
        robot_dict['assignedTargetID'] = [int(rd[k].assignedTargetID)
                                          for k in ks]
        robot_dict['hasApogee'] = [int(rd[k].hasApogee) for k in ks]
        robot_dict['hasBoss'] = [int(rd[k].hasBoss) for k in ks]
        robot_dict['isAssigned'] = [int(rd[k].isAssigned()) for k in ks]
        robot_dict['isCollided'] = [int(self.isCollided(k)) for k in ks]
        return(robot_dict)

    def target_dict(self):
        """Create dictionary with target information

        Comments:
        --------

        Each value in dictionary contains a list with an element per target.
"""
        ks = list(self.targetDict.keys())
        target_dict = dict()
        target_dict['id'] = [int(self.targetDict[k].id) for k in ks]
        target_dict['x'] = [float(self.targetDict[k].x) for k in ks]
        target_dict['y'] = [float(self.targetDict[k].y) for k in ks]
        target_dict['fiberType'] = [fiberTypeStr(self.targetDict[k].fiberType)
                                    for k in ks]
        return(target_dict)

    def json(self):
        """Returns a javascript string setting robot and target dicts

        Comments:
        --------

        javascript string sets javascript dicts robot_str and target_obj

        Probably this should just return the jsons and not the "robot_obj ="
        and "target_obj =" parts.
"""
        robot_dict = self.robot_dict()
        target_dict = self.target_dict()
        json_str = "robot_obj = " + json.dumps(robot_dict) + ";\n"
        json_str = json_str + "target_obj = " + json.dumps(target_dict) + ";\n"
        return(json_str)

    def html(self, filename):
        """Write HTML format file for visualizing the current robot state

        Parameters:
        ----------

        filename : str
            name of file to write to
"""
        html_str = '<script type="text/javascript">\n'
        html_str = html_str + 'robot_obj = ' + json.dumps(self.robot_dict()) + ";\n"
        html_str = html_str + 'target_obj = ' + json.dumps(self.target_dict()) + ";\n"
        html_str = html_str + '</script>\n'
        fp = open(os.path.join(os.getenv('KAIJU_DIR'), 'etc',
                               'robotGrid.html'), "r")
        for l in fp.readlines():
            html_str = html_str + l
        fp.close()
        fp = open(filename, "w")
        fp.write(html_str)
        fp.close()
        return

    def robot_array(self):
        """Return ndarray with robot and assignment information"""
        robot_dtype = np.dtype([('robotID', np.int32),
                                ('xPos', np.float32),
                                ('yPos', np.float32),
                                ('hasApogee', np.bool),
                                ('hasBoss', np.bool),
                                ('assignedTargetID', np.int32),
                                ('isAssigned', np.bool),
                                ('isCollided', np.bool),
                                ('alpha', np.float32),
                                ('beta', np.float32)])

        robot_array = np.zeros(self.nRobots, dtype=robot_dtype)
        ks = self.robotDict.keys()
        robot_array['robotID'] = np.array([self.robotDict[k].id for k in ks],
                                          dtype=np.int32)
        robot_array['xPos'] = np.array([self.robotDict[k].xPos for k in ks],
                                       dtype=np.float32)
        robot_array['yPos'] = np.array([self.robotDict[k].yPos for k in ks],
                                       dtype=np.float32)
        robot_array['hasApogee'] = np.array([self.robotDict[k].hasApogee
                                             for k in ks],
                                            dtype=np.bool)
        robot_array['hasBoss'] = np.array([self.robotDict[k].hasBoss
                                           for k in ks],
                                          dtype=np.bool)
        robot_array['assignedTargetID'] = np.array([self.robotDict[k].assignedTargetID
                                                    for k in ks],
                                                   dtype=np.int32)
        robot_array['isAssigned'] = [int(self.robotDict[k].isAssigned())
                                     for k in ks]
        robot_array['isCollided'] = [int(self.isCollided(k)) for k in ks]
        robot_array['alpha'] = np.array([self.robotDict[k].alpha
                                         for k in ks], dtype=np.float32)
        robot_array['beta'] = np.array([self.robotDict[k].beta
                                        for k in ks], dtype=np.float32)
        return(robot_array)

    def target_array(self):
        """Return ndarray with target information"""
        target_dtype = np.dtype([('targetID', np.int32),
                                 ('x', np.float32),
                                 ('y', np.float32),
                                 ('priority', np.int32),
                                 ('fiberType', np.string_, 30)])

        target_array = np.zeros(len(self.targetDict), dtype=target_dtype)
        ks = list(self.targetDict.keys())
        for i, k in enumerate(ks):
            target_array['targetID'][i] = self.targetDict[k].id
            target_array['x'][i] = self.targetDict[k].x
            target_array['y'][i] = self.targetDict[k].y
            target_array['priority'][i] = self.targetDict[k].priority
            ft = fiberType2Str[self.targetDict[k].fiberType]
            target_array['fiberType'][i] = ft
        return(target_array)

    def target_fromarray(self, target_array):
        """Fill robotGrid targetDict from target array

        Parameters:
        ----------

        target_array : ndarray
            array with target information

        Comments:
        --------

        Does not clearTargetList() first, so will add targets to
        existing list.
"""
        for it in np.arange(len(target_array)):
            ft = target_array['fiberType'][it].decode().strip()
            fiberType = str2FiberType[ft]
            self.addTarget(targetID=target_array['targetID'][it],
                           x=target_array['x'][it],
                           y=target_array['y'][it],
                           fiberType=fiberType,
                           priority=target_array['priority'][it])
        return

    def robot_fromarray(self, robot_array):
        """Assign robots according to robot_array

        Parameters:
        ----------

        robot_array : ndarray
            array with robot information

        Comments:
        --------

        Assumes that the robotRict is already created, and is
        consistent with the robot_array information. It double
        checks that any information in robot_array is consistent
        with this robotGrid.
"""
        for ir, r in enumerate(robot_array):
            if(r['isAssigned']):
                indx = r['assignedTargetID']
                self.assignRobot2Target(r['robotID'], indx)
            else:
                self.robotDict[r['robotID']].setAlphaBeta(r['alpha'],
                                                          r['beta'])
        r = self.robot_array()
        for n in r.dtype.names:
            if(n in robot_array.dtype.names):
                ibad = np.where(r[n] != robot_array[n])[0]
                if(len(ibad) > 0):
                    raise RuntimeError("Inconsistency in robot file in column {n}".format(n=n))
        return







class RobotGridFilledHex(RobotGrid):
    """Filled hexagon grid class for robots in FPS

    Parameters:
    ----------

    stepSize : float, np.float32
        step size for paths (degrees), default 1.

    collisionBuffer : float, np.float32
        half-width of beta arm, including buffer (mm), default 2.0

    Attributes:
    ----------

    stepSize : float, np.float32
        step size for paths (degrees). A maximum perturbation allowed for either
        alpha or beta axes

    collisionBuffer : float, np.float32
        half-width of beta arm, including buffer (mm)

    epsilon : float
        smoothing parameter used in the RDP path simplification.  Smaller values
        mean smaller deviations from the rough path are allowed.

    seed : int
        seed for random number generator when used

    robotDict : dictionary of Robot class objects
        all robots

    nRobots : int
        number of robots

    fiducialDict : dictionary of Fiducial objects
        positions of fiducials

    targetDict : dictionary of Target class objects
        targets in field, with ID as keys

    smoothCollisions : int
        number of collisions detected after attempted path smoothing, should
        be zero when things work out

    didFail : bool
        path generation failed, not all robots reached target

    nSteps : int
        steps taken to

"""
    def __init__(self, stepSize=1., collisionBuffer=2.0):
        super().__init__(seed=0, stepSize=stepSize, collisionBuffer=collisionBuffer)
        self._load_grid()
        return

    def _load_grid(self):
        """Load filled hex grid of robot locations"""
        gridFile = os.path.join(os.environ["KAIJU_DIR"], "etc",
                                "fps_filledHex.txt")

        bossXY = []
        baXY = []
        fiducialXY = []
        with open(gridFile, "r") as f:
            lines = f.readlines()
        for line in lines:
            line = line.strip()
            line = line.split("#")[0]
            if not line:
                continue
            row, col, x, y, fType = line.split()
            coords = [float(x), float(y)]
            if fType == "BA":
                baXY.append(coords)
            elif fType == "BOSS":
                bossXY.append(coords)
            else:
                fiducialXY.append(coords)

        fiberID = 0
        fiducialID = 0
        hasApogee = False
        for b in bossXY:
            self.addRobot(fiberID, b[0], b[1], hasApogee)
            fiberID += 1
        hasApogee = True
        for ba in baXY:
            self.addRobot(fiberID, ba[0], ba[1], hasApogee)
            fiberID += 1
        for fiducial in fiducialXY:
            self.addFiducial(fiducialID, fiducial[0], fiducial[1],
                             self.collisionBuffer)
            fiducialID += 1
        self.initGrid()

        return()


