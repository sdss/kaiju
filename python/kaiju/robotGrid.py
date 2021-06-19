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
        self.totalReplaced = 0 # hack for now until it goes into the C++
        self.runtime = 0 # hack for now until
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
        # r["onTargetVec"] = robot.onTargetVec

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
            # print("rid", robot.id, robot.alpha, robot.beta)
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
            # otv = np.array(robot.onTargetVec)
            # print("otargvec", otv)
            # r["onTargetVec"] = robot.onTargetVec[-1]
            # # find last False (after which must be true)
            # # +1 because step numbers start from 1 and indices start from 0
            # whereFalse = np.argwhere(otv==False).flatten()
            # if not whereFalse:
            #     # empty list started on target? thats crazy
            #     lastFalse = 0
            # else:
            #     lastFalse = int(whereFalse[-1] + 1)
            # if lastFalse == len(otv):
            #     # last step was false robot never got there
            #     r["stepConverged"] = str(-1 )# -1 incicates never got to target
            # else:
            #     r["stepConverged"] = str(lastFalse)
            robotDict[robot.id] = r

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
        r["seed"] = self.seed
        r["algType"] = str(self.algType)
        r["totalReplaced"] = self.totalReplaced
        r["runtime"] = self.runtime
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
        nks = len(ks)
        robot_dict['id'] = [int(0)] * nks
        robot_dict['xPos'] = [float(0)] * nks
        robot_dict['yPos'] = [float(0)] * nks
        robot_dict['alpha'] = [float(0)] * nks
        robot_dict['beta'] = [float(0)] * nks
        robot_dict['validTargetIDs'] = [[]] * nks
        robot_dict['assignedTargetID'] = [int(0)] * nks
        robot_dict['hasApogee'] = [int(0)] * nks
        robot_dict['hasBoss'] = [int(0)] * nks
        robot_dict['isAssigned'] = [int(0)] * nks
        robot_dict['assignedFiberType'] = ['none'] * nks
        robot_dict['isCollided'] = [int(0)] * nks
        for i, k in enumerate(ks):
            crd = rd[k]
            robot_dict['id'][i] = int(crd.id)
            robot_dict['xPos'][i] = float(crd.xPos)
            robot_dict['yPos'][i] = float(crd.yPos)
            robot_dict['alpha'][i] = float(crd.alpha)
            robot_dict['beta'][i] = float(crd.beta)
            robot_dict['validTargetIDs'][i] = [int(y) for y in crd.validTargetIDs]
            robot_dict['assignedTargetID'][i] = int(crd.assignedTargetID)
            robot_dict['hasApogee'][i] = int(crd.hasApogee)
            robot_dict['hasBoss'][i] = int(crd.hasBoss)
            robot_dict['isAssigned'][i] = int(crd.isAssigned())
            if(crd.assignedTargetID >= 0):
                robot_dict['assignedFiberType'][i] = fiberType2Str[self.targetDict[crd.assignedTargetID].fiberType]
            robot_dict['isCollided'][i] = int(self.isCollided(k))
        return(robot_dict)

    def target_dict(self):
        """Create dictionary with target information

        Comments:
        --------

        Each value in dictionary contains a list with an element per target.
"""
        ks = list(self.targetDict.keys())
        nks = len(ks)
        target_dict = dict()
        target_dict['id'] = [int(0)] * nks
        target_dict['x'] = [float(0)] * nks
        target_dict['y'] = [float(0)] * nks
        target_dict['fiberType'] = ['none'] * nks
        for i, k in enumerate(ks):
            ctd = self.targetDict[k]
            target_dict['id'][i] = int(ctd.id)
            target_dict['x'][i] = float(ctd.x)
            target_dict['y'][i] = float(ctd.y)
            target_dict['fiberType'][i] = fiberType2Str[ctd.fiberType]
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
        json_str = '{'
        json_str = json_str + '"robot_obj" : ' + json.dumps(robot_dict) + ",\n"
        json_str = json_str + '"target_obj" : ' + json.dumps(target_dict) + "\n"
        json_str = json_str + '}'
        return(json_str)

    def html(self, filebase):
        """Write HTML format file for visualizing the current robot state

        Parameters:
        ----------

        filebase : str
            base of html and json to write to
"""
        fieldfile = filebase + '.json'
        fp = open(fieldfile, "w")
        fp.write(self.json())
        fp.close()

        fp = open(os.path.join(os.getenv('KAIJU_DIR'), 'etc',
                               'robotGrid.html'), "r")
        html_str = ''
        for l in fp.readlines():
            l = l.replace("fieldfile", "'" + os.path.basename(fieldfile) + "'")
            html_str = html_str + l
        fp.close()

        fp = open(filebase + '.html', "w")
        fp.write(html_str)
        fp.close()

        fp = open(os.path.join(os.getenv('KAIJU_DIR'), 'etc',
                               'robotGrid.js'), "r")
        js_str = ''
        for l in fp.readlines():
            js_str = js_str + l
        fp.close()

        fp = open(os.path.join(os.path.dirname(filebase), 'robotGrid.js'), "w")
        fp.write(js_str)
        fp.close()
        return

    def robot_array(self):
        """Return ndarray with robot and assignment information"""
        robot_dtype = np.dtype([('robotID', np.int32),
                                ('xPos', np.float32),
                                ('yPos', np.float32),
                                ('hasApogee', np.bool),
                                ('hasBoss', np.bool),
                                ('assignedTargetID', np.int64),
                                ('isAssigned', np.bool),
                                ('isCollided', np.bool),
                                ('alpha', np.float32),
                                ('beta', np.float32)])

        robot_array = np.zeros(self.nRobots, dtype=robot_dtype)
        ks = self.robotDict.keys()
        for i, k in enumerate(ks):
            crd = self.robotDict[k]
            robot_array['robotID'] = crd.id
            robot_array['xPos'] = crd.xPos
            robot_array['yPos'] = crd.yPos
            robot_array['hasApogee'] = crd.hasApogee
            robot_array['hasBoss'] = crd.hasBoss
            robot_array['assignedTargetID'] = crd.assignedTargetID
            robot_array['isAssigned'] = crd.isAssigned()
            robot_array['isCollided'] = self.isCollided(crd.id)
            robot_array['alpha'] = crd.alpha
            robot_array['beta'] = crd.beta
        return(robot_array)

    def target_array(self):
        """Return ndarray with target information"""
        target_dtype = np.dtype([('targetID', np.int64),
                                 ('x', np.float64),
                                 ('y', np.float64),
                                 ('priority', np.int32),
                                 ('fiberType', np.string_, 30)])

        target_array = np.zeros(len(self.targetDict), dtype=target_dtype)
        i = 0
        for k, td in self.targetDict.items():
            td = self.targetDict[k]
            target_array['targetID'][i] = td.id
            target_array['x'][i] = td.x
            target_array['y'][i] = td.y
            target_array['priority'][i] = td.priority
            ft = fiberType2Str[td.fiberType]
            target_array['fiberType'][i] = ft
            i = i + 1
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

class RobotGridDesignRef(RobotGrid):
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
                                "fps_DesignReference.txt")

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
            # make col 1 indexed to match pdf maps
            # and wok hole naming convention
            col = "C" + str(int(col) + 1)
            if row == "0" or row.startswith("-"):
                row = "R" + row
            else:
                row = "R+" + row

            holeName = row + col
            coords = [float(x), float(y)]
            if fType == "BA":
                baXY.append(coords)
            elif fType == "BOSS":
                bossXY.append(coords)
            elif fType == "Fiducial":
                fiducialXY.append(coords)
            else:
                # ignore other elements (like empty holes)
                pass

        fiberID = 0
        fiducialID = 0
        hasApogee = False
        for b in bossXY:
            self.addRobot(holeName, fiberID, b[0], b[1], hasApogee)
            fiberID += 1
        hasApogee = True
        for ba in baXY:
            self.addRobot(holeName, fiberID, ba[0], ba[1], hasApogee)
            fiberID += 1
        for fiducial in fiducialXY:
            self.addFiducial(fiducialID, fiducial[0], fiducial[1],
                             self.collisionBuffer)
            fiducialID += 1
        self.initGrid()

        return()



