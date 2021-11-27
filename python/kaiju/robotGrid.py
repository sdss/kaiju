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
from shapely.geometry import LineString
import coordio
from coordio.defaults import IHAT, JHAT, KHAT, calibration
import pandas as pd


# positionerTable = calibration.positionerTable
# wokCoords = calibration.wokCoords
# fiducialCoords = calibration.fiducialCoords

# __all__ = ['RobotGrid', 'RobotGridFilledHex']
# default orientation of positioner to wok
# iHat = [0,-1,0]
# jHat = [1,0,0]
# kHat = [0,0,1]

KAIJU_ETC_DIR = os.path.join(
    os.path.dirname(__file__), "etc"
)

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
        step size for paths (degrees), default 0.1.

    epsilon : float, or None
        distance metric for path simplification, smaller
        values yeild more points, default is stepsize * 2.2

    seed : int
        random seed for grid

    scaleFac : float
        multiplicative scale factor to model thermal expansion of wok,
        default is 1


    Attributes:
    ----------

    stepSize : float, np.float32
        step size for paths (degrees). A maximum perturbation allowed for either
        alpha or beta axes

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
        steps taken to generate a path

    """
    def __init__(
        self, stepSize=0.1,
        epsilon=None, seed=0, scaleFac=1):

        np.random.seed(seed) # for seedifying stuff out here in python

        self.stepSize = stepSize
        self.scaleFac = scaleFac
        if epsilon is None:
            self.epsilon = stepSize * 2.2
        else:
            self.epsilon = epsilon
        self.seed = seed
        self.totalReplaced = 0 # hack for now until it goes into the C++
        self.runtime = 0 # hack for now until
        super().__init__(self.stepSize, self.epsilon, self.seed)
        self._load_grid()
        return

    def _load_grid(self):
        # subclasses may want to specify grid initialization here
        # but they don't have to
        pass

    def getRandomPathPair(self, alphaHome=0, betaHome=180, betaLim=None):
        """
        Get a random trajectory pair.

        Parameters
        -------------
        alphaHome : float
            degrees for alpha axis lattice/home state
        betaHome : float
            degrees for beta axis lattice/home state
        betaLim : None, or [minBeta, maxBeta]  must be safe
            flag for making collisions impossible


        Returns
        ---------
        forwardPath: dict
            alpha/beta points for all robots, moving them from lattice state
            to target state (aligned with sources).  The out path.

        reversePath: dict
            alpha/beta points for all robots, moving them from target state
            to lattice state.  The back path.
        """
        # betaSafeLim = 165  # 155 should be safe even.
        for robot in self.robotDict.values():
            robot.setDestinationAlphaBeta(alphaHome, betaHome)
            if robot.isOffline:
                continue
            if betaLim is not None:
                alpha = np.random.uniform(0, 359.99)
                beta = np.random.uniform(betaLim[0], betaLim[1])
                robot.setAlphaBeta(alpha, beta)
            else:
                robot.setXYUniform()


        if betaLim is not None and self.getNCollisions() > 0:
            raise RuntimeError("betaLim specified, but collisions present")
        else:
            self.decollideGrid()
        return self.getPathPair()

    def getPathPair(
        self, speed=2, smoothPoints=5, collisionShrink=0.05, pathDelay=1
    ):
        """
        Get paths in format that jaeger expects.  No checking is done, so
        whoever calls this should check things on the RobotGrid
        and decide what to do next.

        Parameters
        -----------
        speed: float
            RPM at output, how fast robots move, max speed is 3
        smoothPoints: int
            window width for smoothing a path's velocity profile,
            in units of steps.  Smooths out fast switching between forward
            and reverse moves on the axes.
        collisionShrink: float
            mm, how much to decrease the collision buffer to allow
            for smoothing
        pathDely: float
            seconds, how far in the future to put the first point, this
            allows a robot to "catch up" to the expected starting point
            for the path, if it's not there already.

        Returns
        ---------
        toDestination: dict
            alpha/beta points in time for all robots.  Path begins at robot
            grid's initialized state, moving toward destination state.

        fromDestination: dict
            alpha/beta points in time for all robots.  Path begins at robot
            grid's destination state, moving toward the initialized state.
            This is just simply a reversed version of toDestination.
        """
        # break out these parameters later
        # smoothPoints = 5
        # collisionShrink = 0.05  # mm, this value shoudld probably be a function of stepsize
        # speed = 2 # rpm at output
        # pathDelay = 1  # seconds
        ###########

        self.smoothPaths(smoothPoints)
        self.simplifyPaths()
        self.shrinkCollisionBuffer(collisionShrink)
        self.verifySmoothed()
        self.growCollisionBuffer(collisionShrink)

        toDestination = {}
        fromDestination = {}

        for r in self.robotDict.values():

            # if robot is offline, don't get a path for it
            if r.isOffline:
                continue

            ibp = np.array(r.simplifiedBetaPath)


            iap = np.array(r.simplifiedAlphaPath)


            alphaTimesR = iap[:, 0] * self.stepSize / (speed * 360 / 60.)
            alphaDegR = iap[:, 1]
            betaTimesR = ibp[:, 0] * self.stepSize / (speed * 360 / 60.)
            betaDegR = ibp[:, 1]


            # add time buffer for the reverse path, in case robot is
            # not exactly starting from the expected spot.
            # build path from initial state to destination
            armPathR = {}
            armPathR["alpha"] = [(pos, time + pathDelay) for pos, time in zip(alphaDegR, alphaTimesR)]
            armPathR["beta"] = [(pos, time + pathDelay) for pos, time in zip(betaDegR, betaTimesR)]

            toDestination[int(r.id)] = armPathR

            # build path from destination to initial state
            alphaTimesF = np.abs(alphaTimesR - alphaTimesR[-1])[::-1]
            alphaDegF = alphaDegR[::-1]
            betaTimesF = np.abs(betaTimesR - betaTimesR[-1])[::-1]
            betaDegF = betaDegR[::-1]

            armPathF = {}
            armPathF["alpha"] = [(pos, time + pathDelay) for pos, time in zip(alphaDegF, alphaTimesF)]
            armPathF["beta"] = [(pos, time + pathDelay) for pos, time in zip(betaDegF, betaTimesF)]


            fromDestination[int(r.id)] = armPathF

        return toDestination, fromDestination

    def addRobot(self,
        robotID, holeID, basePos, hasApogee=True,
        iHat=IHAT, jHat=JHAT, kHat=KHAT,
        dxyz=[0,0,0], alphaLen=coordio.defaults.ALPHA_LEN, alphaOffDeg=0,
        betaOffDeg=0, elementHeight=coordio.defaults.POSITIONER_HEIGHT,
        metBetaXY=coordio.defaults.MET_BETA_XY,
        bossBetaXY=coordio.defaults.BOSS_BETA_XY,
        apBetaXY=coordio.defaults.AP_BETA_XY,
        collisionSegBetaXY=None, collisionBuffer=2, lefthanded=False
    ):

        if collisionSegBetaXY is None:
            # find the point along the beta axis
            # that will "just" enclose the tip of the
            # beta arm + some "cornerProtection"
            tipFlat = 0.6  # mm, length of flat top beta arm
            betaPost = 0.5  # mm, radius of beta post that can catch fibers
            # how much space around tip to save, set to zero
            # to allow "corner brushes"?
            # cornerProtection = 0.3  # mm
            cornerProtection = 0.55
            dxTip = np.sqrt(collisionBuffer**2 + (tipFlat/2)**2) - cornerProtection
            #dxPost = r - betaPost
            dxPost = 0
            collisionSegBetaXY = [
                [dxPost, 0],  # beta shaft is 1mm diameter
                [coordio.defaults.BETA_LEN - dxTip, 0]
            ]

        else:
            # ensure it's listified
            collisionSegBetaXY = [list(x) for x in collisionSegBetaXY]

        return super().addRobot(
            robotID, holeID, list(basePos), list(iHat), list(jHat),
            list(kHat), list(dxyz), alphaLen, alphaOffDeg,
            betaOffDeg, elementHeight, self.scaleFac, list(metBetaXY),
            list(bossBetaXY), list(apBetaXY), collisionSegBetaXY,
            hasApogee, collisionBuffer, lefthanded
        )

    def addFiducial(self, fiducialID, xyzWok, collisionBuffer=1.5):
        return super().addFiducial(
            fiducialID, list(xyzWok), collisionBuffer
        )

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

        # get collision buffer from the first robot
        # no longer lives on the RobotGrid
        # note robots may have unique collision buffer sizes now...
        cb = list(self.robotDict.items())[0].collisionBuffer

        r = {}
        r["nRobots"] = self.nRobots
        r["epsilon"] = self.epsilon
        r["collisionBuffer"] = cb
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

        # _htmlPath = os.path.join(
        #     os.path.dirname(__file__),
        #     "..",
        #     "..",
        #     "etc",
        #     "robotGrid.html"
        # )

        # fp = open(os.path.join(os.getenv('KAIJU_DIR'), 'etc',
        #                        'robotGrid.html'), "r")

        fp = open(os.path.join(KAIJU_ETC_DIR, "robotGrid.html"), "r")

        html_str = ''
        for l in fp.readlines():
            l = l.replace("fieldfile", "'" + os.path.basename(fieldfile) + "'")
            html_str = html_str + l
        fp.close()

        fp = open(filebase + '.html', "w")
        fp.write(html_str)
        fp.close()

        fp = open(os.path.join(KAIJU_ETC_DIR, 'robotGrid.js'), "r")
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
            robot_array[i]['robotID'] = crd.id
            robot_array[i]['xPos'] = crd.basePos[0]
            robot_array[i]['yPos'] = crd.basePos[1]
            robot_array[i]['hasApogee'] = crd.hasApogee
            robot_array[i]['hasBoss'] = crd.hasBoss
            robot_array[i]['assignedTargetID'] = crd.assignedTargetID
            robot_array[i]['isAssigned'] = crd.isAssigned()
            robot_array[i]['isCollided'] = self.isCollided(crd.id)
            robot_array[i]['alpha'] = crd.alpha
            robot_array[i]['beta'] = crd.beta
        return(robot_array)

    def target_array(self):
        """Return ndarray with target information"""
        target_dtype = np.dtype([('targetID', np.int64),
                                 ('x', np.float64),
                                 ('y', np.float64),
                                 ('z', np.float64),
                                 ('priority', np.int32),
                                 ('fiberType', np.string_, 30)])

        target_array = np.zeros(len(self.targetDict), dtype=target_dtype)
        i = 0
        for k, td in self.targetDict.items():
            td = self.targetDict[k]
            target_array['targetID'][i] = td.id
            target_array['x'][i] = td.xWok
            target_array['y'][i] = td.yWok
            target_array['z'][i] = td.zWok
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
            xyzWok = [
                target_array['x'][it],
                target_array['y'][it],
                target_array['z'][it]
            ]
            self.addTarget(targetID=target_array['targetID'][it],
                           xyzWok=xyzWok,
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

    def _load_from_tables(self, positionerTable, wokCoords, fiducialCoords):

        df = pd.merge(positionerTable, wokCoords, on="holeID").reset_index()

        for ii, row in df.iterrows():
            self.addRobot(
                robotID=row.positionerID,
                holeID=row.holeID,
                basePos=[row.xWok, row.yWok, row.zWok],
                hasApogee=bool("Apogee" in row.holeType),
                iHat=[row.ix, row.iy, row.iz],
                jHat=[row.jx, row.jy, row.jz],
                kHat=[row.kx, row.ky, row.kz],
                dxyz=[row.dx, row.dy, 0],
                alphaLen=row.alphaArmLen,
                alphaOffDeg=row.alphaOffset,
                betaOffDeg=row.betaOffset,
                elementHeight=coordio.defaults.POSITIONER_HEIGHT,
                metBetaXY=[row.metX, row.metY],
                bossBetaXY=[row.bossX, row.bossY],
                apBetaXY=[row.apX, row.apY],
                collisionSegBetaXY=None,
                collisionBuffer=2
            )

        for ii, row in fiducialCoords.iterrows():
            self.addFiducial(
                fiducialID=int(row.id.strip("F")),
                xyzWok=[row.xWok, row.yWok, row.zWok],
                collisionBuffer=3
            )

        # gfa locations hard-coded for now
        # eventually read from a table
        gfaList = [
            [
                [-267.49, 89.15, coordio.defaults.POSITIONER_HEIGHT],
                [-210.95, 187.08, coordio.defaults.POSITIONER_HEIGHT]
            ],
            [
                [-56.54, 276.23, coordio.defaults.POSITIONER_HEIGHT],
                [56.54, 276.23, coordio.defaults.POSITIONER_HEIGHT]
            ],
            [
                [210.95, 187.07, coordio.defaults.POSITIONER_HEIGHT],
                [267.49, 89.15, coordio.defaults.POSITIONER_HEIGHT]
            ],
            [
                [267.49, -89.15, coordio.defaults.POSITIONER_HEIGHT],
                [210.95, -187.08, coordio.defaults.POSITIONER_HEIGHT]
            ],
            [
                [56.54, -276.23, coordio.defaults.POSITIONER_HEIGHT],
                [-56.54, -276.23, coordio.defaults.POSITIONER_HEIGHT]
            ],
            [
                [-210.95, -187.07, coordio.defaults.POSITIONER_HEIGHT],
                [-267.49, -89.15, coordio.defaults.POSITIONER_HEIGHT]
            ]
        ]

        for ii, collisionSegXYZWok in enumerate(gfaList):
            self.addGFA(ii, collisionSegXYZWok, collisionBuffer=3)

        self.initGrid()

    def plot_state(
            self, highlightRobot=None, plotRobotIDs=True,
            returnax=True, figname="kaijuGrid.png"
        ):
        """Plot the current state of the robot grid

        Unassigned robots are skyblue
        Robots at their destination are gold
        Robots that are kaiju-collided are red
        Offline robots are black

        Parameters
        ------------
        highlightRobot : int, or None
            Robot id to highlight in Orange

        plotRobotIDs : bool
            If True, plot the robotIDs

        returnax : bool
            If True, return the matplotlib axis (no figure is written to disk)

        figname : str
            If returnax is False, write the image to disk with this path.
            Defaults to kaijuGrid.png.

        Returns
        ---------
        ax : matplotlib axes instance if returnax is True

        """
        from kaiju.utils import plotOne
        if returnax:
            ax = plotOne(
                step=0, robotGrid=self, isSequence=False,
                plotRobotIDs=plotRobotIDs,
                highlightRobot=highlightRobot, returnax=True
            )
            return ax
        else:
            plotOne(
                step=0, robotGrid=self, figname=figname, isSequence=False,
                plotRobotIDs=plotRobotIDs,
                highlightRobot=highlightRobot
            )


class RobotGridCalib(RobotGrid):

    def _load_grid(self):
        self._load_from_tables(
            calibration.positionerTable,
            calibration.wokCoords,
            calibration.fiducialCoords
        )


class RobotGridNominal(RobotGrid):
    # constructed from a "perfect" grid
    # useful for testing because calib grid will change overtime
    # this is a flat wok model, with GFAs
    def _load_grid(self):

        positionerTable = pd.read_csv(
            os.path.join(KAIJU_ETC_DIR, "positionerTable.csv")
        )

        wokCoords = pd.read_csv(
            os.path.join(KAIJU_ETC_DIR, "wokCoords.csv")
        )

        fiducialCoords = pd.read_csv(
            os.path.join(KAIJU_ETC_DIR, "fiducialCoords.csv")
        )

        self._load_from_tables(positionerTable, wokCoords, fiducialCoords)


