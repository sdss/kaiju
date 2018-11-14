
import copy
from multiprocessing import Pool
import math
import sys
from functools import partial
import shutil
import os
import multiprocessing
import glob

import numpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from descartes import PolygonPatch
from networkx import DiGraph, shortest_path, draw, find_cycle, simple_cycles, shortest_simple_paths, has_path

AlphaArmLength = 7.4  # mm
AlphaRange = [0, 360]
BetaRange = [0, 180]
BetaArmLength = 15  # mm, distance to fiber
BetaArmWidth = 4  # mm
MinTargSeparation = 8  # mm
# length mm along beta for which a collision cant happen
BetaTopCollide = [8.187, 16]  # box from length 8.187mm to 16mm
BetaBottomCollide = [0, 10.689]  # box from 0 to 10.689
BetaArmLengthSafe = BetaArmLength / 2.0
Pitch = 22.4  # mm fudge the 01 for neighbors
MinReach = BetaArmLength - AlphaArmLength
MaxReach = BetaArmLength + AlphaArmLength

# https://internal.sdss.org/trac/as4/wiki/FPSLayout


class Robot(object):
    def __init__(self, id, xPos, yPos, alpha, beta, xAlphaEnd, yAlphaEnd, xBetaEnd, yBetaEnd, isCollided):
        self.id = id
        self.xPos = xPos
        self.yPos = yPos
        self.alpha = alpha
        self.beta = beta
        self.xAlphaEnd = xAlphaEnd
        self.yAlphaEnd = yAlphaEnd
        self.xBetaEnd = xBetaEnd
        self.yBetaEnd = yBetaEnd
        self.isCollided = isCollided

        lineBuffer = BetaArmWidth / 2.0

        self.topCollideLine = LineString(
            [(xAlphaEnd, yAlphaEnd), (xBetaEnd, yBetaEnd)]
        ).buffer(lineBuffer, cap_style=1)


def getRobotList(filename):
    with open(filename, "r") as f:
        fileLines = f.readlines()
    robotList = []
    for line in fileLines:
        if line.startswith("#"):
            continue
        id, xPos, yPos, alpha, beta, xAlphaEnd, yAlphaEnd, xBetaEnd, yBetaEnd, isCollided = line.split(",")
        robotList.append(
            Robot(
                int(id),
                float(xPos),
                float(yPos),
                float(alpha),
                float(beta),
                float(xAlphaEnd),
                float(yAlphaEnd),
                float(xBetaEnd),
                float(yBetaEnd),
                int(isCollided),
            )
        )
    return robotList


def plotGrid(robotList, title=None, xlim=None, ylim=None, save=True):
    fig = plt.figure(figsize=(9, 9))
    ax = plt.gca()
    # ax = fig.add_subplot(111)
    xAll = [r.xPos for r in robotList]
    yAll = [r.yPos for r in robotList]
    plt.scatter(xAll, yAll)
    for robot in robotList:
        topcolor = "blue"
        if robot.isCollided == 1:
            topcolor = "red"
        plt.plot([robot.xPos, robot.xAlphaEnd], [robot.yPos, robot.yAlphaEnd], 'k', linewidth=3)
        # plt.plot([robot.xAlphaEnd, robot.xBetaEnd],[robot.yAlphaEnd, robot.yBetaEnd], color=topcolor, alpha=0.5, linewidth=5)
        patch = PolygonPatch(robot.topCollideLine, fc=topcolor, ec=topcolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)
    plt.axis('equal')
    if title is not None:
        plt.title(title)
    if xlim:
        plt.xlim(xlim)
    if ylim:
        plt.ylim(ylim)
    if save:
        plt.savefig(title, dpi=250)
        plt.close()

def doGrid(filename):
    basename, imgNum = filename.split("_")
    imgNum = int(imgNum.strip(".txt"))
    roboList = getRobotList(filename)
    zeropad = ("%i" % imgNum).zfill(4)
    plotGrid(roboList, xlim=[-300,300], ylim=[-300, 300], title="%s_%s.png" % (basename, zeropad), save=True)


def plotSet(basename):

    filenames = glob.glob(basename + "*.txt")
    p = multiprocessing.Pool(10)
    p.map(doGrid, filenames)

def plotSetSlow(basename):
    filenames = glob.glob(basename + "*.txt")[:10]
    for filename in filenames:
        doGrid(filename)



if __name__ == "__main__":
    # plotSet("interp_")
    plotSet("step_")
# ffmpeg -r 10 -f image2 -i interp_%04d.png -pix_fmt yuv420p robotMovie.mp4

