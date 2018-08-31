
import copy
from multiprocessing import Pool
import math
import sys
from functools import partial
import shutil
import os

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
BetaArmWidth = 5  # mm
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
    def __init__(self, id, xPos, yPos, alpha, beta, tcCoords, bcCoords, isTC, isBC):
        self.id = id
        self.xPos = xPos
        self.yPos = yPos
        self.alpha = alpha
        self.beta = beta
        self.tcCoords = tcCoords
        self.bcCoords = bcCoords
        self.isTC = isTC
        self.isBC = isBC

        lineBuffer = BetaArmWidth / 2.0

        self.topCollideLine = LineString(
            [[tcCoords[0], tcCoords[1]], [tcCoords[2], tcCoords[3]]]
        ).buffer(lineBuffer, cap_style=1)

        self.bottomCollideLine = LineString(
            [[bcCoords[0], bcCoords[1]], [bcCoords[2], bcCoords[3]]]
        ).buffer(lineBuffer, cap_style=1)


def getRobotList(filename):
    with open(filename, "r") as f:
        fileLines = f.readlines()
    robotList = []
    for line in fileLines:
        if line.startswith("#"):
            continue
        id, xPos, yPos, alpha, beta, tcx1, tcy1, tcx2, tcy2, bcx1, bcy1, bcx2, bcy2, isTC, isBC = line.split(",")
        tcCoords = [float(tcx1), float(tcy1), float(tcx2), float(tcy2)]
        bcCoords = [float(bcx1), float(bcy1), float(bcx2), float(bcy2)]
        robotList.append(
            Robot(
                int(id),
                float(xPos),
                float(yPos),
                float(alpha),
                float(beta),
                tcCoords,
                bcCoords,
                int(isTC),
                int(isBC)
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
        topcolor, bottomcolor = "blue", "green"
        if robot.isTC == 1:
            topcolor = "red"
        if robot.isBC == 1:
            bottomcolor = "red"
        plt.plot([robot.xPos, robot.bcCoords[0]], [robot.yPos, robot.bcCoords[1]], 'k', linewidth=3)
        patch = PolygonPatch(robot.topCollideLine, fc=topcolor, ec=topcolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)
        patch = PolygonPatch(robot.bottomCollideLine, fc=bottomcolor, ec=bottomcolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)
    plt.axis('equal')
    if title is not None:
        plt.title(title)
    if xlim:
        plt.xlim(xlim)
    if ylim:
        plt.ylim(ylim)
    if save:
        plt.savefig(title)
        plt.close()


if __name__ == "__main__":
    i = 0
    while True:
        filename = "step_%i.txt" % i
        if os.path.exists(filename):
            roboList = getRobotList(filename)
            zeropad = ("%i" % i).zfill(4)
            plotGrid(roboList, xlim=[-300,300], ylim=[-300,300], title="step_%s.png" % zeropad, save=True)
        else:
            break
        i += 1

