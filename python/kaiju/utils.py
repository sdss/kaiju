import matplotlib
import matplotlib.pyplot as plt
import numpy
from shapely.geometry import LineString, Point
from descartes import PolygonPatch
from multiprocessing import Pool, cpu_count
from subprocess import Popen
import glob
import os
from .cKaiju import RobotGrid

matplotlib.use('Agg')

# internalBuffer = 1.5
rg = None # need global because C++ obj can't be pickled for multiprocessing


def plotOne(step, robotGrid=None, figname=None, isSequence=True, plotTargets=False, internalBuffer=1.5):
    global rg

    if robotGrid is not None:
        rg = robotGrid
    plt.figure(figsize=(10,10))
    ax = plt.gca()
    for robotInd, robot in enumerate(rg.allRobots):
        if isSequence:
            alphaX = robot.roughAlphaX[step][1]
            alphaY = robot.roughAlphaY[step][1]
            betaX = robot.roughBetaX[step][1]
            betaY = robot.roughBetaY[step][1]
        else:
            # step input is ignored!!
            alphaPoint = robot.betaCollisionSegment[0]
            betaPoint = robot.betaCollisionSegment[1]
            alphaX = alphaPoint[0]
            alphaY = alphaPoint[1]
            betaX = betaPoint[0]
            betaY = betaPoint[1]
        plt.plot([robot.xPos, alphaX], [robot.yPos, alphaY], color='black', linewidth=2, alpha=0.5)

        topCollideLine = LineString(
            [(alphaX, alphaY), (betaX, betaY)]
        ).buffer(internalBuffer, cap_style=1)
        topcolor = 'blue'
        edgecolor = 'black'
        if rg.isCollidedInd(robotInd):
            topcolor = "red"
        if not robot.isAssigned():
            topcolor = "skyblue"
        patch = PolygonPatch(topCollideLine, fc=topcolor, ec=edgecolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)
    for fiducial in rg.fiducialList:
        fPoint = Point(fiducial[0], fiducial[1]).buffer(internalBuffer, cap_style=1)
        patch = PolygonPatch(fPoint, fc="cyan", ec="black", alpha=0.8, zorder=10)
        ax.add_patch(patch)

    if figname is None:
        figname = "step_%04d.png"%(step)
    plt.savefig(figname, dpi=250)
    plt.close()

def plotPaths(robotGrid, nframes=None):
    # figure out how to downsample paths
    global rg
    rg = robotGrid
    steps = range(robotGrid.nSteps)
    # down sample if nframes specified

    p = Pool(cpu_count())
    p.map(plotOne, steps)

    fps = 10 # frames per second
    args = ['ffmpeg', '-r', '%i'%fps, '-f', 'image2', '-i', 'step_%04d.png',
            '-pix_fmt', 'yuv420p', 'example.mp4']

    movie = Popen(args)
    movie.wait()

    # clean up imgs
    imgs = glob.glob("step*.png")
    for img in imgs:
        os.remove(img)

def hexFromDia(nDia, pitch = 22.4):
    """
    inputs:
    nDia: is points along the equator of the hex, must be odd

    returns:
    xArr, yArr: both nd arrays of x and y coordinates for each
    position in the hex. Units are mm and the center of hex is 0,0
    """
    # first determine the xy positions for the center line
    nRad = int((nDia + 1) / 2)
    xPosDia = numpy.arange(-nRad + 1, nRad) * pitch
    yPosDia = numpy.zeros(nDia)

    vertShift = numpy.sin(numpy.radians(60.0)) * pitch
    horizShift = numpy.cos(numpy.radians(60.0)) * pitch
    # populate the top half of the hex
    # determine number of rows above the diameter row
    xUpperHalf = []
    yUpperHalf = []
    for row in range(1, nRad):
        xPos = xPosDia[:-1 * row] + row * horizShift
        yPos = yPosDia[:-1 * row] + row * vertShift
        xUpperHalf.extend(xPos)
        yUpperHalf.extend(yPos)
    xUpperHalf = numpy.asarray(xUpperHalf).flatten()
    yUpperHalf = numpy.asarray(yUpperHalf).flatten()

    xLowerHalf = xUpperHalf[:]
    yLowerHalf = -1 * yUpperHalf[:]

    xAll = numpy.hstack((xLowerHalf, xPosDia, xUpperHalf)).flatten()
    yAll = numpy.hstack((yLowerHalf, yPosDia, yUpperHalf)).flatten()
    # return arrays in a sorted order, rasterized. 1st position is lower left
    # last positon is top right
    ind = numpy.lexsort((xAll, yAll))
    return xAll[ind], yAll[ind]

def robotGridFromFilledHex(stepSize, collisionBuffer, seed=0):
    gridFile = os.path.join(os.environ["KAIJU_DIR"], "etc", "fps_filledHex.txt")
    # Row   Col     X (mm)  Y (mm)          Assignment
    #
    #-13   0 -145.6000 -252.1866  BA
    #-13   1 -123.2000 -252.1866  BA
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


    epsilon = stepSize * 2.2

    rg = RobotGrid(stepSize, collisionBuffer, epsilon, seed)
    fiberID = 0
    hasApogee = False
    for b in bossXY:
        rg.addRobot(fiberID, b[0], b[1], hasApogee)
        fiberID += 1
    hasApogee = True
    for ba in baXY:
        rg.addRobot(fiberID, ba[0], ba[1], hasApogee)
        fiberID += 1
    for fiducial in fiducialXY:
        rg.addFiducial(fiducial[0], fiducial[1])
    rg.initGrid()
    return rg










