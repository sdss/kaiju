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
import coordio

matplotlib.use('Agg')

# internalBuffer = 1.5
rg = None # need global because C++ obj can't be pickled for multiprocessing

def plotTraj(r, figprefix="traj_", dpi=500):
    # r is a robot
    spa = numpy.array(r.smoothedAlphaPath)
    spb = numpy.array(r.smoothedBetaPath)
    rpa = numpy.array(r.alphaPath)
    rpb = numpy.array(r.betaPath)
    aRDP = numpy.array(r.simplifiedAlphaPath);
    bRDP = numpy.array(r.simplifiedBetaPath);


    av = numpy.array(r.alphaVel)
    bv = numpy.array(r.betaVel)
    vSteps = numpy.arange(len(av))
    sav = numpy.array(r.smoothAlphaVel)
    sbv = numpy.array(r.smoothBetaVel)
    ss = numpy.arange(len(sav))

    # print("plotting", r.id)
    # print("alpha start", rpa[0,:] - aRDP[0,:])
    # print("alpha end", rpa[-1,:] - aRDP[-1,:])
    # print("beta start", rpb[0,:] - bRDP[0,:])
    # print("beta end", rpb[-1,:] - bRDP[-1,:])

    fig, ax = plt.subplots(2,1, figsize=(10,10))


    ax[0].plot(rpa[:,0], rpa[:,1], linewidth=0.2, label="rough alpha", alpha=0.8)
    ax[0].plot(rpb[:,0], rpb[:,1], linewidth=0.2, label="rough beta", alpha=0.8)
    ax[0].plot(spa[:,0], spa[:,1], 'k-', linewidth=0.2, label="smooth alpha")
    ax[0].plot(spb[:,0], spb[:,1], 'k-', linewidth=0.2, label="smooth beta")
    ax[0].plot(
        aRDP[:,0], aRDP[:,1], 'oc-', linewidth=0.2, markeredgewidth=0.4,
        fillstyle="none", markersize=2, label="RDP alpha", alpha=0.7
    )
    ax[0].plot(
        bRDP[:,0], bRDP[:,1], 'oc-', linewidth=0.2, markeredgewidth=0.4,
        fillstyle="none", markersize=2, label="RDP beta", alpha=0.7
    )
    ax[0].legend()

    ax[1].plot(vSteps, av, linewidth=0.2, label="alphaVel", alpha=0.4)
    ax[1].plot(vSteps, bv, linewidth=0.2, label="betaVel", alpha=0.4)
    ax[1].plot(ss, sav, 'k-', linewidth=0.2, label="smoothAlpha")
    ax[1].plot(ss, sbv, 'k-', linewidth=0.2, label="smoothBeta")

    ax[1].legend()
    # plt.legend()


    plt.savefig(figprefix+"robot_%s.pdf"%r.id, dpi=dpi)
    plt.close()


def plotOne(
    step, robotGrid=None, figname=None, isSequence=True,
    plotTargets=False, plotRobotIDs=False, dpi=100,
    xlim=None, ylim=None, highlightRobot=None, returnax=False
):
    global rg
    if hasattr(step, "__len__"):
        fig = step[1]
        step = step[0]
    else:
        fig = step

    if robotGrid is not None:
        rg = robotGrid
    plt.figure(figsize=(10,10))
    ax = plt.gca()
    maxX = 0
    maxY = 0
    for robotID, robot in rg.robotDict.items():
        # account for xy calibration offsets in base
        # pos of robot in wok space
        wokBase = coordio.libcoordio.tangentToWok(
            [0,0,0],
            robot.basePos,
            robot.iHat,
            robot.jHat,
            robot.kHat,
            robot.elementHeight,
            robot.scaleFac,
            robot.dxyz[0],
            robot.dxyz[1],
            robot.dxyz[2]
        )

        basePos = wokBase[:-1]
        # basePos = robot.basePos

        if basePos[0] > maxX:
            maxX = basePos[0]
        if basePos[1] > maxY:
            maxY = basePos[1]
        if isSequence:
            alphaX = robot.roughAlphaX[step][1]
            alphaY = robot.roughAlphaY[step][1]
            betaX = robot.roughBetaX[step][1]
            betaY = robot.roughBetaY[step][1]
            onTarget = False
            # onTarget = robot.onTargetVec[step]
        else:
            # step input is ignored!!
            alphaPoint = robot.collisionSegWokXYZ[0]
            betaPoint = robot.collisionSegWokXYZ[1]
            alphaX = alphaPoint[0]
            alphaY = alphaPoint[1]
            betaX = betaPoint[0]
            betaY = betaPoint[1]
            onTarget = robot.score() == 0
        plt.plot(
            [basePos[0], alphaX], [basePos[1], alphaY],
            color='black', linewidth=2, alpha=0.5
        )

        topCollideLine = LineString(
            [(alphaX, alphaY), (betaX, betaY)]
        ).buffer(robot.collisionBuffer, cap_style=1)
        topcolor = 'blue'
        edgecolor = 'black'
        if not robot.isAssigned():
            topcolor = "skyblue"
        if onTarget:
            topcolor = "gold"
        if not isSequence and rg.isCollided(robotID):
            # collision trumps not assigned
            topcolor = "red"
        if highlightRobot == robotID:
            topcolor = "orange"
        if robot.isOffline:
            topcolor = "black"
        patch = PolygonPatch(
            topCollideLine, fc=topcolor, ec=edgecolor, alpha=0.5, zorder=10
        )
        ax.add_patch(patch)
        if plotRobotIDs:
            strID = ("%i"%robot.id).zfill(4)
            xm = (alphaX + betaX)/2
            ym = (alphaY + betaY)/2
            dx = alphaX-betaX
            dy = alphaY-betaY
            rot = numpy.degrees(numpy.arctan2(dy, dx))
            if alphaX < betaX:
                rot += 180
            props = {"ha": "center", "va": "center", "fontsize": 2}
            ax.text(xm, ym, strID, props, rotation=rot)

    for fiducialID, fiducial in rg.fiducialDict.items():

        fPoint = Point(
            fiducial.xyzWok[0], fiducial.xyzWok[1]
        ).buffer(fiducial.collisionBuffer) #, cap_style=1)

        patch = PolygonPatch(fPoint, fc="cyan", ec="black", alpha=0.8, zorder=10)
        ax.add_patch(patch)
    for gfa in rg.gfaDict.values():

        gLine = LineString(
            [gfa.collisionSegWokXYZ[0], gfa.collisionSegWokXYZ[1]]
        ).buffer(gfa.collisionBuffer, cap_style=1)
        patch = PolygonPatch(gLine, fc="cyan", ec="black", alpha=0.5, zorder=10)
        ax.add_patch(patch)

    if xlim is not None:
        ax.set_xlim(xlim)
    if ylim is not None:
        ax.set_ylim(ylim)
    ax.set_xlim([-maxX-50, maxX+50])
    ax.set_ylim([-maxY-50, maxY+50])
    ax.set_aspect("equal")

    if returnax:
        return ax

    if figname is None:
        figname = "step_%04d.png"%(fig)
    plt.savefig(figname, dpi=dpi)
    plt.close()

def plotPaths(robotGrid, downsample=None, filename=None):
    # figure out how to downsample paths
    global rg
    rg = robotGrid
    steps = list(range(robotGrid.nSteps)) # not sure why rg.nSteps is broken
    if downsample is not None:
        steps = steps[::downsample] #+ [steps[-1]]
        # throwout last step, sometimes is out of range index depending
        # on which downsample was selected
        steps = steps[:-1]


    figs = list(range(len(steps)))
    stepfigs = list(zip(steps, figs))
    print("plotting steps", figs[-1])
    # down sample if nframes specified

    p = Pool(cpu_count())
    p.map(plotOne, stepfigs)

    # for stepFig in stepfigs:
    #     print("stepfig", stepFig)
    #     plotOne(stepFig)

    fps = 30 # frames per second
    if filename is None:
        filename = "example.mp4"
    # clobber file if it already exists
    if os.path.exists(filename):
        os.remove(filename)
    args = ['ffmpeg', '-r', '%i'%fps, '-f', 'image2', '-i', 'step_%04d.png',
            '-pix_fmt', 'yuv420p', filename]

    movie = Popen(args)
    movie.wait()

    # clean up imgs
    imgs = glob.glob("step*.png")
    for img in imgs:
        os.remove(img)


def hexFromDia(nDia, pitch=22.4, rotAngle=0):
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

    # rotate grid?
    if rotAngle:
        cos = numpy.cos(numpy.radians(rotAngle))
        sin = numpy.sin(numpy.radians(rotAngle))
        rotMat = numpy.array([
            [cos, sin],
            [-sin, cos]
        ])
        xyAll = numpy.array([xAll, yAll]).T
        xyRot = numpy.dot(xyAll, rotMat)
        xAll = xyRot[:,0]
        yAll = xyRot[:,1]
    # return arrays in a sorted order, rasterized. 1st position is lower left
    # last positon is top right
    ind = numpy.lexsort((xAll, yAll))
    return xAll[ind], yAll[ind]











