from kaiju import utils, RobotGrid
import json
import os
import time
from multiprocessing import Pool, cpu_count
from functools import partial
import numpy
import itertools


saveDir = "/home/csayres/kaijuRun"

nProcs = 23

nTrials = 100
cbuff = [1.5, 2, 2.5, 3, 3.5]
angStep = [0.05, 0.1, 0.5, 1, 1.5]
greed = [1, 0.95, 0.9, 0.8, 0.7]
phobia = [0, 0.1, 0.2, 0.3]
maxReplacements = 40
hasApogee = True
nDia = 27

alphaDest = 10
betaDest = 170
xPos, yPos = xPos, yPos = utils.hexFromDia(nDia, pitch=22.4, rotAngle=90)

def getGrid(angStep, cbuff, seed, replace=None):
    rg = RobotGrid(
        stepSize=angStep, collisionBuffer=cbuff,
        epsilon=angStep*2, seed=seed
    )
    for robotID, (x,y) in enumerate(zip(xPos,yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
    rg.initGrid()
    for robot in rg.robotDict.values():
        robot.setXYUniform()
        robot.setDestinationAlphaBeta(10,170)
    try:
        rg.decollideGrid()
    except:
        return "decollide failed"
    if replace is not None:
        replaceFailed = True
        robot = rg.robotDict[replace]
        while True:
            robot.setXYUniform()
            if not rg.isCollided(replace):
                replaceFailed = False
                break
        if replaceFailed:
            return "failed to replace"
    return rg



def doOne(inputList):
    seed, angStep, cbuff, greed, phobia = inputList
    outList = []
    filename = "summary_%i_%.2f_%.2f_%.2f_%.2f.json"%(seed, cbuff, angStep, greed, phobia)
    filepath = os.path.join(saveDir, filename)
    replaceRobot = None
    for i in range(maxReplacements):
        rg = getGrid(angStep, cbuff, seed, replaceRobot)
        if rg == "decollide failed":
            outList.append(rg)
            break
        elif rg == "failed to replace":
            outList.append(rg)
            break

        t1 = time.time()
        rg.pathGenMDP(greed, phobia)
        runtime = time.time()-t1
        outList.append([runtime, rg.robotGridSummaryDict()])
        if not rg.didFail:
            break
        replaceRobot = int(numpy.random.choice(rg.deadlockedRobots()))

    with open(filepath, "w") as f:
        json.dump(outList, f, separators=(',', ':'))


# use itertools for better load balancing
seeds = range(nTrials)
gridIter = itertools.product(seeds,angStep,cbuff,greed,phobia)
p = Pool(nProcs)
p.map(doOne, gridIter)
p.close()

