from kaiju import utils, RobotGrid
import json
import os
import time
from multiprocessing import Pool, cpu_count
from functools import partial
import numpy
import itertools


saveDir = "/home/csayres/kaijuRun"

nProcs = 26

# nTrials = 15
seeds = range(0, 100)
cbuff = [1.5, 1.75, 2, 2.25, 2.5, 2.75, 3]
angStep = [0.01]
greed = [-1]
phobia = [-1]
maxReplacements = 60
hasApogee = True
nDia = 17 #27 #is full run

alphaDest = 10
betaDest = 170
xPos, yPos = xPos, yPos = utils.hexFromDia(nDia, pitch=22.4, rotAngle=90)

def getGrid(angStep, cbuff, seed):
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

    rg.decollideGrid()
    # print("nCollisions in getGrid", rg.getNCollisions())
    return rg


def doOne(inputList):
    seed, angStep, cbuff, greed, phobia = inputList
    outList = []
    filename = "summary_%i_%.2f_%.2f_%.2f_%.2f.json"%(seed, cbuff, angStep, greed, phobia)
    filepath = os.path.join(saveDir, filename)
    try:
        rg = getGrid(angStep, cbuff, seed)
    except:
        outList.append("decollide failed")
        json.dump(outList, open(filepath, "w", separators=(',', ':')))
        return

    # record initial alpha beta positions for all robots
    rbInit = {}
    for rID, robot in rg.robotDict.items():
        rbInit[rID] = [robot.alpha, robot.beta]

    for i in range(maxReplacements):
        for rID, robot in rg.robotDict.items():
            initAlpha, initBeta = rbInit[rID]
            robot.setAlphaBeta(initAlpha, initBeta)
        # print("nCollisions at loop top", rg.getNCollisions())

        t1 = time.time()
        if greed == 1 and phobia == 0:
            rg.pathGenGreedy()
        elif greed == -1 and phobia == -1:
            rg.pathGen()
        else:
            rg.pathGenMDP(greed, phobia)
        runtime = time.time()-t1
        outList.append([runtime, rg.robotGridSummaryDict()])
        if not rg.didFail:
            break # done!

        deadlockedRobots = numpy.array(rg.deadlockedRobots())
        numpy.random.shuffle(deadlockedRobots)
        # print("deadlockedRobots", deadlockedRobots)
        # if we're here pathGen failed.  try to replace a deadlocked
        # robot
        for rID, robot in rg.robotDict.items():
            initAlpha, initBeta = rbInit[rID]
            robot.setAlphaBeta(initAlpha, initBeta)
        # print("nCollisions after deadlock", rg.getNCollisions())
        foundNewSpot = False
        for rID in deadlockedRobots:
            # try 300 times to find a new spot for this guy
            for jj in range(300):
                robot = rg.robotDict[rID]
                robot.setXYUniform()
                if rg.getNCollisions()==0:
                    foundNewSpot = True
                    rbInit[rID] = [robot.alpha, robot.beta]
                    break
            if foundNewSpot:
                break
        if not foundNewSpot:
            outList.append("failed to find valid replacement")
            break

    with open(filepath, "w") as f:
        json.dump(outList, f, separators=(',', ':'))


if __name__ == "__main__":

    # this deadlocks with nDia = 27
    # seed=19
    # cbuff=1.5
    # angStep=1.5
    # greed=0.9
    # phobia=0.2
    # inputList = [seed, angStep, cbuff, greed, phobia]
    # doOne(inputList)


# use itertools for better load balancing
   # seeds = range(nTrials)
    gridIter = itertools.product(seeds,angStep,cbuff,greed,phobia)
    p = Pool(nProcs)
    p.map(doOne, gridIter)
    p.close()

    # first = list(gridIter)[0]
    # doOne(first)

