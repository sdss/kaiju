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

nTrials = 20
cbuff = [1.5, 2, 2.5, 3, 3.5]
angStep = [0.05, 0.1, 0.5, 1, 1.5]
greed = [1, 0.95, 0.9, 0.8, 0.7]
phobia = [0, 0.1, 0.2, 0.3]
maxReplacements = 40
hasApogee = True
nDia = 27 #is full run

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
    print("nCollisions in getGrid", rg.getNCollisions())
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
        print("nCollisions at loop top", rg.getNCollisions())

        t1 = time.time()
        rg.pathGenMDP(greed, phobia)
        runtime = time.time()-t1
        outList.append([runtime, rg.robotGridSummaryDict()])
        if not rg.didFail:
            break # done!

        deadlockedRobots = numpy.array(rg.deadlockedRobots())
        numpy.random.shuffle(deadlockedRobots)
        print("deadlockedRobots", deadlockedRobots)
        # if we're here pathGen failed.  try to replace a deadlocked
        # robot
        for rID, robot in rg.robotDict.items():
            initAlpha, initBeta = rbInit[rID]
            robot.setAlphaBeta(initAlpha, initBeta)
        print("nCollisions after deadlock", rg.getNCollisions())
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
    seed=19
    cbuff=1.5
    angStep=1.5
    greed=0.9
    phobia=0.2
    inputList = [seed, angStep, cbuff, greed, phobia]
    doOne(inputList)


# use itertools for better load balancing
# seeds = range(nTrials)
# gridIter = itertools.product(seeds,angStep,cbuff,greed,phobia)
# p = Pool(nProcs)
# p.map(doOne, gridIter)
# p.close()

