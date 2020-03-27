from kaiju import utils, RobotGrid
import json
import os
import time
from multiprocessing import Pool, cpu_count
from functools import partial
import numpy
numpy.random.seed(0)
import networkx as nx
import itertools
import glob
import pandas as pd


saveDir = "/home/csayres/kaijuRunVarNum"

nProcs = 14

# nTrials = 15
seeds = range(0, 200)
cbuff = 2.5
angStep = 0.1

# cbuff = [1.5, 2, 2.5, 3, 3.5]
# angStep = [0.01, 0.05, 0.1, .25, 0.5, 0.75, 1]#, 1]
greedPhob = [(0.9, 0.3),(1, 0)]
nDias = numpy.arange(7,67,4)
# greed = [-1]
# phobia = [-1]
maxReplacements = 60
hasApogee = True
# nDia = 27 #is full run

alphaDest = 10
betaDest = 170
# xPos, yPos = xPos, yPos = utils.hexFromDia(nDia, pitch=22.4, rotAngle=90)

def getGrid(angStep, cbuff, seed, nDia):
    rg = RobotGrid(
        stepSize=angStep, collisionBuffer=cbuff,
        epsilon=angStep*2, seed=seed
    )
    xPos, yPos = xPos, yPos = utils.hexFromDia(nDia, pitch=22.4, rotAngle=90)
    for robotID, (x,y) in enumerate(zip(xPos,yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
    rg.initGrid()
    for robot in rg.robotDict.values():
        robot.setXYUniform()
        robot.setDestinationAlphaBeta(10,170)

    rg.decollideGrid()
    # print("nCollisions in getGrid", rg.getNCollisions())
    return rg

def findReplacementIDs(rg):
    G = nx.Graph()
    deadlockedRobots = rg.deadlockedRobots()
    print("deadlockedRobots", deadlockedRobots)
    G.add_nodes_from(deadlockedRobots)

    for dlr in deadlockedRobots:
        neighbors = rg.robotDict[dlr].robotNeighbors
        for nID in neighbors:
            if nID in deadlockedRobots:
                G.add_edge(dlr, nID)
    subGraphs = nx.connected_components(G)
    # this picks one out of each connected
    # deadlock pileup
    return [numpy.random.choice(list(x)) for x in subGraphs]

def doOne(inputList):
    seed, nDia, (greed, phobia) = inputList
    outList = []
    basename = "%i_%i_%.2f_%.2f"%(seed, nDia, greed, phobia)
    filename = "summary_" + basename + ".json"
    errname = "error_" + basename + ".err"
    errpath = os.path.join(saveDir, errname)
    filepath = os.path.join(saveDir, filename)
    if os.path.exists(filepath):
        print("skipping, already done")
        return
    try:
        rg = getGrid(angStep, cbuff, seed, nDia)
    except:
        with open(errpath, "a") as f:
            f.write("decollide failed\n")
        return

    totalReplaced = 0
    # record initial alpha beta positions for all robots
    rbInit = {}
    for rID, robot in rg.robotDict.items():
        rbInit[rID] = [robot.alpha, robot.beta]

    for i in range(maxReplacements):
        # get a fresh grid each time
        if totalReplaced > rg.nRobots:
            rg.totalReplaced = rg.nRobots
            # don't start a new grid
            # don't try further
            # break here
            outList.append(rg.robotGridSummaryDict())
            break
        rg = getGrid(angStep, cbuff, seed, nDia)
        rg.totalReplaced = totalReplaced
        # initialize robots to starting place
        for rID, robot in rg.robotDict.items():
            initAlpha, initBeta = rbInit[rID]
            robot.setAlphaBeta(initAlpha, initBeta)
        # print("nCollisions at loop top", rg.getNCollisions())

        tstart = time.time()
        if greed == 1 and phobia == 0:
            rg.pathGenGreedy()
        # elif greed == -1 and phobia == -1:
        #     rg.pathGen()
        else:
            rg.pathGenMDP(greed, phobia)
        tend = time.time()
        rg.runtime = tend-tstart
        outList.append(rg.robotGridSummaryDict())
        if not rg.didFail:
            break # done!

        # deadlockedRobots = numpy.array(rg.deadlockedRobots())
        # numpy.random.shuffle(deadlockedRobots)
        replacementIDs = findReplacementIDs(rg)
        # print("deadlockedRobots", deadlockedRobots)
        # if we're here pathGen failed.  try to replace a deadlocked
        # robot
        for rID, robot in rg.robotDict.items():
            initAlpha, initBeta = rbInit[rID]
            robot.setAlphaBeta(initAlpha, initBeta)
        # print("nCollisions after deadlock", rg.getNCollisions())

        for rID in replacementIDs:
            # throwAway will update the grid with new positions
            # if successful. returns true/false to indicate success
            robot = rg.robotDict[rID]
            foundNewSpot = rg.throwAway(rID)
            if foundNewSpot == False:
                with open(errpath, "a") as f:
                    f.write("failed to find replacement for robot %i replacement iter %i\n"%(rID, i))
            else:
                # indicate this robots new spot in the init dict
                rbInit[rID] = [robot.alpha, robot.beta]
                totalReplaced += 1


    with open(filepath, "w") as f:
        json.dump(outList, f, separators=(',', ':'))

def compileResults():
    speed = 5*360/60. # degrees per sec (5 RPM)
    # nRobots = 547
    maxReplacements = 40


    pandasDict = {
        "seed" : [],
        "efficiency" : [],
        "replacements" : [],
        "collisionBuffer" : [],
        "greed" : [],
        "phobia" : [],
        "angStep" : [],
        "nSteps" : [],
        "foldtime" : [],
        "nRobots" : [],
        "foldDeg" : [],
        "algorithm": [],
        "totalRuntime": [],
        "lastRuntime": []
    }


    files = glob.glob(os.path.join(saveDir, "*.json"))
    t1 = time.time()
    for file in files:
        f = json.load(open(file, "r"))
        file = file.strip(".json")
        # all strings
        nRobots = f[0]["nRobots"]
        seed = f[0]["seed"]
        angStep = f[0]["angStep"]
        collisionBuffer = f[0]["collisionBuffer"]
        greed = f[0]["greed"]
        phobia = f[0]["phobia"]
        totalReplaced = f[-1]["totalReplaced"]
        totalRuntime = numpy.sum(x["runtime"] for x in f) # sum runtime including replacements
        pandasDict["totalRuntime"].append(totalRuntime)
        pandasDict["lastRuntime"].append(f[-1]["runtime"])
        if greed == -1:
            pandasDict["algorithm"].append("fold")
        elif greed == 1:
            pandasDict["algorithm"].append("GC")
        else:
            pandasDict["algorithm"].append("MDP")
        pandasDict["nRobots"].append(nRobots)
        pandasDict["seed"].append(seed)
        pandasDict["collisionBuffer"].append(collisionBuffer)
        pandasDict["greed"].append(greed)
        pandasDict["phobia"].append(phobia)
        pandasDict["angStep"].append(angStep)
        replacementTries = len(f) # remember multiple replacements done at once
        pandasDict["replacements"] = replacementTries
        if totalReplaced >= nRobots:
            totalReplaced = nRobots
        pandasDict["efficiency"].append(1-totalReplaced/nRobots)
        if totalReplaced == nRobots:
            pandasDict["foldtime"].append(numpy.nan)
            pandasDict["nSteps"].append(numpy.nan)
            pandasDict["foldDeg"].append(numpy.nan)
        else:
            nSteps = f[-1]["nSteps"] # take the number of steps at last iter
            # print("nReplaced for file", file, replacements)
            pandasDict["foldtime"].append(nSteps*angStep/speed)
            pandasDict["nSteps"].append(nSteps)
            pandasDict["foldDeg"].append(nSteps*angStep)

        # print("read seed", seed)
    print("took", time.time()-t1)
    df = pd.DataFrame(pandasDict)
    df.to_csv(os.path.join(saveDir, "allSeedsVarNum.csv"), index=False)


if __name__ == "__main__":

    # this deadlocks with nDia = 27
    # seed=19
    # cbuff=1.5
    # angStep=1.5
    # greed=0.9
    # phobia=0.2
    # inputList = [seed, angStep, cbuff, greed, phobia]
    # doOne(inputList)
    gridIter = itertools.product(seeds,nDias,greedPhob)
    p = Pool(nProcs)
    p.map(doOne, gridIter)
    p.close()
    compileResults()

# use itertools for better load balancing
   # seeds = range(nTrials)
   # MDP, Greedy, Fold algs
    # greedPhob = [[0.95, 0.2], [1, 0], [-1, -1]]
    # for g, p in greedPhob:
    #     print("on greed/phob", g, p)
    #     greed = [g]
    #     phobia = [p]
    #     gridIter = itertools.product(seeds,angStep,cbuff,greed,phobia)
    #     p = Pool(nProcs)
    #     p.map(doOne, gridIter)
    #     p.close()

    # first = list(gridIter)[0]
    # doOne(first)

