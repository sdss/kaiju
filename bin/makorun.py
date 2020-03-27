# from kaiju import utils, RobotGrid
# import json
import os
import time
from multiprocessing import Pool, cpu_count
from functools import partial
import numpy
# numpy.random.seed(0)
# import networkx as nx
import itertools

from runSim import doOne, compileResults


# saveDir = "/home/csayres/kaijuRun"

# nProcs = 24

# # nTrials = 15
# seeds = range(0, 2000)
# cbuff = [1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.25, 3.5]
# angStep = [0.1]
# greedPhob = [(0.9, 0.3),(1, 0)]
# # greed = [-1]
# # phobia = [-1]
# maxReplacements = 60
# hasApogee = True
# nDia = 27 #is full run

# alphaDest = 10
# betaDest = 170
# xPos, yPos = xPos, yPos = utils.hexFromDia(nDia, pitch=22.4, rotAngle=90)

# def getGrid(angStep, cbuff, seed):
#     rg = RobotGrid(
#         stepSize=angStep, collisionBuffer=cbuff,
#         epsilon=angStep*2, seed=seed
#     )
#     for robotID, (x,y) in enumerate(zip(xPos,yPos)):
#         rg.addRobot(robotID, x, y, hasApogee)
#     rg.initGrid()
#     for robot in rg.robotDict.values():
#         robot.setXYUniform()
#         robot.setDestinationAlphaBeta(10,170)

#     rg.decollideGrid()
#     # print("nCollisions in getGrid", rg.getNCollisions())
#     return rg

# def findReplacementIDs(rg):
#     G = nx.Graph()
#     deadlockedRobots = rg.deadlockedRobots()
#     print("deadlockedRobots", deadlockedRobots)
#     G.add_nodes_from(deadlockedRobots)

#     for dlr in deadlockedRobots:
#         neighbors = rg.robotDict[dlr].robotNeighbors
#         for nID in neighbors:
#             if nID in deadlockedRobots:
#                 G.add_edge(dlr, nID)
#     subGraphs = nx.connected_components(G)
#     # this picks one out of each connected
#     # deadlock pileup
#     return [numpy.random.choice(list(x)) for x in subGraphs]

# def doOne(inputList):
#     seed, angStep, cbuff, (greed, phobia) = inputList
#     outList = []
#     basename = "%i_%.2f_%.2f_%.2f_%.2f"%(seed, cbuff, angStep, greed, phobia)
#     filename = "summary_" + basename + ".json"
#     errname = "error_" + basename + ".err"
#     errpath = os.path.join(saveDir, errname)
#     filepath = os.path.join(saveDir, filename)
#     try:
#         rg = getGrid(angStep, cbuff, seed)
#     except:
#         with open(errpath, "a") as f:
#             f.write("decollide failed\n")
#         return

#     totalReplaced = 0
#     # record initial alpha beta positions for all robots
#     rbInit = {}
#     for rID, robot in rg.robotDict.items():
#         rbInit[rID] = [robot.alpha, robot.beta]

#     for i in range(maxReplacements):
#         # get a fresh grid each time
#         rg = getGrid(angStep, cbuff, seed)
#         rg.totalReplaced = totalReplaced
#         # initialize robots to starting place
#         for rID, robot in rg.robotDict.items():
#             initAlpha, initBeta = rbInit[rID]
#             robot.setAlphaBeta(initAlpha, initBeta)
#         # print("nCollisions at loop top", rg.getNCollisions())

#         if greed == 1 and phobia == 0:
#             rg.pathGenGreedy()
#         # elif greed == -1 and phobia == -1:
#         #     rg.pathGen()
#         else:
#             rg.pathGenMDP(greed, phobia)
#         outList.append(rg.robotGridSummaryDict())
#         if not rg.didFail:
#             break # done!

#         # deadlockedRobots = numpy.array(rg.deadlockedRobots())
#         # numpy.random.shuffle(deadlockedRobots)
#         replacementIDs = findReplacementIDs(rg)
#         # print("deadlockedRobots", deadlockedRobots)
#         # if we're here pathGen failed.  try to replace a deadlocked
#         # robot
#         for rID, robot in rg.robotDict.items():
#             initAlpha, initBeta = rbInit[rID]
#             robot.setAlphaBeta(initAlpha, initBeta)
#         # print("nCollisions after deadlock", rg.getNCollisions())

#         for rID in replacementIDs:
#             # throwAway will update the grid with new positions
#             # if successful. returns true/false to indicate success
#             robot = rg.robotDict[rID]
#             foundNewSpot = rg.throwAway(rID)
#             if foundNewSpot == False:
#                 with open(errpath, "a") as f:
#                     f.write("failed to find replacement for robot %i replacement iter %i\n"%(rID, i))
#             else:
#                 # indicate this robots new spot in the init dict
#                 rbInit[rID] = [robot.alpha, robot.beta]
#                 totalReplaced += 1


#     with open(filepath, "w") as f:
#         json.dump(outList, f, separators=(',', ':'))


if __name__ == "__main__":

    saveDir = "/home/csayres/kaijuRun"

    nProcs = 24

    # nTrials = 15
    seeds = range(0, 2)
    cbuff = [1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.25, 3.5]
    angStep = [0.1]
    greedPhob = [(0.9, 0.3),(1, 0)]
    nDia = [27] #is full run
    # this deadlocks with nDia = 27
    # seed=19
    # cbuff=1.5
    # angStep=1.5
    # greed=0.9
    # phobia=0.2
    # inputList = [seed, angStep, cbuff, greed, phobia]
    # doOne(inputList)

    doOnePartial = partial(doOne, saveDir=saveDir)

    gridIter = itertools.product(seeds,nDia,angStep,cbuff,greedPhob)
    p = Pool(nProcs)
    p.map(doOnePartial, gridIter)
    p.close()
    compileResults(saveDir, "allSeeds.csv")

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

