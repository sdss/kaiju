import numpy
import matplotlib.pyplot as plt

from ortools.constraint_solver import pywrapcp

from kaiju.robot import RobotGrid


if __name__ == "__main__":
    # nRobots = 500
    # nc = int(numpy.sqrt((nRobots*4-1)/3))
    # print("nc", nc)
    nc = 7
    rg = RobotGrid(nc)
    nRobots = len(rg.robotList)
    print("nrobots", nRobots)
    targetCache = rg.cachePositions()
    validTargs = []
    solveVars = []
    solver = pywrapcp.Solver("fps")
    # create variables
    for robot in rg.robotList:
        # validTargs.append()
        validTargs = [x.id for x in robot.swapList()] + [robot.id]
        solveVar = solver.IntVar(validTargs, "%i"%robot.id)
        solveVars.append(solveVar)
    # add constraints
    # for ii in range(nRobots):
    #     solveVar = solveVars[ii]
    #     otherVars = [solveVars[x] for x in validTargs[ii]]
    #     if otherVars:
    #         maxArgs = tuple(solveVar==otherVar for otherVar in otherVars)
    #         solver.Add(solver.Max(maxArgs)==1)

    # for ii in range(nRobots):
    #     for jj in range(nRobots):
    #         if ii == jj:
    #             continue
    #         solver.Add(solveVars[ii] != solveVars[jj])

    solver.Add(solver.AllDifferent(solveVars))
    # solver.AllDifferent(solveVars)
    db = solver.Phase(solveVars, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)
    solver.Solve(db)
    count = 0

    bestNCollide = None
    bestSolution = None
    while solver.NextSolution():
        count += 1
        solveInds = [x.Value() for x in solveVars]
        # print("solveInds", solveInds)
        # print("total", nRobots, len(set(solveInds)))
        solveInds = []
        for ii, robot in enumerate(rg.robotList):
            solveInd = solveVars[ii].Value()
            solveInds.append(solveInd)
            robot.setAlphaBetaFromFocalXY(*targetCache[solveInd])
        nCollide = rg.nCollisions
        if bestNCollide is None or nCollide < bestNCollide:
            bestNCollide = nCollide
            bestSolution = solveInds
            print("bestNCollide", bestNCollide)
        if bestNCollide == 0:
            print("early exit found solution")
            solver.EndSearch()
            break


        # rg.plotGrid()
        # plt.savefig("fig%i.png"%count)
        # if count == 20:
        #     break
        # break
    print("Number of solutions:", count)
    for robot,bestInd in zip(rg.robotList,bestSolution):
        robot.setAlphaBetaFromFocalXY(*targetCache[bestInd])
    rg.plotGrid()
    plt.show()
    print("best n Collide: ", bestNCollide)


