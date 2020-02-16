import pytest
import matplotlib.pyplot as plt
import numpy
from kaiju import RobotGrid
from kaiju import utils

nDia = 15
angStep = 3
collisionBuffer = 2.25
epsilon = angStep * 2
hasApogee = True


def test_forwardGreedy(plot=False):
    xPos, yPos = utils.hexFromDia(35, pitch=22.4)
    seed = 1
    rg = RobotGrid(
        stepSize=angStep, collisionBuffer=collisionBuffer,
        epsilon=epsilon, seed=seed
    )

    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
    rg.initGrid()
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
    assert rg.getNCollisions() > 10

    rg.decollideGrid()

    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(robot.alpha, robot.beta)
        robot.setAlphaBeta(0, 180)
    assert rg.getNCollisions() == 0
    rg.pathGenGreedy()
    if plot:
        utils.plotPaths(rg, filename="forwardGreedy.mp4")

def test_reverseGreedy(plot=False):
    xPos, yPos = utils.hexFromDia(35, pitch=22.4)
    seed = 1
    rg = RobotGrid(
        stepSize=angStep, collisionBuffer=collisionBuffer,
        epsilon=epsilon, seed=seed
    )

    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
    rg.initGrid()
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
    assert rg.getNCollisions() > 10

    rg.decollideGrid()

    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(0, 180)
    assert rg.getNCollisions() == 0
    rg.pathGenGreedy()
    if plot:
        utils.plotPaths(rg, filename="reverseGreedy.mp4")

def test_forwardMDP(plot=False):
    xPos, yPos = utils.hexFromDia(35, pitch=22.4)
    seed = 1
    rg = RobotGrid(
        stepSize=angStep, collisionBuffer=collisionBuffer,
        epsilon=epsilon, seed=seed
    )

    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
    rg.initGrid()
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
    assert rg.getNCollisions() > 10

    rg.decollideGrid()

    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(robot.alpha, robot.beta)
        robot.setAlphaBeta(0, 180)
    assert rg.getNCollisions() == 0
    rg.pathGenMDP()
    if plot:
        utils.plotPaths(rg, filename="forwardMDP.mp4")

def test_reverseMDP(plot=False):
    xPos, yPos = utils.hexFromDia(35, pitch=22.4)
    seed = 1
    rg = RobotGrid(
        stepSize=angStep, collisionBuffer=collisionBuffer,
        epsilon=epsilon, seed=seed
    )

    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
    rg.initGrid()
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
    assert rg.getNCollisions() > 10

    rg.decollideGrid()

    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(0, 180)
    assert rg.getNCollisions() == 0
    rg.pathGenMDP()
    if plot:
        utils.plotPaths(rg, filename="reverseMDP.mp4")

def test_setMDP(plot=False):

    xPos, yPos = utils.hexFromDia(45, pitch=22.4)
    print("using ", len(xPos), "robots")
    for seed in range(100):
        rg = RobotGrid(
            stepSize=angStep, collisionBuffer=collisionBuffer,
            epsilon=epsilon, seed=seed
        )

        for robotID, (x, y) in enumerate(zip(xPos, yPos)):
            rg.addRobot(robotID, x, y, hasApogee)
        rg.initGrid()
        for rID in rg.robotDict:
            robot = rg.getRobot(rID)
            robot.setXYUniform()
        assert rg.getNCollisions() > 10

        rg.decollideGrid()

        for robot in rg.robotDict.values():
            robot.setTargetAlphaBeta(20, 170)
        assert rg.getNCollisions() == 0
        rg.pathGenMDP()

        deadlockedRobots = []
        for r in rg.robotDict.values():
            if not r.onTargetVec[-1]:
                deadlockedRobots.append(r.id)
        if len(deadlockedRobots):
            print("seed", seed, "failed with these", deadlockedRobots, "in ", rg.nSteps)
            break
        else:
            print("seed", seed, "didn't fail", rg.nSteps, " taken to solve")

    if plot:
        utils.plotPaths(rg, filename="reverseSetMDP.mp4")

if __name__ == "__main__":
    # test_forwardGreedy(plot=True)
    # test_reverseGreedy(plot=True)
    # test_forwardMDP(plot=True)
    # test_reverseMDP(plot=True)

    test_setMDP(plot=True)
    # test_forwardPathGen(plot=True)
    # test_reversePathGen(plot=True)
    # test_reversePathGen2(plot=True)
    # test_reversePathGen3(plot=True)
    # test_forwardPathGen2(plot=True)


