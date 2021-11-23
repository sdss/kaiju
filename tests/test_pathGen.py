import pytest
import numpy
import time

from kaiju.robotGrid import RobotGrid, RobotGridNominal
from kaiju import utils

# nDia = 15
# angStep = 1
# collisionBuffer = 1.8 #2
# epsilon = angStep * 2
hasApogee = True


def test_hexDeadlockedPath(plot=False):
    xPos, yPos = utils.hexFromDia(35, pitch=22.4)
    seed = 3
    cb = 2.5
    angStep = 1
    downsample = int(numpy.floor(3 / angStep))
    rg = RobotGrid(angStep, seed=seed)
    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, str(robotID), [x,y,0], hasApogee)
        rg.robotDict[robotID].setDestinationAlphaBeta(0, 180)
    rg.setCollisionBuffer(cb)
    rg.initGrid()
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
    assert rg.getNCollisions() > 10
    rg.decollideGrid()
    assert rg.getNCollisions() == 0
    rg.pathGenGreedy()
    assert rg.didFail
    if plot:
        utils.plotPaths(rg, downsample=downsample, filename="test_hexDeadlockedPath.mp4")
    # print("rg did fail", rg.didFail)


def test_pathGen(plot=False):
    xPos, yPos = utils.hexFromDia(15, pitch=22.4)
    print("using", len(xPos), "robots")
    seed = 2
    smoothPts = 5
    collisionShrink = 0.03
    angStep = 0.1
    collisionBuffer = 2
    # epsilon = angStep * 2
    downsample = int(numpy.floor(3 / angStep))
    rg = RobotGrid(angStep, seed=seed)
    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, str(robotID), [x, y, 0], hasApogee)
        rg.robotDict[robotID].setDestinationAlphaBeta(0, 180)
    rg.setCollisionBuffer(collisionBuffer)
    rg.initGrid()
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
    assert rg.getNCollisions() > 10
    if plot:
        utils.plotOne(0, rg, figname="pathGenInitial.png", isSequence=False)
    rg.decollideGrid()
    assert rg.getNCollisions() == 0
    if plot:
        utils.plotOne(0, rg, figname="pathGenDecollided.png", isSequence=False)
    rg.pathGenGreedy()
    # print("rg alg type", rg.algType, type(rg.algType), str(rg.algType))
    # sd = rg.robotGridSummaryDict()
    # for d in sd["robotDict"].values():
    #     print(d["id"], d["alpha"], d["beta"])
    # print("deadlocks", rg.deadlockedRobots())
    assert not rg.didFail
    rg.smoothPaths(smoothPts)
    rg.simplifyPaths()
    rg.verifySmoothed()
    assert rg.smoothCollisions > 100
    print(rg.smoothCollisions)
    rg.shrinkCollisionBuffer(collisionShrink)
    rg.verifySmoothed()
    # assert rg.smoothCollisions == 0
    print(rg.smoothCollisions)
    if plot:
        utils.plotOne(0, rg, figname="donepaht.png", isSequence=False)
        utils.plotPaths(rg, downsample=downsample, filename="pathGen.mp4")



def test_withDefulatArgs(plot=False):
    rg = RobotGridNominal() # this is the test, that no args still works

    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
        robot.setDestinationAlphaBeta(0, 180)
    assert rg.getNCollisions() > 10
    rg.decollideGrid()
    assert rg.getNCollisions() == 0
    rg.pathGenGreedy()
    if plot:
        utils.plotPaths(rg, filename="test_default.mp4")


if __name__ == "__main__":
    # pytest won't run these, run by hand for the plot output
    # test_hexDeadlockedPath(plot=True)
    # test_pathGen(plot=True)
    # test_filledHexDeadlockedPath(plot=True)
    # test_filledHexPath(plot=True)
    # test_pathGen(plot=True)
    test_pathGen(plot=False)


