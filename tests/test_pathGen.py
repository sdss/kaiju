import pytest

from kaiju.cKaiju import RobotGrid
from kaiju import utils

nDia = 15
angStep = 1
collisionBuffer = 2
epsilon = angStep * 2
hasApogee = True


def test_hexDeadlockedPath(plot=False):
    xPos, yPos = utils.hexFromDia(35, pitch = 22.4)
    seed = 3
    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)
    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
        rg.robotDict[robotID].setTargetAlphaBeta(0, 180)
    rg.initGrid()
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
    assert rg.getNCollisions() > 10
    rg.decollideGrid()
    assert rg.getNCollisions() == 0
    rg.pathGen()
    assert rg.didFail
    if plot:
        utils.plotPaths(rg, filename="test_hexDeadlockedPath.mp4")
    # print("rg did fail", rg.didFail)



def test_pathGen(plot=False):
    xPos, yPos = utils.hexFromDia(15, pitch=22.4)
    seed = 1
    smoothPts = 5
    collisionShrink = 0.3
    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)
    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
        rg.robotDict[robotID].setTargetAlphaBeta(0, 180)
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
    rg.pathGen()
    assert not rg.didFail
    rg.smoothPaths(smoothPts)
    rg.simplifyPaths()
    rg.verifySmoothed()
    assert rg.smoothCollisions > 100
    rg.setCollisionBuffer(collisionBuffer - collisionShrink)
    rg.verifySmoothed()
    assert rg.smoothCollisions == 0
    if plot:
        utils.plotPaths(rg, filename="pathGen.mp4")


def test_filledHexDeadlockedPath(plot=False):
    seed = 1
    rg = utils.robotGridFromFilledHex(angStep, collisionBuffer, seed)
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
        robot.setTargetAlphaBeta(0, 180)
    assert rg.getNCollisions() > 20
    if plot:
        utils.plotOne(0, rg, figname="filledHexDeadlockedInitial.png", isSequence=False)
    rg.decollideGrid()
    if plot:
        utils.plotOne(0, rg, figname="filledHexDeadlockedDecollided.png", isSequence=False)
    assert rg.getNCollisions() == 0
    rg.pathGen()
    if plot:
        utils.plotPaths(rg, filename="filledHexDeadlocked.mp4")
    assert rg.didFail

def test_filledHexPath(plot=False):
    seed = 3
    rg = utils.robotGridFromFilledHex(angStep, collisionBuffer, seed)
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
        robot.setTargetAlphaBeta(0, 180)
    assert rg.getNCollisions() > 20
    if plot:
        utils.plotOne(0, rg, figname="filledHexInitial.png", isSequence=False)
    rg.decollideGrid()
    if plot:
        utils.plotOne(0, rg, figname="filledHexDecollided.png", isSequence=False)
    assert rg.getNCollisions() == 0
    rg.pathGen()
    if plot:
        utils.plotPaths(rg, filename="filledHex.mp4")
    assert not rg.didFail

def test_withDefulatArgs(plot=False):
    xPos, yPos = utils.hexFromDia(35, pitch = 22.4)
    rg = RobotGrid() # this is the test, that no args still works
    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
    rg.initGrid()
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
        robot.setTargetAlphaBeta(0, 180)
    assert rg.getNCollisions() > 10
    rg.decollideGrid()
    assert rg.getNCollisions() == 0
    rg.pathGen()
    if plot:
        utils.plotPaths(rg, filename="test_default.mp4")


if __name__ == "__main__":
    # pytest won't run these, run by hand for the plot output
    # test_hexDeadlockedPath(plot=True)
    # test_pathGen(plot=True)
    # test_filledHexDeadlockedPath(plot=True)
    # test_filledHexPath(plot=True)
    test_pathGen(plot=True)


