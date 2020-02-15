import pytest
import matplotlib.pyplot as plt
import numpy
from kaiju import RobotGrid
from kaiju import utils

nDia = 15
angStep = 3
collisionBuffer = 2
epsilon = angStep * 2
hasApogee = True


def test_forwardPathGen(plot=False):
    xPos, yPos = utils.hexFromDia(15, pitch=22.4)
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
    if plot:
        utils.plotOne(0, rg, figname="forwardPathGenInitialEuc.png", isSequence=False)
    rg.decollideGrid()
    if plot:
        utils.plotOne(0, rg, figname="forwardPathDecollidedEuc.png", isSequence=False)
    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(robot.alpha, robot.beta)
        robot.setAlphaBeta(0, 180)
    assert rg.getNCollisions() == 0
    rg.pathGen2()
    if plot:
        utils.plotPaths(rg, filename="forwardPathGenEuc.mp4")

def test_reversePathGen(plot=False):
    xPos, yPos = utils.hexFromDia(15, pitch=22.4)
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
    if plot:
        utils.plotOne(0, rg, figname="reversePathGenInitialEuc.png", isSequence=False)
    rg.decollideGrid()
    if plot:
        utils.plotOne(0, rg, figname="reversePathDecollidedEuc.png", isSequence=False)
    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(0, 180)
    assert rg.getNCollisions() == 0
    rg.pathGen2()
    if plot:
        utils.plotOne(-1, rg, figname="reversePathEndEuc.png", isSequence=False)
        plt.figure()
        for robot in rg.robotDict.values():
            ap = numpy.array(robot.alphaPath)
            bp = numpy.array(robot.betaPath)
            plt.plot(ap[:,0], ap[:,1], color="orange", alpha=0.3)
            plt.plot(bp[:,0], bp[:,1], color="blue", alpha=0.3)
        plt.savefig("alphabetapathEuc.png")
        plt.close()

    if plot:
        utils.plotPaths(rg, filename="reversePathGenEuc.mp4")


if __name__ == "__main__":
    test_reversePathGen(plot=True)
    test_forwardPathGen(plot=True)


