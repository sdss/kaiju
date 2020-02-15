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
        utils.plotOne(0, rg, figname="forwardPathGenInitialRot.png", isSequence=False)
    rg.decollideGrid()
    if plot:
        utils.plotOne(0, rg, figname="forwardPathDecollidedRot.png", isSequence=False)
    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(robot.alpha, robot.beta)
        robot.setAlphaBeta(0, 180)
    assert rg.getNCollisions() == 0
    rg.pathGen3()
    if plot:
        utils.plotPaths(rg, filename="forwardPathGenRot.mp4")

def test_reversePathGen(plot=False):
    xPos, yPos = utils.hexFromDia(45, pitch=22.4)
    print("n robots", len(xPos))
    seed = 1
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
        if plot:
            utils.plotOne(0, rg, figname="reversePathGenInitialRot.png", isSequence=False)
        rg.decollideGrid()
        if plot:
            utils.plotOne(0, rg, figname="reversePathDecollidedRot.png", isSequence=False)
        for robot in rg.robotDict.values():
            robot.setTargetAlphaBeta(30, 180)
        assert rg.getNCollisions() == 0
        rg.pathGen3()
        # rg.pathGen4()
        deadlockedRobots = []
        for r in rg.robotDict.values():
            if not r.onTargetVec[-1]:
                deadlockedRobots.append(r.id)
        if len(deadlockedRobots):
            print("seed", seed, "failed with these", deadlockedRobots)
            break
        else:
            print("seed", seed, "didn't fail", rg.nSteps, " taken to solve")
    if plot:
        utils.plotOne(-1, rg, figname="reversePathEndRot.png", isSequence=False)
        plt.figure()
        for robot in rg.robotDict.values():
            ap = numpy.array(robot.alphaPath)
            bp = numpy.array(robot.betaPath)
            plt.plot(ap[:,0], ap[:,1], color="orange", alpha=0.3)
            plt.plot(bp[:,0], bp[:,1], color="blue", alpha=0.3)
        plt.savefig("alphabetapathRot.png")
        plt.close()

    if plot:
        utils.plotPaths(rg, filename="reversePathGenRot.mp4")


def test_reversePathGen2(plot=False):
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
        utils.plotOne(0, rg, figname="reversePathGenInitialRot180.png", isSequence=False)
    rg.decollideGrid()
    if plot:
        utils.plotOne(0, rg, figname="reversePathDecollidedRot180.png", isSequence=False)
    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(180, 180)
    assert rg.getNCollisions() == 0
    rg.pathGen3()
    if plot:
        utils.plotOne(-1, rg, figname="reversePathEndRot180.png", isSequence=False)
        plt.figure()
        for robot in rg.robotDict.values():
            ap = numpy.array(robot.alphaPath)
            bp = numpy.array(robot.betaPath)
            plt.plot(ap[:,0], ap[:,1], color="orange", alpha=0.3)
            plt.plot(bp[:,0], bp[:,1], color="blue", alpha=0.3)
        plt.savefig("alphabetapathRot180.png")
        plt.close()

    if plot:
        utils.plotPaths(rg, filename="reversePathGenRot180.mp4")

def test_reversePathGen3(plot=False):
    xPos, yPos = utils.hexFromDia(45, pitch=22.4)
    seed = 1
    angStep = 2
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
    # assert rg.getNCollisions() > 10
    if plot:
        utils.plotOne(0, rg, figname="reversePathGenInitialRot180_90.png", isSequence=False)
    rg.decollideGrid()
    if plot:
        utils.plotOne(0, rg, figname="reversePathDecollidedRot180_90.png", isSequence=False)
    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(180, 90)
        # robot.setTargetAlphaBeta(0, 180)
    assert rg.getNCollisions() == 0
    rg.pathGen3()
    print("a;sdkf ", rg.nSteps, len(rg.robotDict[1].alphaPath))
    # if plot:
    #     f, ax = plt.subplots(2,1)
    #     for r in rg.robotDict.values():
    #         dAlpha = numpy.array(r.alphaPath)
    #         dAlpha[:,1] -= r.targetAlpha
    #         dBeta = numpy.array(r.betaPath)
    #         dBeta[:,1] -= r.targetBeta
    #         ax[0].plot(dAlpha[:,0], dAlpha[:,1])
    #         ax[1].plot(dBeta[:,0], dBeta[:,1])
    #     ax[0].set_ylim([-5, 5])
    #     ax[1].set_ylim([-5, 5])
    #     plt.savefig("investigate.png")
    #     plt.close()

    if plot:
        utils.plotOne(-1, rg, figname="reversePathEndRot180_90.png", isSequence=False)
        plt.figure()
        print("nsteps", rg.nSteps)
        for robot in rg.robotDict.values():
            ap = numpy.array(robot.alphaPath)
            bp = numpy.array(robot.betaPath)
            plt.plot(ap[:,0], ap[:,1], color="orange", alpha=0.3)
            plt.plot(bp[:,0], bp[:,1], color="blue", alpha=0.3)
        plt.savefig("alphabetapathRot180_90.png")
        plt.close()

    if plot:
        utils.plotPaths(rg, filename="reversePathGenRot180_90.mp4")


def test_forwardPathGen2(plot=False):
    xPos, yPos = utils.hexFromDia(25, pitch=22.4)
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
        utils.plotOne(0, rg, figname="forwardPathGenInitialRot180.png", isSequence=False)
    rg.decollideGrid()
    if plot:
        utils.plotOne(0, rg, figname="forwardPathDecollidedRot180.png", isSequence=False)
    for robot in rg.robotDict.values():
        robot.setTargetAlphaBeta(robot.alpha, robot.beta)
        robot.setAlphaBeta(180, 90)
    assert rg.getNCollisions() == 0
    rg.pathGen3()
    if plot:
        utils.plotPaths(rg, filename="forwardPathGenRot180.mp4")

if __name__ == "__main__":
    # test_forwardPathGen(plot=True)
    test_reversePathGen(plot=True)
    # test_reversePathGen2(plot=True)
    # test_reversePathGen3(plot=True)
    # test_forwardPathGen2(plot=True)


