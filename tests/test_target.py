import pytest

import numpy
import matplotlib.pyplot as plt

from kaiju.cKaiju import RobotGrid, BossFiber, ApogeeFiber, MetrologyFiber
from kaiju import utils


def test_nonInit():
    rg = RobotGrid()
    rg.addRobot(robotID=1, xPos=0, yPos=0)
    rg.addFiducial(fiducialID=1, x=22.4, y=0)
    with pytest.raises(RuntimeError) as excinfo:
        rg.addTarget(targetID=1, x=0, y=10, fiberType=BossFiber, priority=1)
    assert "Initialize RobotGrid before adding targets" in str(excinfo.value)


def test_doubleTargetID():
    rg = RobotGrid()
    rg.addRobot(robotID=1, xPos=0, yPos=0)
    rg.addFiducial(fiducialID=1, x=22.4, y=0)
    rg.initGrid()
    rg.addTarget(targetID=1, x=0, y=10, fiberType=BossFiber, priority=1)
    with pytest.raises(RuntimeError) as excinfo:
        rg.addTarget(targetID=1, x=10, y=10, fiberType=BossFiber, priority=1)
    assert "Target ID already exists" in str(excinfo.value)


def test_addTarget():
    rg = RobotGrid()
    rg.addRobot(robotID=1, xPos=0, yPos=0)
    rg.addFiducial(fiducialID=1, x=22.4, y=0)
    rg.initGrid()
    rg.addTarget(targetID=1, x=0, y=10, fiberType=BossFiber, priority=1)
    rg.addTarget(2, 10, 10, ApogeeFiber)
    rg.addTarget(3, -20, -40, MetrologyFiber)
    assert True

def generateTargs(nTargs, fiberType):
    rg = utils.robotGridFromFilledHex()
    # put in range -300 300
    randomX = numpy.random.random_sample(nTargs)*650 - 325
    randomY = numpy.random.random_sample(nTargs)*560 - 280
    for tid, (x, y) in enumerate(zip(randomX, randomY)):
        rg.addTarget(tid, x, y, fiberType)
    return rg

def plotTargs(rg, figname):
    unreachableTargets = rg.unreachableTargets()
    xyGood = []
    xyBad = []
    for tid, target in rg.targetDict.items():
        if tid in unreachableTargets:
            xyBad.append([target.x, target.y])
        else:
            xyGood.append([target.x, target.y])
    xyGood = numpy.array(xyGood)
    xyBad = numpy.array(xyBad)
    plt.figure(figsize=(8,8))
    plt.plot(xyGood[:,0], xyGood[:,1], 'og')
    plt.plot(xyBad[:,0], xyBad[:,1], 'oc')
    # overplot fiducials
    for fid, fiducial in rg.fiducialDict.items():
        plt.plot(fiducial.x, fiducial.y, 'kx')
    # plt.xlim([-300, 300])
    # plt.ylim([-300, 300])
    ax = plt.gca()
    ax.set_aspect("equal")
    plt.savefig(figname, dpi=250)


def test_tonsOBosstargs(plot=False):
    nTargs = 100000
    rg = generateTargs(nTargs, BossFiber)
    unreachableTargets = rg.unreachableTargets()
    unFrac = len(unreachableTargets)/nTargs
    assert numpy.abs(unFrac - 0.29457) < 0.01
    if plot:
        plotTargs(rg, "tonOBoss.png")


def test_tonsOAptargs(plot=False):
    nTargs = 100000
    rg = generateTargs(nTargs, ApogeeFiber)
    unreachableTargets = rg.unreachableTargets()
    unFrac = len(unreachableTargets)/nTargs
    assert numpy.abs(unFrac - 0.30379) < 0.01
    if plot:
        plotTargs(rg, "tonOAp.png")


def test_validRobots(plot=False):
    nTargs = 50000
    rg = generateTargs(nTargs, BossFiber)
    assert len(rg.targetlessRobots()) == 0
    for rid, robot in rg.robotDict.items():
        assert len(robot.validTargetIDs) > 100
    if plot:
        # show a set of targets for a subset of robots
        rIDs = list(rg.robotDict.keys())
        for ii in [20, 120, 450]:
            robot = rg.robotDict[rIDs[ii]]
            targetXYs = []
            for tid in robot.validTargetIDs:
                target = rg.targetDict[tid]
                targetXYs.append([target.x, target.y])
            targetXYs = numpy.array(targetXYs)
            plt.plot(targetXYs[:,0], targetXYs[:,1], 'og', markersize=1, alpha=0.5)
        ax = plt.gca()
        plt.xlim([-350, 350])
        ax.set_aspect("equal")
        plt.savefig("subsetTarg.png", dpi=250)

def test_targetAssign(plot=False):
    rg = utils.robotGridFromFilledHex()
    for robot in rg.robotDict.values():
        robot.setXYUniform()
    rg.decollideGrid()
    if plot:
        utils.plotOne(0, rg, figname="beforeAssign.png", isSequence=False)#, highlightRobot=205)
    # find boss target positions for all robots
    for rID, robot in rg.robotDict.items():
        x,y,z = robot.bossFiberPos
        # give the target the same id as the robot
        rg.addTarget(rID, x, y, BossFiber)
        targ = rg.targetDict[rID]
        assert rID in targ.validRobotIDs
        assert rID in robot.validTargetIDs
        rg.assignRobot2Target(robotID=rID, targID=targ.id)

    unreachableTargets = rg.unreachableTargets()
    assert len(unreachableTargets) == 0
    targetlessRobots = rg.targetlessRobots()
    assert len(targetlessRobots) == 0

    # unassign target 100
    rg.unassignTarget(100)
    assert rg.robotDict[100].isAssigned() == False
    assert rg.targetDict[100].isAssigned() == False

    rg.unassignTarget(99)
    assert rg.robotDict[99].isAssigned() == False
    assert rg.targetDict[99].isAssigned() == False

    assignedTargets = rg.assignedTargets()
    assert (99 in assignedTargets) == False
    assert (100 in assignedTargets) == False

    if plot:
        utils.plotOne(0, rg, figname="afterAssign.png", isSequence=False)

"""
robot alpha beta (138.53, 1.66) xpos/ypos (-33.6 -252.1866) number: 205 valid targs []
alphabeta [138.5303302264647, 1.6620349800302785] boss fiber not in valid list?
i think issue is it is at nearly full extension?

test targets at full extension?
"""
if __name__ == "__main__":
    # test_tonsOBosstargs(plot=True)
    # test_tonsOAptargs(plot=True)
    test_targetAssign(plot=True)
