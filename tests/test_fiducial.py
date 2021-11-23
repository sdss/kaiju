import pytest

from kaiju.robotGrid import RobotGrid
from kaiju import utils

import coordio


def test_uniqueFiducial():
    angStep = 1
    collisionBuffer = 2
    epsilon = angStep * 2
    hasApogee = True
    fiducialID = 1
    seed = 0
    rg = RobotGrid(angStep, epsilon, seed)
    rg.setCollisionBuffer(collisionBuffer)
    rg.addFiducial(fiducialID, [0, 0, 0])
    with pytest.raises(RuntimeError) as excinfo:
        rg.addFiducial(fiducialID, [30, 0, 0])
    assert "Fiducial ID already exists" in str(excinfo.value)

def test_fiducial(plot=False):
    angStep = 1
    collisionBuffer = 2
    fiducialCollisionBuffer = 1.5
    epsilon = angStep * 2
    hasApogee = True
    robotID = 1
    fiducialID = 10
    seed = 0
    rg = RobotGrid(angStep, epsilon, seed)
    rg.setCollisionBuffer(collisionBuffer)
    rg.addRobot(robotID, str(robotID), [0, 0, 0], hasApogee)
    rg.addFiducial(fiducialID, [22.4, 0, coordio.defaults.POSITIONER_HEIGHT], fiducialCollisionBuffer)
    rg.initGrid()
    robot = rg.getRobot(robotID)
    for betaAng in range(20):
        robot.setAlphaBeta(90,betaAng)
        rColliders = rg.robotColliders(robotID)
        fColliders = rg.fiducialColliders(robotID)
        if plot:
            utils.plotOne(0, rg, figname="fiducial_%i.png"%betaAng, isSequence=False, xlim=[-30, 30], ylim=[-30, 30])

        # assert len(rColliders) == 0
        # if betaAng < 14:
        #     assert fColliders == [fiducialID]
        # else:
        #     assert len(fColliders) == 0

def grow(plot=False):
    angStep = 1
    collisionBuffer = 2
    fiducialCollisionBuffer = 1.5
    epsilon = angStep * 2
    hasApogee = True
    robotID = 1
    fiducialID = 10
    seed = 0
    for cb in [1.5, 2, 2.5, 3]:
        rg = RobotGrid(angStep, epsilon, seed)
        rg.setCollisionBuffer(cb)
        rg.addRobot(robotID, str(robotID), [0, 0, 0], hasApogee)
        rg.addFiducial(fiducialID, [22.4, 0, coordio.defaults.POSITIONER_HEIGHT], fiducialCollisionBuffer)
        rg.initGrid()
        robot = rg.getRobot(robotID)
        robot.setAlphaBeta(90,0)
        if plot:
            utils.plotOne(0, rg, figname="grow_%.2f.png"%cb, isSequence=False, xlim=[-30, 30], ylim=[-30, 30])


if __name__ == "__main__":
    grow(plot=True)