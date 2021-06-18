import pytest

from kaiju.cKaiju import RobotGrid
from kaiju import utils

nDia = 15
angStep = 1
collisionBuffer = 2
epsilon = angStep * 2
seed = 0
hasApogee = True

def test_hexGrid():
    xPos, yPos = utils.hexFromDia(15, pitch = 22.4)
    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)
    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, x, y, hasApogee)
    rg.initGrid()


def test_doubleRobotID():
    # should raise runtime error when two robots have same id
    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)
    robotID = 1
    rg.addRobot(robotID, 0, 0, hasApogee)
    with pytest.raises(RuntimeError) as excinfo:
        rg.addRobot(robotID, 30, 0, hasApogee)
    assert "Robot ID already exists" in str(excinfo.value)

def test_filledHexGrid():
    rg = utils.robotGridFromFilledHex(angStep, collisionBuffer)

def test_doubleFiducialID():
    # should raise runtime error when two fiducials have same id
    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)
    fiducialID = 1
    rg.addFiducial(fiducialID, 0, 0)
    with pytest.raises(RuntimeError) as excinfo:
        rg.addFiducial(fiducialID, 100, 0)
    assert "Fiducial ID already exists" in str(excinfo.value)
