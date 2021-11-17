import pytest
import coordio

from kaiju.robotGrid import RobotGrid, RobotGridNominal
from kaiju import utils

nDia = 15
angStep = 1
collisionBuffer = 2
epsilon = angStep * 2
seed = 0
hasApogee = True


def test_hexGrid():
    xPos, yPos = utils.hexFromDia(15, pitch = 22.4)
    rg = RobotGrid(angStep, epsilon, seed)
    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        basePos = [x, y, 0]
        rg.addRobot(
            robotID, str(robotID), basePos, hasApogee,
            collisionBuffer=collisionBuffer
        )
    rg.initGrid()


def test_doubleRobotID():
    # should raise runtime error when two robots have same id
    rg = RobotGrid(angStep, epsilon, seed)
    robotID = 1
    rg.addRobot(
        robotID, str(robotID), [0, 0, 0], hasApogee,
        collisionBuffer=collisionBuffer
    )
    with pytest.raises(RuntimeError) as excinfo:
        rg.addRobot(robotID, str(robotID), [30, 0, 0], hasApogee)
    assert "Robot ID already exists" in str(excinfo.value)

def test_filledHexGrid():
    apo = RobotGridNominal(angStep)

def test_doubleFiducialID():
    # should raise runtime error when two fiducials have same id
    rg = RobotGrid(angStep, epsilon, seed)
    fiducialID = 1
    rg.addFiducial(fiducialID, [0,0,coordio.defaults.POSITIONER_HEIGHT])
    with pytest.raises(RuntimeError) as excinfo:
        rg.addFiducial(fiducialID, [100,0,coordio.defaults.POSITIONER_HEIGHT])
    assert "Fiducial ID already exists" in str(excinfo.value)

if __name__ == "__main__":
    test_doubleRobotID()