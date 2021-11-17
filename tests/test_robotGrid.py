import pytest

import kaiju
from kaiju.robotGrid import RobotGridNominal
import coordio

fpZ = coordio.defaults.POSITIONER_HEIGHT

def test_filledHex():
    # should make a grid
    rg = RobotGridNominal()


def test_robot_array():
    # should make an array from robots and convert back the same
    rg = RobotGridNominal()
    rarray = rg.robot_array()
    rg2 = RobotGridNominal()
    rg2.robot_fromarray(rarray)


def test_add_targets():
    # should add targets OK
    rg = RobotGridNominal()
    xyzWok1 = [10, 10, fpZ]
    xyzWok2 = [30, 30, fpZ]
    rg.addTarget(targetID=1, xyzWok=xyzWok1, priority=1.,
                 fiberType=kaiju.cKaiju.ApogeeFiber)
    rg.addTarget(targetID=2, xyzWok=xyzWok2, priority=1.,
                 fiberType=kaiju.cKaiju.BossFiber)


def test_target_array():
    # should add targets OK
    rg = RobotGridNominal()
    xyzWok1 = [10, 10, fpZ]
    xyzWok2 = [30, 30, fpZ]
    rg.addTarget(targetID=1, xyzWok=xyzWok1, priority=1.,
                 fiberType=kaiju.cKaiju.ApogeeFiber)
    rg.addTarget(targetID=2,xyzWok=xyzWok2, priority=1.,
                 fiberType=kaiju.cKaiju.BossFiber)
    ts = rg.target_array()
    rg2 = RobotGridNominal()
    rg2.target_fromarray(ts)
    for id in [1, 2]:
        assert rg2.targetDict[id].xWok == rg.targetDict[id].xWok
        assert rg2.targetDict[id].yWok == rg.targetDict[id].yWok
        assert rg2.targetDict[id].priority == rg.targetDict[id].priority
        assert rg2.targetDict[id].fiberType == rg.targetDict[id].fiberType
        assert rg2.targetDict[id].id == rg.targetDict[id].id
