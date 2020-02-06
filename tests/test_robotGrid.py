import pytest

import kaiju
import kaiju.robotGrid


def test_filledHex():
    # should make a grid
    rg = kaiju.robotGrid.RobotGridFilledHex()


def test_robot_array():
    # should make an array from robots and convert back the same
    rg = kaiju.robotGrid.RobotGridFilledHex()
    rarray = rg.robot_array()
    rg2 = kaiju.robotGrid.RobotGridFilledHex()
    rg2.robot_fromarray(rarray)


def test_add_targets():
    # should add targets OK
    rg = kaiju.robotGrid.RobotGridFilledHex()
    rg.addTarget(targetID=1, x=10., y=10., priority=1.,
                 fiberType=kaiju.ApogeeFiber)
    rg.addTarget(targetID=2, x=30., y=30., priority=1.,
                 fiberType=kaiju.BossFiber)


def test_target_array():
    # should add targets OK
    rg = kaiju.robotGrid.RobotGridFilledHex()
    rg.addTarget(targetID=1, x=10., y=10., priority=1.,
                 fiberType=kaiju.ApogeeFiber)
    rg.addTarget(targetID=2, x=30., y=30., priority=1.,
                 fiberType=kaiju.BossFiber)
    ts = rg.target_array()
    rg2 = kaiju.robotGrid.RobotGridFilledHex()
    rg2.target_fromarray(ts)
    for id in [1, 2]:
        assert rg2.targetDict[id].x == rg.targetDict[id].x
        assert rg2.targetDict[id].y == rg.targetDict[id].y
        assert rg2.targetDict[id].priority == rg.targetDict[id].priority
        assert rg2.targetDict[id].fiberType == rg.targetDict[id].fiberType
        assert rg2.targetDict[id].id == rg.targetDict[id].id
