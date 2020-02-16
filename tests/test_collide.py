import pytest

import kaiju
import kaiju.robotGrid


def test_collide():
    # should make a grid
    rg = kaiju.robotGrid.RobotGridFilledHex()
    rg.robotDict[61].setAlphaBeta(53.533632923800,
                                  157.006436516039)
    rg.robotDict[296].setAlphaBeta(318.465535308471,
                                   72.060067856199)
    assert rg.isCollided(61) == rg.isCollided(296)
