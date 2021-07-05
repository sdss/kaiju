import pytest

import kaiju
from kaiju.robotGrid import RobotGridAPO
from kaiju import utils


def test_collide(plot=False):
    # should make a grid
    rg = RobotGridAPO()
    rg.robotDict[61].setAlphaBeta(90,
                                  0)
    rg.robotDict[296].setAlphaBeta(270,
                                   0)
    assert rg.isCollided(61)
    assert rg.isCollided(296)

    if plot:
      utils.plotOne(0, rg, figname="test_collide.png", isSequence=False, xlim=[-30, 30], ylim=[-30, 30])

if __name__ == "__main__":
  test_collide(True)