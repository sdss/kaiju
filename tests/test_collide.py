import pytest

import kaiju
from kaiju.robotGrid import RobotGridAPO
from kaiju import utils


def test_collide(plot=False):
    # should make a grid
    rg = RobotGridAPO()
    rg.robotDict[61].setAlphaBeta(53.533632923800,
                                  157.006436516039)
    rg.robotDict[296].setAlphaBeta(318.465535308471,
                                   72.060067856199)
    assert rg.isCollided(61) == rg.isCollided(296)

    if plot:
      utils.plotOne(0, rg, figname="test_collide.png", isSequence=False, xlim=[-30, 30], ylim=[-30, 30])

if __name__ == "__main__":
  test_collide(True)