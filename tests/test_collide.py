import pytest

import kaiju
from kaiju.robotGrid import RobotGridNominal
from kaiju import utils


def test_collide(plot=False):
    # should make a grid
    rg = RobotGridNominal()
    collidedRobotIDs = []
    for rid, r in rg.robotDict.items():
      if r.holeID == "R-13C1":
        r.setAlphaBeta(90,0)
        collidedRobotIDs.append(rid)
      elif r.holeID == "R-13C2":
        collidedRobotIDs.append(rid)
        r.setAlphaBeta(270, 0)
      else:
        r.setAlphaBeta(90,180)

    for rid in collidedRobotIDs:
      assert rg.isCollided(rid)

    assert rg.getNCollisions() == 2

    if plot:
      utils.plotOne(0, rg, figname="test_collide.png", isSequence=False, xlim=[-30, 30], ylim=[-30, 30])

if __name__ == "__main__":
  test_collide(True)