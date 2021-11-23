import pytest
import numpy
from kaiju import RobotGridCalib
from kaiju import utils
import time
numpy.random.seed(0)


def test_explode(plot=False):

    rg = RobotGridCalib(seed=350)

    rg.setCollisionBuffer(2.5)
    for robot in rg.robotDict.values():
        robot.setXYUniform()
        robot.setDestinationAlphaBeta(0, 180)


    rg.decollideGrid()

    assert rg.getNCollisions() == 0
    tstart = time.time()
    rg.pathGenEscape(100)
    print("pathgen took", time.time()-tstart)

    if plot:
        utils.plotPaths(rg, filename="explode.mp4")


if __name__ == "__main__":
    test_explode(plot=True)
