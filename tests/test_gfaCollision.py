from kaiju import RobotGridCalib
from kaiju.utils import plotOne
import numpy


def test_gfaCollision(plot=False):
    rg = RobotGridCalib()

    for alphaAng in numpy.linspace(0, 360, 100): #[0, 60, 120, 180, 240, 300, 360]:
        for r in rg.robotDict.values():
            r.setAlphaBeta(alphaAng, 0)
        # assert rg.getNCollisions() == 6
        if plot:
            plotOne(0, rg, "gfa%i_collide.png"%alphaAng, False)
        rg.decollideGrid()
        assert rg.getNCollisions() == 0
        if plot:
            plotOne(0, rg, "gfa%i_decollide.png"%alphaAng, False)


if __name__ == "__main__":
    test_gfaCollision(plot=True)