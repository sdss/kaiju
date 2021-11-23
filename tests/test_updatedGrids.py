import pytest
import numpy
import seaborn as sns
import matplotlib.pyplot as plt

from kaiju.robotGrid import RobotGridNominal
from kaiju import utils


def test_me(plot=False):
    angStep = 0.1
    downsample = int(numpy.floor(10 / angStep))
    seed = 4
    rg = RobotGridNominal(stepSize=angStep, seed=seed)
    rg.setCollisionBuffer(2)
    for r in rg.robotDict.values():
        r.setXYUniform()
        r.setDestinationAlphaBeta(0, 180)
    rg.decollideGrid()
    rg.pathGenMDP(0.8, 0.2)
    deadlockedRobots = []
    for r in rg.robotDict.values():
        # if not r.onTargetVec[-1]:
        if r.score() > 0:
            deadlockedRobots.append(r.id)

    if plot:
        utils.plotPaths(rg, downsample=downsample, filename="update.mp4")

        xs = []
        ys = []
        types = []
        for r in rg.robotDict.values():
            xs.append(r.basePos[0])
            ys.append(r.basePos[1])
            if r.hasApogee:
                types.append("BA")
            else:
                types.append("BOSS")

        for f in rg.fiducialDict.values():
            xs.append(f.xyzWok[0])
            ys.append(f.xyzWok[1])
            types.append("Fiducial")

        sns.scatterplot(x=xs, y=ys, hue=types)
        plt.savefig("scatterAPO.png")

    if len(deadlockedRobots):
        print("seed", seed, "failed with these", deadlockedRobots, "in ", rg.nSteps)
        assert False



if __name__ == "__main__":
    test_me(plot=True)
