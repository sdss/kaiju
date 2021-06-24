import pytest
import numpy
import seaborn as sns
import matplotlib.pyplot as plt

from kaiju.robotGrid import RobotGridAPO, RobotGridLCO
from kaiju import utils


def test_APO(plot=False):
    angStep = 0.1
    downsample = int(numpy.floor(10 / angStep))
    seed = 4
    rg = RobotGridAPO(stepSize=angStep, collisionBuffer=2, seed=seed)
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
    if len(deadlockedRobots):
        print("seed", seed, "failed with these", deadlockedRobots, "in ", rg.nSteps)
        assert False

    if plot:
        utils.plotPaths(rg, downsample=downsample, filename="apoUpdate.mp4")

        xs = []
        ys = []
        types = []
        for r in rg.robotDict.values():
            xs.append(r.xPos)
            ys.append(r.yPos)
            if r.hasApogee:
                types.append("BA")
            else:
                types.append("BOSS")

        for f in rg.fiducialDict.values():
            xs.append(f.x)
            ys.append(f.y)
            types.append("Fiducial")

        sns.scatterplot(x=xs, y=ys, hue=types)
        plt.savefig("scatterAPO.png")


def test_LCO(plot=False):
    angStep = 0.1
    downsample = int(numpy.floor(10 / angStep))
    seed = 4
    rg = RobotGridLCO(stepSize=angStep, collisionBuffer=2, seed=seed)
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
    if len(deadlockedRobots):
        print("seed", seed, "failed with these", deadlockedRobots, "in ", rg.nSteps)
        assert False

    if plot:
        utils.plotPaths(rg, downsample=downsample, filename="lcoUpdate.mp4")

        xs = []
        ys = []
        types = []
        for r in rg.robotDict.values():
            xs.append(r.xPos)
            ys.append(r.yPos)
            if r.hasApogee:
                types.append("BA")
            else:
                types.append("BOSS")

        for f in rg.fiducialDict.values():
            xs.append(f.x)
            ys.append(f.y)
            types.append("Fiducial")

        sns.scatterplot(x=xs, y=ys, hue=types)
        plt.savefig("scatterLCO.png")


if __name__ == "__main__":
    test_LCO(plot=True)
