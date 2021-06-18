from multiprocessing import Pool, cpu_count
from functools import partial
import itertools
import numpy
from runSim import doOne, compileResults
import pandas as pd
import os
import numpy
from kaiju import utils

nDias = numpy.arange(7,67,4)
roboMap = {}
for nDia in nDias:
    xPos, yPos = xPos, yPos = utils.hexFromDia(nDia, pitch=22.4, rotAngle=90)
    roboMap[len(xPos)] = nDia

if __name__ == "__main__":


    saveDir = "/home/csayres/kaijuRunVarNum"

    nProcs = 18

    seeds = range(0, 200)
    cbuff = [2.5]
    angStep = [0.1]
    greedPhob = [(0.9, 0.3),(1, 0)]
    # nDias = numpy.arange(7,67,4)

    doOnePartial = partial(doOne, saveDir=saveDir)

    gridIter = itertools.product(seeds,nDias,angStep,cbuff,greedPhob)
    p = Pool(nProcs)
    p.map(doOnePartial, gridIter)
    p.close()
    compileResults(saveDir, "allSeedsVarNum.csv")

    for ii in range(10):
        numpy.random.seed(ii)
        df = pd.read_csv(os.path.join(saveDir, "allSeedsVarNum.csv"))
        dfz = df[df["efficiency"]==0]
        if len(dfz) == 0:
            break
        repeats = []
        for ind, row in dfz.iterrows():
            seed = row["seed"]
            nDia = nDia = roboMap[row["nRobots"]]
            angStep = row["angStep"]
            cbuff = row["collisionBuffer"]
            greed = row["greed"]
            phobia = row["phobia"]
            repeats.append([seed, nDia, angStep, cbuff, [greed, phobia]])
        fname = os.path.join(saveDir, "repeats_%i.txt"%ii)
        with open(fname, "r") as f:
            for repeat in repeats:
                f.write("%s\n"%str(repeat))
        p = Pool(nProcs)
        p.map(doOnePartial, repeats)
        p.close()
        compileResults(saveDir, "allSeedsVarNum.csv")


