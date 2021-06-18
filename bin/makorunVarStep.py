from multiprocessing import Pool, cpu_count
from functools import partial
import itertools
import pandas as pd
import numpy
import os

from runSim import doOne, compileResults

if __name__ == "__main__":

    saveDir = "/home/csayres/kaijuRunVarStep"

    nProcs = 18

    # nTrials = 15
    seeds = range(300, 500)
    nDia = [27]
    cbuff = [1.5, 2, 2.5, 3, 3.5]
    angStep = [0.05, 0.1, .25, 0.5, 0.75, 1]
    greedPhob = [(0.9, 0.3),(1, 0)]
    doOnePartial = partial(doOne, saveDir=saveDir)
    gridIter = itertools.product(seeds,nDia,angStep,cbuff,greedPhob)
    p = Pool(nProcs)
    p.map(doOnePartial, gridIter)
    p.close()
    compileResults(saveDir, "allSeedsVarStep.csv")


    for ii in range(5):
        numpy.random.seed(ii)
        df = pd.read_csv(os.path.join(saveDir, "allSeedsVarStep.csv"))
        dfz = df[df["efficiency"]==0]
        if len(dfz) == 0:
            break
        repeats = []
        for ind, row in dfz.iterrows():
            seed = row["seed"]
            if seed < 300:
                continue # we already did this one
            nDia = 27
            angStep = row["angStep"]
            cbuff = row["collisionBuffer"]
            greed = row["greed"]
            phobia = row["phobia"]
            repeats.append([seed, nDia, angStep, cbuff, [greed, phobia]])
        fname = os.path.join(saveDir, "repeats_%i.txt"%ii)
        with open(fname, "w") as f:
            for repeat in repeats:
                f.write("%s\n"%str(repeat))
        p = Pool(nProcs)
        p.map(doOnePartial, repeats)
        p.close()
        compileResults(saveDir, "allSeedsVarStep.csv")
