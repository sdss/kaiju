from multiprocessing import Pool, cpu_count
from functools import partial
import itertools

from runSim import doOne, compileResults

if __name__ == "__main__":

    saveDir = "/home/csayres/kaijuRunVarStep"

    nProcs = 10

    # nTrials = 15
    seeds = range(0, 500)
    nDia = [27]
    cbuff = [1.5, 2, 2.5, 3, 3.5]
    angStep = [0.01, 0.05, 0.1, .25, 0.5, 0.75, 1]
    greedPhob = [(0.9, 0.3),(1, 0)]
    doOnePartial = partial(doOne, saveDir=saveDir)
    gridIter = itertools.product(seeds,nDia,angStep,cbuff,greedPhob)
    p = Pool(nProcs)
    p.map(doOnePartial, gridIter)
    p.close()
    compileResults(saveDir, "allSeedsVarStep.csv")


