from multiprocessing import Pool, cpu_count
from functools import partial
import itertools

from runSim import doOne, compileResults

if __name__ == "__main__":

    saveDir = "/home/csayres/kaijuRun"

    nProcs = 24

    seeds = range(0, 2000)
    cbuff = [1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.25, 3.5]
    angStep = [0.1]
    greedPhob = [(0.9, 0.3),(1, 0)]
    nDia = [27]

    doOnePartial = partial(doOne, saveDir=saveDir)

    gridIter = itertools.product(seeds,nDia,angStep,cbuff,greedPhob)
    p = Pool(nProcs)
    p.map(doOnePartial, gridIter)
    p.close()
    compileResults(saveDir, "allSeeds.csv")


