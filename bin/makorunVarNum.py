from multiprocessing import Pool, cpu_count
from functools import partial
import itertools
import numpy
from runSim import doOne, compileResults

if __name__ == "__main__":


    saveDir = "/home/csayres/kaijuRunVarNum"

    nProcs = 10

    seeds = range(0, 200)
    cbuff = [2.5]
    angStep = [0.1]
    greedPhob = [(0.9, 0.3),(1, 0)]
    nDias = numpy.arange(7,67,4)

    doOnePartial = partial(doOne, saveDir=saveDir)

    gridIter = itertools.product(seeds,nDias,angStep,cbuff,greedPhob)
    p = Pool(nProcs)
    p.map(doOnePartial, gridIter)
    p.close()
    compileResults(saveDir, "allSeedsVarNum.csv")


