from multiprocessing import Pool, cpu_count
from functools import partial
import itertools
import runSim
runSim.maxReplacements = 20
from runSim import doOne, compileResults

zeroEff = [
[64,27,0.10,3.50,[0.90,0.30]],
[194,27,0.10,3.50,[0.90,0.30]],
[478,27,0.10,3.25,[0.90,0.30]],
[675,27,0.10,3.25,[0.90,0.30]],
[375,27,0.10,3.50,[0.90,0.30]],
[511,27,0.10,3.50,[0.90,0.30]],
[744,27,0.10,3.50,[0.90,0.30]],
[535,27,0.10,3.00,[0.90,0.30]],
[371,27,0.10,3.50,[0.90,0.30]],
[187,27,0.10,3.25,[0.90,0.30]],
[32,27,0.10,3.50,[0.90,0.30]],
[821,27,0.10,3.50,[0.90,0.30]],
[791,27,0.10,3.50,[0.90,0.30]],
[160,27,0.10,3.50,[0.90,0.30]],
[854,27,0.10,3.50,[0.90,0.30]],
[213,27,0.10,3.50,[0.90,0.30]],
[854,27,0.10,3.25,[0.90,0.30]],
[226,27,0.10,3.50,[0.90,0.30]],
[182,27,0.10,3.50,[0.90,0.30]],
[140,27,0.10,3.50,[0.90,0.30]],
[843,27,0.10,3.50,[0.90,0.30]],
[293,27,0.10,3.50,[0.90,0.30]],
[291,27,0.10,3.50,[0.90,0.30]],
[683,27,0.10,3.25,[0.90,0.30]],
[54,27,0.10,3.50,[0.90,0.30]],
[528,27,0.10,3.50,[0.90,0.30]],
[237,27,0.10,2.75,[0.90,0.30]],
[338,27,0.10,2.75,[0.90,0.30]],
[928,27,0.10,3.25,[0.90,0.30]],
[806,27,0.10,3.50,[0.90,0.30]],
[876,27,0.10,3.50,[0.90,0.30]],
[266,27,0.10,3.50,[0.90,0.30]],
[42,27,0.10,3.00,[0.90,0.30]],
[204,27,0.10,3.50,[0.90,0.30]],
[478,27,0.10,3.50,[0.90,0.30]],
[1044,27,0.10,3.25,[0.90,0.30]],
[151,27,0.10,3.25,[0.90,0.30]],
[970,27,0.10,2.50,[0.90,0.30]],
[970,27,0.10,3.00,[0.90,0.30]],
[458,27,0.10,2.75,[0.90,0.30]],
[206,27,0.10,3.50,[0.90,0.30]],
[799,27,0.10,3.25,[0.90,0.30]],
[865,27,0.10,3.25,[0.90,0.30]],
[314,27,0.10,3.50,[0.90,0.30]],
[150,27,0.10,2.75,[0.90,0.30]],
[616,27,0.10,3.50,[0.90,0.30]],
[22,27,0.10,3.25,[0.90,0.30]],
[567,27,0.10,3.50,[0.90,0.30]],
[8,27,0.10,3.50,[0.90,0.30]],
[816,27,0.10,3.25,[0.90,0.30]],
[902,27,0.10,3.00,[0.90,0.30]],
[755,27,0.10,3.50,[0.90,0.30]],
[374,27,0.10,3.00,[0.90,0.30]],
[229,27,0.10,3.50,[0.90,0.30]],
[885,27,0.10,3.50,[0.90,0.30]],
[526,27,0.10,3.25,[0.90,0.30]],
[656,27,0.10,3.00,[0.90,0.30]],
[546,27,0.10,2.50,[0.90,0.30]],
[547,27,0.10,3.50,[0.90,0.30]],
[1023,27,0.10,3.25,[0.90,0.30]],
[424,27,0.10,3.50,[0.90,0.30]],
[157,27,0.10,3.50,[0.90,0.30]],
[726,27,0.10,3.50,[0.90,0.30]],
[611,27,0.10,3.50,[0.90,0.30]],
[38,27,0.10,3.50,[1.00,0.00]],
[921,27,0.10,3.25,[0.90,0.30]],
[49,27,0.10,3.25,[0.90,0.30]],
[223,27,0.10,3.50,[0.90,0.30]],
[834,27,0.10,3.50,[0.90,0.30]],
[320,27,0.10,3.50,[0.90,0.30]],
[598,27,0.10,3.50,[0.90,0.30]],
[351,27,0.10,3.50,[0.90,0.30]]
]

if __name__ == "__main__":

    saveDir = "/home/csayres/kaijuRunZeroEff"
    ll = len(zeroEff)
    # nProcs = 24
    for ii, ze in enumerate(zeroEff):
        print("working on %i of %i "%(ii,ll), ze)
        doOne(ze, saveDir, plot=True)
    compileResults(saveDir, "allSeeds.csv")


