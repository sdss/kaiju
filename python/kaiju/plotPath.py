import numpy
import matplotlib.pyplot as plt
import glob
import multiprocessing

def plotOne(filename):
    fig = plt.figure(figsize=(9, 9))
    alphaBeta = numpy.loadtxt(filename)
    tsteps = range(alphaBeta.shape[0])

    plt.plot(tsteps, alphaBeta[:,0])
    plt.plot(tsteps, alphaBeta[:,1])
    plt.ylim([-10, 370])
    fname = filename.strip(".txt") + ".png"
    plt.savefig(fname, dpi=500)
    plt.close()

def plotSet():
    filenames = glob.glob("path_*.txt")
    p = multiprocessing.Pool(10)
    p.map(plotOne, filenames)

filenames = glob.glob("path_*.txt")
for filename in filenames:
    plotOne(filename)