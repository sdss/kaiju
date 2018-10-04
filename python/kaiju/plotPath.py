import numpy
import matplotlib.pyplot as plt
import glob
import multiprocessing

def plotOne(filename):
    fig = plt.figure(figsize=(9, 9))
    alphaBeta = numpy.loadtxt(filename)
    tsteps = range(alphaBeta.shape[0])

    plt.plot(tsteps, alphaBeta[:,0], label="alpha")
    plt.plot(tsteps, alphaBeta[:,1], label="beta")
    plt.ylim([-10, 370])
    plt.ylabel("position (deg)")
    plt.xlabel("step")
    plt.legend()
    fname = filename.strip(".txt") + ".png"
    robotNum = int(filename.split("_")[-1].strip(".txt"))
    plt.title("Robot %i path"%robotNum)
    plt.savefig(fname, dpi=500)
    plt.close()

def plotVelocity(filename):
    fig = plt.figure(figsize=(9, 9))
    alphaBeta_ = numpy.loadtxt(filename)
    alphaBeta = numpy.diff(alphaBeta_, axis=0)
    tsteps = range(alphaBeta.shape[0])
    plt.plot(tsteps, alphaBeta[:,0], label="alpha")
    plt.plot(tsteps, alphaBeta[:,1], label="beta")
    # plt.ylim([-10, 370])
    plt.ylabel("speed (deg)")
    plt.xlabel("step")
    plt.legend()
    fname = filename.strip(".txt") + ".png"
    robotNum = int(filename.split("_")[-1].strip(".txt"))
    plt.title("Robot %i path"%robotNum)
    plt.savefig(fname, dpi=500)
    plt.close()

def plotSet():
    filenames = glob.glob("path_*.txt")
    p = multiprocessing.Pool(10)
    p.map(plotOne, filenames)

#filenames = glob.glob("path_*.txt")
filenames = ["path_%04i.txt"%x for x in [204, 244, 247, 252, 262, 370]]

for filename in filenames:
    plotOne(filename)