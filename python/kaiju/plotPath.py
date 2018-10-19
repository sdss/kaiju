import numpy
import matplotlib.pyplot as plt
import glob
import multiprocessing

def plotOne(filename, isSmooth=False, isAlpha=True, interp=False):
    # fig = plt.figure(figsize=(9, 9))
    path = numpy.loadtxt(filename)
    label = "alpha"
    linestyle = "-"
    linewidth = 0.5
    alpha = 1
    if not isAlpha:
        label = "beta"
    if interp:
        label+=" interp"
        linestyle = '-'
        alpha = 1
        linewidth = 0.25
    elif isSmooth:
        label+=" smooth"
        linestyle = '.-'
        alpha = 0.2
        linewidth = 1

    plt.plot(path[:,0], path[:,1], linestyle, label=label, alpha=alpha, linewidth=linewidth)
    plt.ylim([-10, 370])
    plt.ylabel("position (deg)")
    plt.xlabel("step")
    plt.legend()
    # fname = filename.strip(".txt") + ".png"
    robotNum = int(filename.split("_")[-1].strip(".txt"))
    # plt.title("Robot %i path"%robotNum)
    # plt.savefig(fname, dpi=500)
    # plt.close()

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
pathNums = [204, 244, 247, 252, 262, 370]

for filenum in pathNums:
    fig = plt.figure(figsize=(9, 9))
    alphaFile = "pathAlpha_%04i.txt"%filenum
    betaFile = "pathBeta_%04i.txt"%filenum
    smoothAlphaFile = "smoothpathAlpha_%04i.txt"%filenum
    smoothBetaFile = "smoothpathBeta_%04i.txt"%filenum
    ismoothAlphaFile = "ismoothpathAlpha_%04i.txt"%filenum
    ismoothBetaFile = "ismoothpathBeta_%04i.txt"%filenum
    plotOne(alphaFile)
    plotOne(betaFile, isAlpha=False)

    plotOne(smoothAlphaFile, isSmooth=True)
    plotOne(smoothBetaFile, isSmooth=True, isAlpha=False)

    plotOne(ismoothAlphaFile, isSmooth=True, interp=True)
    plotOne(ismoothBetaFile, isSmooth=True, isAlpha=False, interp=True)
    fname = "path_%04i.png"%filenum
    plt.title("Robot %i path"%filenum)
    plt.savefig(fname, dpi=1000)
    plt.close()

    # ffmpeg -r 10 -f image2 -i interp_%04d.png -pix_fmt yuv420p -vcodec libx264 robotMovie.mp4