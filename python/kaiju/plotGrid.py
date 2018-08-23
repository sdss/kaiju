
import copy
from multiprocessing import Pool
import math
import sys
from functools import partial
import shutil
import os

import numpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from descartes import PolygonPatch
from networkx import DiGraph, shortest_path, draw, find_cycle, simple_cycles, shortest_simple_paths, has_path

AlphaArmLength = 7.4  # mm
AlphaRange = [0, 360]
BetaRange = [0, 180]
BetaArmLength = 15  # mm, distance to fiber
BetaArmWidth = 5  # mm
MinTargSeparation = 8  # mm
# length mm along beta for which a collision cant happen
BetaTopCollide = [8.187, 16]  # box from length 8.187mm to 16mm
BetaBottomCollide = [0, 10.689]  # box from 0 to 10.689
BetaArmLengthSafe = BetaArmLength / 2.0
Pitch = 22.4  # mm fudge the 01 for neighbors
MinReach = BetaArmLength - AlphaArmLength
MaxReach = BetaArmLength + AlphaArmLength

# https://internal.sdss.org/trac/as4/wiki/FPSLayout


class Robot(object):
    def __init__(self, id, xPos, yPos, alpha, beta):
        self.id = id
        self.xPos = xPos
        self.yPos = yPos
        self.alpha = alpha
        self.beta = beta


def getRobotList(filename):
    with open(filename, "r") as f:
        fileLines = f.readlines()
    robotList = []
    for line in fileLines:
        if line.startswith("#"):
            continue
        id, xPos, yPos, alpha, beta = line.split(",")
        robotList.append(
            Robot(
                int(id),
                float(xPos),
                float(yPos),
                float(alpha),
                float(beta),
            )
        )
    return robotList


def topCollideLine(self):
    if self._topCollideLine is None:
        lineBuffer = BetaArmWidth / 2.0
        x, y = self.xyFiberFocal - self.xyAlphaFocal
        fTheta = numpy.arctan2(y, x)
        x1 = numpy.cos(fTheta) * (BetaTopCollide[0] + lineBuffer) + \
            self.xyAlphaFocal[0]
        y1 = numpy.sin(fTheta) * (BetaTopCollide[0] + lineBuffer) + \
            self.xyAlphaFocal[1]

        x2 = numpy.cos(fTheta) * (BetaTopCollide[1] - lineBuffer) + \
            self.xyAlphaFocal[0]
        y2 = numpy.sin(fTheta) * (BetaTopCollide[1] - lineBuffer) + \
            self.xyAlphaFocal[1]
        self._topCollideLine = LineString(
            [[x1, y1], [x2, y2]]
        ).buffer(lineBuffer, cap_style=1)
    return self._topCollideLine


def plotGrid(robotList, title=None, xlim=None, ylim=None, save=True):
    fig = plt.figure(figsize=(10, 10))
    ax = plt.gca()
    # ax = fig.add_subplot(111)
    xAll = [r.xPos for r in robotList]
    yAll = [r.yPos for r in robotList]
    plt.scatter(xAll, yAll)
    for robot in robotList:
        ax.plot(
            [robot.xyFocal[0], robot.xyAlphaFocal[0]],
            [robot.xyFocal[1], robot.xyAlphaFocal[1]],
            '-k.', linewidth=3,
            zorder=8,
        )
        topcolor = "blue"
        bottomcolor = "blue"
        # check for collisions, if so plot as red
        for n in robot.sketchyNeighbors:
            if n.bottomIntersect:
                bottomcolor = 'red'
            if n.topIntersect:
                topcolor = "orange"
        if robot.threwAway:
            topcolor, bottomcolor = "magenta", "magenta"
        if robot.deadLocked:
            topcolor, bottomcolor = "gold", "gold"
        if robot.onTarget:
            topcolor, bottomcolor = "green", "green"
        if robot.alphaDir == -1 or robot.betaDir == -1:
            topcolor, bottomcolor = "cyan", "cyan"
        ax.plot(
            [robot.xyAlphaFocal[0], robot.xyFiberFocal[0]],
            [robot.xyAlphaFocal[1], robot.xyFiberFocal[1]],
            '-', color="green", linewidth=2,
        )
        patch = PolygonPatch(robot._topCollideLine, fc=topcolor, ec=topcolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)
        patch = PolygonPatch(robot._bottomCollideLine, fc=bottomcolor, ec=bottomcolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)
    plt.axis('equal')
    if title is not None:
        plt.title(title)
    if xlim:
        plt.xlim(xlim)
    if ylim:
        plt.ylim(ylim)
    if save:
        plt.savefig(title)
        plt.close()

if __name__ == "__main__":
    # note 1060 is a real bugger!

    # inspectFails(3165)

    # reverseMove(3165)

    # seeds = [20,21,46,58,84,102,109,119,121,133,147,152,169,171,174,195,207,208,221,234,251,263,288,311,322,326,328,334,335,339,356,368,370,384,387,408,416,444,448,453,458,460,467,475,482,486,511,526,556,596,605,614,616,631,641,655,667,694,704,708,709,712,730,762,763,765,768,775,785,803,806,825,839,840,849,861,862,869,898,913,916,919,958,961,963,971,986,994,1002,1022,1030,1033,1044,1054,1055,1060,1068,1070,1080,1108,1113,1129,1157,1196,1198,1206,1226,1228,1230,1241,1260,1264,1266,1303,1304,1318,1328,1339,1357,1403,1421,1428,1448,1461,1466,1475,1481,1482,1484,1495,1514,1562,1597,1601,1602,1608,1611,1616,1628,1632,1644,1650,1676,1684,1688,1696,1707,1718,1731,1738,1760,1767,1774,1802,1831,1851,1853,1859,1875,1879,1886,1895,1896,1901,1906,1912,1920,1927,1928,1955,1968,1982,1983,1990,2014,2016,2023,2028,2041,2047,2059,2060,2070,2085,2094,2098,2118,2121,2129,2150,2194,2206,2220,2238,2258,2277,2279,2286,2300,2306,2324,2355,2366,2402,2409,2429,2430,2432,2433,2464,2474,2475,2476,2483,2511,2514,2525,2537,2582,2591,2596,2643,2649,2690,2709,2712,2720,2742,2747,2755,2757,2782,2789,2799,2809,2816,2838,2840,2858,2862,2869,2873,2897,2908,2928,2933,2944,2969,2970,2976,2981,2991,2992,3003,3004,3013,3018,3022,3040,3058,3061,3068,3080,3090,3094,3098,3111,3115,3120,3123,3128,3141,3163,3165,3168,3178,3179,3180,3212,3253,3277,3285,3291,3315,3316,3339,3343,3355,3362,3365,3368,3373,3377,3386,3390,3400,3425,3457,3470,3471,3476,3478,3527,3553,3569,3588,3599,3607,3635,3641,3645,3658,3665,3668,3669,3688,3691,3692,3697,3703,3716,3719,3723,3730,3733,3739,3752,3770,3788,3793,3810,3818,3843,3858,3872,3879,3883,3924,3928,3942,3954,3964,3985,3987,3991,3999,4067,4079,4088,4101,4109,4134,4137,4158,4184,4206,4210,4215,4257,4261,4265,4268,4269,4302,4311,4312,4326,4381,4382,4408,4413,4420,4431,4451,4476,4477,4480,4488,4491,4494,4496,4504,4528,4555,4556,4563,4570,4597,4610,4623,4633,4637,4648,4654,4658,4661,4666,4667,4671,4672,4694,4698,4712,4736,4739,4762,4780,4786,4788,4796,4811,4820,4864,4867,4890,4894,4896,4899,4901,4902,4913,4920,4932,4953,4959,4960,4982]
    # seeds2 = [24,26,40,62,65,79,80,81,91,95,98,109,113,121,132,149,155,169,184,187,196,197,199,227,236,241,253,275,277,281,306,307,309,311,348,356,360,372,375,389,401,424,459,472,478,482,487,502,509,515,528,551,555,565,569,571,573,582,592,593,608,622,640,650,663,688,698,721,735,748,750,772,786,815,844,877,878,895,896,901,904,908,920,928,937,955,966,968,969,975,984,992,1009,1017,1020,1035,1050,1055,1085,1091,1102,1103,1104,1111,1129,1146,1149,1155,1160,1188,1207,1224,1228,1251,1261,1270,1272,1288,1292,1294,1297,1304,1326,1332,1348,1356,1378,1382,1416,1417,1424,1427,1432,1438,1439,1448,1464,1470,1522,1528,1529,1580,1582,1594,1600,1604,1607,1608,1620,1629,1634,1652,1653,1665,1668,1684,1698,1701,1712,1748,1750,1791,1805,1858,1860,1864,1873,1892,1906,1915,1916,1932,1945,2008,2010,2022,2075,2112,2145,2151,2158,2160,2193,2199,2211,2225,2268,2277,2284,2298,2301,2308,2319,2326,2333,2342,2371,2403,2410,2414,2416,2456,2476,2497,2515,2523,2527,2530,2534,2556,2564,2591,2593,2601,2606,2625,2628,2640,2677,2727,2758,2790,2798,2808,2821,2828,2831,2841,2861,2872,2896,2905,2906,2909,2918,2961,2981,2995,2996,3039,3044,3047,3075,3092,3093,3103,3111,3165,3178,3184,3185,3214,3228,3234,3238,3239,3248,3264,3268,3281,3287,3291,3298,3316,3330,3342,3371,3374,3381,3384,3448,3466,3470,3483,3495,3498,3501,3516,3553,3560,3586,3614,3617,3629,3634,3649,3655,3658,3689,3697,3705,3715,3727,3733,3738,3741,3783,3823,3837,3846,3859,3870,3879,3894,3897,3906,3914,3938,3950,3955,3960,3967,3977,3984,3990,4002,4043,4062,4072,4091,4107,4156,4157,4165,4185,4188,4191,4192,4196,4206,4212,4221,4228,4236,4237,4243,4244,4255,4260,4269,4282,4302,4306,4307,4315,4336,4357,4364,4378,4397,4402,4403,4405,4423,4428,4433,4460,4511,4513,4514,4542,4548,4587,4603,4606,4614,4617,4620,4634,4640,4682,4713,4734,4744,4759,4766,4772,4790,4801,4860,4863,4865,4873,4880,4883,4895,4899,4903,4904,4908,4914,4930,4935,4936,4944,4945,4951,4952,4956,4964,4991]
    # seeds = list(set(seeds+seeds2))

    # seeds = range(5000)
    # p = Pool(29)
    # results = p.map(reverseMove, seeds)
    # with open("results.txt", "w") as f:
    #     f.write("seed, steps, ontarget, total\n")
    #     for result in results:
    #         f.write("%i, %i, %i, %i\n"%tuple(result))


    # known fails failed14109.png  failed6486.png  failed7117.png

    reverseMove(14109)











