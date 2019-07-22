from kaiju import Robot
import numpy


def test_problematicXY():
    # problematic xy for boss/apogee fiber?
    #655  22.399635035412551076 y -0.12777439845131582929
    # this issue arose when we found that the top fiber
    # couldn't reach full extension in the old snowflake ferrule
    x = 22.399635035412551076
    y = -0.12777439845131582929
    robot = Robot(0, 0, 0, 1)
    ab = robot.alphaBetaFromFiberXY(x, y, 1)
    assert not numpy.isnan(ab[0]) and not numpy.isnan(ab[1])


def test_alphaBetaXY():
    robot = Robot(0, 0, 0, 1)
    alphaBeta = []
    for i in range(1000000):
        robot.setXYUniform()
        alphaBeta.append([robot.alpha, robot.beta])
    alphaBeta = numpy.asarray(alphaBeta)
    maxVals = numpy.max(alphaBeta, axis=0)
    minVals = numpy.min(alphaBeta, axis=0)
    print("max ab", maxVals)
    print("min vals", minVals)

    assert minVals[0] > 0 and minVals[1] > 0
    assert maxVals[0] < 360 and maxVals[1] <= 180

# test_alphaBetaXY()
# # test_problematicXY2()


