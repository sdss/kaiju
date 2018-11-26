import numpy
import matplotlib.pyplot as plt
import glob
import multiprocessing

steps = []
filenames = glob.glob("success*.txt")
for file in filenames:
    with open(file, "r") as f:
        line = f.readline()
        step = int(line.split("step=")[-1].split(")")[0])
        steps.append(step)
steps = numpy.asarray(steps) # steps are increments of 1 degree
speed = 30.0 # degrees per sec (5 rpm)

seconds = steps / speed


plt.hist(seconds, 100)
plt.xlabel("seconds")
plt.title("Configuration Times (folded<-->onTarget)")
plt.show()
