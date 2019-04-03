import numpy as np
import os
import sys


def noise_stats(name,noise_file):
    print(name)
    print("============================")

    # read GPS data
    noise = np.loadtxt(noise_file, skiprows=1, delimiter=",")

    # check length of data
    print(len(noise))

    # check sample rate
    t = noise[:, 0]
    dt = np.diff(t)
    print("DT      ", np.mean(dt), 1.0 / np.mean(dt))

    # get mean and standard deviation
    x = noise[:, 1]
    mu = np.mean(x)
    st = np.std(x)
    print(f'{name} mean {mu} stddev {st}')


def noise_data(f_in, f_out):
    """
    accumulate noise data over multiple runs
    :param f_in: log file from noise scenario
    :param f_out: output file of noise data
    :return:
    """
    # read the input file first row
    with open(f_in, "r") as infile:
        # skip first line
        line = infile.readline()
        # append the data to the specified output file
        with open(f_out, "a") as outfile:
            line = infile.readline()
            while line:
                outfile.write(line)
                line = infile.readline()


# uncomment these to accumulate more data from additional runs
noise_data("../config/log/Graph1.txt", "gps_data.txt")
noise_data("../config/log/Graph2.txt", "acc_data.txt")

# compute GPS noise stats
noise_stats("GPS","gps_data.txt")

# compute ACC noise stats
noise_stats("ACC", "acc_data.txt")

"""
GPS
============================
164
DT       0.050305312883435586 19.87861604831162
GPS mean -0.03645068902439024 stddev 0.7154310058595122
ACC
============================
1694
DT       0.004999878322504429 200.00486721826903
ACC mean -0.01661793447461629 stddev 0.4913677662625582
"""