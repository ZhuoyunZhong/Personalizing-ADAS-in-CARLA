from os import path
from os.path import dirname, abspath
import sys

import numpy as np
from math import pi
from scipy.stats import norm
from sklearn.mixture import GaussianMixture
import matplotlib.pyplot as plt

try:
    sys.path.append(dirname(dirname(dirname(abspath(__file__)))))
except IndexError:
    pass
from agents.learning.model import Model
from agents.learning.model import GMM


# Load data from a file
def load_data(file_name):
    train_path = path.join(dirname(abspath(__file__)), file_name)
    states = None
    if path.exists(train_path):
        with open(train_path, 'rb') as f:
            states = np.loadtxt(f, delimiter=",")
            states = np.atleast_2d(states)
    else:
        print("%s does not exist."% file_name)
    
    return states


# Show some of the data set
def show_data_set(driver, file_name, range):
    states = load_data(driver+"/" + file_name + "_states.csv")
    states = states[range[0]:range[1] ,:]
    model = Model()
    model._state_list = states.tolist()
    if file_name == "target_speed":
        model.update_target_speed(debug=True)
    if file_name == "safe_distance":
        model.update_safe_distance(debug=True)
    if file_name == "GMM":
        model.update_sin_param(debug=True)


# Plot comparison
def plot_comparison(file_name):
    d1 = load_data("Driver1/" + file_name + "_train_data.csv")
    d2 = load_data("Driver2/" + file_name + "_train_data.csv")
    d3 = load_data("Driver3/" + file_name + "_train_data.csv")
    d = [d1, d2, d3]

    plt.figure()
    for driver_num, driver_data in enumerate(d):
        mean = driver_data[:,0]
        cov = driver_data[:,1]
        order = np.sort(mean)
        for i in range(mean.size):
            x = np.linspace(order[0]-3, order[-1]+3, 300).reshape(-1,1)
            y = norm.pdf(x, mean, np.sqrt(cov))
            if i == int(mean.size)-1:
                plt.plot(x, y, c="C"+str(driver_num), label='Driver '+str(driver_num+1))
            else:
                plt.plot(x, y, c="C"+str(driver_num))
            plt.xlabel(file_name)
            plt.ylabel("density of probability")
    plt.legend()
    plt.show()

# GMM train and predict
def gmm_train_and_predict(driver, standard_case):
    gmm = GMM()
    data = load_data(driver + "/GMM_train_data.csv")
    gmm.train(data)

    long_v = np.sum(data[:,5])
    if gmm.GMM_model is not None:
        GMM_v = standard_case
        dt = gmm.predict_value(GMM_v)[0][0]
        if np.isnan(dt) or dt < 0:
            print("GMM model failed, send dt = 4")
        else:
            print("Predict dt: %s from GMM" % (dt))
    
        t = np.linspace(0, dt, 200)
        x = np.linspace(0, long_v*dt, 200)
        y = -(-3.5)/(2*pi) * np.sin(2*pi * t/dt) + (-3.5) * t/dt
        
    return t, x, y

# Plot lane change comparison
def plot_gmm_comparison(standard_case=np.array([[10, -3.5, 15, -12]])):
    t1, x1, y1 = gmm_train_and_predict("Driver1", standard_case)
    t2, x2, y2 = gmm_train_and_predict("Driver2", standard_case)
    t3, x3, y3 = gmm_train_and_predict("Driver3", standard_case)

    fig = plt.figure()

    ax1 = fig.add_subplot(211)
    ax1.plot(t1, y1, c='r', label="Driver1")
    ax1.plot(t2, y2, c='g', label="Driver2")
    ax1.plot(t3, y3, c='b', label="Driver3")
    ax1.set_xlabel("Time s")
    ax1.set_ylabel("Lateral distance m")

    ax2 = fig.add_subplot(212)
    ax2.plot(t1, x1, c='r', label="Driver1")
    ax2.plot(t2, x2, c='g', label="Driver2")
    ax2.plot(t3, x3, c='b', label="Driver3")
    ax2.set_xlabel("Time s")
    ax2.set_ylabel("Longitudinal distance m")

    plt.legend()
    plt.show()


if __name__ == "__main__":
    show_data_set("Driver1", "target_speed", [0, 630])
    show_data_set("Driver1", "safe_distance", [0, 665])
    show_data_set("Driver1", "GMM", [0, 310])

    plot_comparison("target_speed")
    plot_comparison("safe_distance")
    plot_gmm_comparison()
