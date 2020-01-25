#!/usr/bin/python
# --------------------------------------------------
# Simulation of on single future trajectory.
# --------------------------------------------------

# Imports:
import numpy as np

# Start class:
class SingleSim:

    # Model:
    model = None

    # Settings:
    settings = None

    # Constructor:
    def __init__(self, model):

        # Set Model:
        self.model = model

    # Simulation:
    def sim(self, timeline):

        # Init output:
        predPaths = []

        # Generate Inputs:
        inputs = self.model.u0

        # Simulate the model:
        predPaths.append(self.model.sim(inputs, timeline))

        # Return the paths:
        return np.array(predPaths), [1.0]


#!/usr/bin/python
#--------------------------------------------------
# Numeric ode solver.
#--------------------------------------------------

# Imports:
import numpy as np

# Start Runge-Kutta-4 integration:
def IntegrateRK4(function, initState, inputs, timeLine):

    # Cython defintions:
    k1 = np.zeros((initState.shape[0]), np.float)
    k2 = np.zeros((initState.shape[0]), np.float)
    k3 = np.zeros((initState.shape[0]), np.float)
    k4 = np.zeros((initState.shape[0]), np.float)

    # Initialize output:
    res = np.zeros((timeLine.shape[0], initState.shape[0]), np.float)

    # Set inital state to first output:
    res[0, :] = initState.T

    # Iterate over feature timesteps:
    for nStep in range(1, timeLine.shape[0]):

        # Calculate step size:
        stepSize = timeLine[nStep] - timeLine[nStep - 1]

        # Interim functions:
        k1 = function(res[nStep - 1, :], 0.0, inputs)
        k2 = function(res[nStep - 1, :] + ((stepSize / 2.0) * k1), stepSize / 2.0, inputs)
        k3 = function(res[nStep - 1, :] + ((stepSize / 2.0) * k2), stepSize / 2.0, inputs)
        k4 = function(res[nStep - 1, :] + (stepSize * k3), stepSize, inputs)

        # Calculate new state:
        res[nStep, :] = res[nStep - 1, :] + (stepSize * (((1.0 / 6.0) * k1) + ((1.0 / 3.0) * k2) + ((1.0 / 3.0) * k3) + ((1.0 / 6.0) * k4)))

    # Return:
    return res



# --------------------------------------------------
# Constant Turn Rate and Acceleration model (CTRA)
# --------------------------------------------------

# Imports:
import numpy as np
import math
import warnings

# Start class:
class ModelCTRA:

    # Definitions:
    nInputs = 2 # Acceleration, yaw rate

    # Initial state and input:
    s0 = [0, 0, 0, 0] # X, Y, v, yaw
    u0 = [0, 0]

    # State limits:
    vLimit = [-14, 70]

    # Distribution parameter coefficients interpolators:
    gaussMuAcc = None
    gaussSigmaAcc = None
    gaussMuYawRate = None
    gaussSigmaYawRate = None

    # Constructor:
    def __init__(self):
        pass

    # Initialize the model:
    def init(self, x, y, v, yaw, acc, yawRate):

        # Set state:
        self.s0 = [x, y, v, yaw]
        self.u0 = [acc, yawRate]

    # Model Equations:
    def ode(self, s, t, u):
        # s[0]: x   -> dx(t)   = v(t) * cos(psi(t)) + u[0] * t * cos(psi(t))
        # s[1]: y   -> dy(t)   = v(t) * sin(psi(t)) + u[0] * t * sin(psi(t))
        # s[2]: v   -> dv(t)   = u[0] = const.
        # s[3]: psi -> dpsi(t) = u[1] = const.
        # -----
        # u[0]: dv = a
        # u[1]: dpsi
        # -----

        # Differential:
        ds = np.array([s[2] * math.cos(s[3]), # + (u[0] * t * math.cos(s[3])),
                       s[2] * math.sin(s[3]), # + (u[0] * t * math.sin(s[3])),
                       u[0],
                       u[1]])

        # Constrain velocity:
        if s[2] > self.vLimit[1] or s[2] < self.vLimit[0]:
            ds[2] = 0

        # Saturate at zero velocity:
        if (s[2] > 0 and s[2] + (ds[2] * t) <= 0.0) or (s[2] < 0 and s[2] + (ds[2] * t) >= 0.0):
            ds[2] = -(s[2] / t)

        # Reuturn:
        return ds

    # Return maximum possible inputs based on current state:
    def getInputBounds(self):

        # Calculate limits:
        minAcc = self.limitAccMin(self.s0[2])
        maxAcc = self.limitAccMax(self.s0[2])
        minYawRate = self.limitYawRateMin(self.s0[2])
        maxYawRate = self.limitYawRateMax(self.s0[2])

        # Return:
        return [minAcc, minYawRate], [maxAcc, maxYawRate]

    # Return the parameters for the current distribution of the inputs:
    def getInputDistParams(self):

        # Calculate parameters:
        muAcc = self.gaussMuAcc([self.s0[2], self.u0[0]])[0]
        sigmaAcc = max(0.0001, self.gaussSigmaAcc([self.s0[2], self.u0[0]])[0] * self.settings['simulation']['distributionScale'])
        muYawRate = self.gaussMuYawRate([self.s0[2], self.u0[1]])[0]
        sigmaYawRate = max(0.0001, self.gaussSigmaYawRate([self.s0[2], self.u0[1]])[0] * self.settings['simulation']['distributionScale'])

        # Handle NaN's through extrapolation:
        if np.isnan(muAcc) or np.isnan(sigmaAcc):
            warnings.warn('TO dynamics [v=' + str(self.s0[2]) + '|a=' + str(self.u0[0]) + '] out of bounds for distribution parameter calculation! Using defaults.', Warning)
            muAcc = float(self.u0[0])
            sigmaAcc = 0.0001
        if np.isnan(muYawRate) or np.isnan(sigmaYawRate):
            warnings.warn('TO dynamics [v=' + str(self.s0[2]) + '|w=' + str(self.u0[1]) + '] out of bounds for distribution parameter calculation! Using defaults.', Warning)
            muYawRate = float(self.u0[1])
            sigmaYawRate = 0.0001

        # Return results:
        return [muAcc, muYawRate], [sigmaAcc, sigmaYawRate]

    # Simulate:
    def sim(self, inputs, timeline):

        # Call the solver:
        sol = IntegrateRK4(self.ode, np.array(self.s0), np.array(inputs), np.array(timeline))

        # Return trajectory:
        return [[sol[i, 0], sol[i, 1], sol[i, 3], sol[i, 2]] for i in range(len(sol))]


# Plot resutls:

import matplotlib
matplotlib.use("TkAgg")
from matplotlib import pyplot as plt


def plotResults(timeline, predPath):

    # Figure:
    fig = plt.figure()

    # Plot path(s):
    ax1 = fig.add_subplot(211)
    for nPath in range(len(predPath)):
        ax1.plot([predPath[nPath][i][0] for i in range(len(predPath[nPath]))],
                 [predPath[nPath][i][1] for i in range(len(predPath[nPath]))])
    ax1.axis('equal')

    # Plot velocities:
    ax1 = fig.add_subplot(212)
    for nPath in range(len(predPath)):
        ax1.plot(timeline, [predPath[nPath][i][3] for i in range(len(predPath[nPath]))])

    # Show figure:
    plt.show(block=False)

    # Return plot handle:
    return fig



if __name__ == "__main__":
    model = ModelCTRA()

    model.init(0.0, 0.0, 2.0, 0.0, 1.0, 0.3)

    sgSim = SingleSim(model)

    span = 100
    freq = 10
    timeline = [(1 / float(freq)) * i for i in range(span * freq + 1)]

    predPath, _ = sgSim.sim(timeline)
    print(predPath)

    fig = plotResults(timeline, predPath)
    plt.savefig('foo.png')