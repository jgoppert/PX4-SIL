PX4-SIL
=======

PX4 Software in the loop.

The goal is to develop a useful development and test tool for PX4. By using python, we can tie into useful tools such as sympy to automatically generate equations of motion for unique vehicles. The structure of the simulation is based upon execution of periodic processes. This differs from event based execution for diagrams such as in simulink and scicoslab, but more closely follows execution on the real system.

Use cases (goals):

1. Prototype new estimator/ controller module in python. Test out with pure python modules or optionally, test out with cython modules compiled from actuall C++ Firmware code.

2. Do integration testing for each commit.

3. Develop a control system for a new vehicle. Find equations of motion in sympy, store expressions in lambda functions to pass to numpy, integrate with scipy.integrate.ode. Test control systems against simulated dynamics.

You can view the ipython notebooks using notebook viewer, for example:
* Visit http://nbviewer.ipython.org/github/jgoppert/PX4-SIL/tree/master/
* Click on any of the notebooks with the .ipynb file extension.
