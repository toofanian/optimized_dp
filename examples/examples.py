import imp
import numpy as np
# Utility functions to initialize the problem
from odp.Grid import Grid
from odp.Shapes import *

# Specify the  file that includes dynamic systems
from odp.dynamics import DubinsCapture, DubinsCar, ActiveCruiseControl
from odp.dynamics import DubinsCar4D2
# Plot options
from odp.Plots import PlotOptions
# Solver core
from odp.solver import HJSolver, computeSpatDerivArray

import math

""" USER INTERFACES
- Define grid
- Generate initial values for grid using shape functions
- Time length for computations
- Initialize plotting option
- Call HJSolver function
"""

##################################################### EXAMPLE 1 #####################################################

# g = Grid(np.array([-4.0, -4.0, -math.pi]), np.array([4.0, 4.0, math.pi]), 3, np.array([40, 40, 40]), [2])

# # Implicit function is a spherical shape - check out CylinderShape API
# Initial_value_f = CylinderShape(g, [], np.zeros(3), 1)

# # Look-back length and time step of computation
# lookback_length = 5.0
# t_step = 0.05

# small_number = 1e-5
# tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

# # Specify the dynamical object. DubinsCapture() has been declared in dynamics/
# # Control uMode is maximizing, meaning that we're avoiding the initial target set
# my_car = DubinsCapture(uMode="min", dMode="max", dMax=0.0, wMax=0.5)

# # Specify how to plot the isosurface of the value function ( for higher-than-3-dimension arrays, which slices, indices
# # we should plot if we plot at all )
# po2 = PlotOptions(do_plot=True, plot_type="2d_plot", plotDims=[0,1],
#                   slicesCut=[20])

"""
Assign one of the following strings to `TargetSetMode` to specify the characteristics of computation
"TargetSetMode":
{
"none" -> compute Backward Reachable Set,
"minVWithV0" -> min V with V0 (compute Backward Reachable Tube),
"maxVWithV0" -> max V with V0,
"maxVWithVInit" -> compute max V over time,
"minVWithVInit" -> compute min V over time,
"minVWithVTarget" -> min V with target set (if target set is different from initial V0)
"maxVWithVTarget" -> max V with target set (if target set is different from initial V0)
}

(optional)
Please specify this mode if you would like to add another target set, which can be an obstacle set
for solving a reach-avoid problem
"ObstacleSetMode":
{
"minVWithObstacle" -> min with obstacle set,
"maxVWithObstacle" -> max with obstacle set
}
"""

# # In this example, we compute a Backward Reachable Tube
# compMethods = { "TargetSetMode": "minVWithV0"}
# # HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
# result = HJSolver(my_car, g, Initial_value_f, tau, compMethods, po2, saveAllTimeSteps=True )
# # np.save("/home/ubuntu/dev/odp_test_ST/results_examples.npy", result)
# last_time_step_result = result[..., 0]
# # Compute spatial derivatives at every state
# x_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=1, accuracy="low")
# y_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=2, accuracy="low")
# T_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=3, accuracy="low")

# # Let's compute optimal control at some random idices
# spat_deriv_vector = (x_derivative[10,20,30], y_derivative[10,20,30], T_derivative[10,20,30])
# state_vector = (g.grid_points[0][10], g.grid_points[1][20], g.grid_points[2][30])

# # Compute the optimal control
# opt_ctrl = my_car.optCtrl_inPython(state_vector, spat_deriv_vector)
# print("Optimal control is {}\n".format(opt_ctrl))

# g = Grid(np.array([-4.0, -4.0, -math.pi]), np.array([4.0, 4.0, math.pi]), 3, np.array([40, 40, 40]), [2])

# initial_value_f = -CylinderShape(g, [2], np.zeros(3), 3.5)

# lookback_length = 10.0
# t_step = 0.05

# small_number = 1e-5
# tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

# my_car = DubinsCar(uMode="max", dMode="min", dMax=0.0, wMax=0.5, speed=1)

# po2 = PlotOptions(do_plot=True, plot_type="2d_plot", plotDims=[0,1], slicesCut=[20])

# compMethods = {"TargetSetMode": "minVWithV0"}

# result = HJSolver(my_car, g, initial_value_f, tau, compMethods, po2, saveAllTimeSteps=True)


g = Grid(np.array([0.0, -20.0, 20.0]), np.array([1e3, 20.0, 80.0]), 3, np.array([50, 50, 50]))
shape1 = Lower_Half_Space(g, dim=2, value=60)
shape2 = Upper_Half_Space(g, dim=2, value=40)
initial_value_f = -Intersection(shape1, shape2)
lookback_length = 5.0
t_step = 0.05

small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

my_car = ActiveCruiseControl([0.1, 5.0, 0.25], 0.0, 1650, uMode="max", dMode="min")

po2 = PlotOptions(do_plot=True, plot_type="2d_plot", plotDims=[1, 2], slicesCut=[0])

compMethods = {"TargetSetMode": "minVWithV0"}
result = HJSolver(my_car, g, initial_value_f, tau, compMethods, po2, saveAllTimeSteps=True)


##################################################### EXAMPLE 2 #####################################################

# g = Grid(np.array([-3.0, -1.0, 0.0, -math.pi]), np.array([3.0, 4.0, 4.0, math.pi]), 4, np.array([60, 60, 20, 36]), [3])

# # Define my object
# my_car = DubinsCar4D2()

# # Use the grid to initialize initial value function
# Initial_value_f = CylinderShape(g, [2,3], np.zeros(4), 1)

# # Look-back length and time step
# lookback_length = 1.0
# t_step = 0.05

# small_number = 1e-5

# tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

# po = PlotOptions(do_plot=True, plot_type="3d_plot", plotDims=[0,1,3],
#                   slicesCut=[19])

# # In this example, we compute a Backward Reachable Tube
# compMethods = { "TargetSetMode": "minVWithV0"}
# result = HJSolver(my_car, g, Initial_value_f, tau, compMethods, po, saveAllTimeSteps=True)

# last_time_step_result = result[..., 0]

# # Compute spatial derivatives at every state
# x_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=1, accuracy="low")
# y_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=2, accuracy="low")
# v_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=3, accuracy="low")
# T_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=4, accuracy="low")

# # Let's compute optimal control at some random idices
# spat_deriv_vector = (x_derivative[10,20,15,15], y_derivative[10,20,15,15],
#                      v_derivative[10,20,15,15], T_derivative[10,20,15,15])

# # Compute the optimal control
# opt_a, opt_w = my_car.optCtrl_inPython(spat_deriv_vector)
# print("Optimal accel is {}\n".format(opt_a))
# print("Optimal rotation is {}\n".format(opt_w))

# ##################################################### EXAMPLE 3 #####################################################

# Third scenario with Reach-Avoid set
# g = Grid(np.array([-4.0, -4.0, -math.pi]), np.array([4.0, 4.0, math.pi]), 3, np.array([40, 40, 40]), [2])

# # Reachable set
# goal = CylinderShape(g, [2], np.zeros(3), 0.5)

# # Avoid set
# obstacle = CylinderShape(g, [2], np.array([1.0, 1.0, 0.0]), 0.5)

# # Look-back length and time step
# lookback_length = 1.5
# t_step = 0.05

# small_number = 1e-5
# tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

# my_car = DubinsCapture(uMode="min", dMode="max")

# po2 = PlotOptions(do_plot=True, plot_type="2d_plot", plotDims=[0,1],
#                   slicesCut=[20])

# """
# Assign one of the following strings to `TargetSetMode` to specify the characteristics of computation
# "TargetSetMode":
# {
# "none" -> compute Backward Reachable Set,
# "minVWithV0" -> min V with V0 (compute Backward Reachable Tube),
# "maxVWithV0" -> max V with V0,
# "maxVWithVInit" -> compute max V over time,
# "minVWithVInit" -> compute min V over time,
# "minVWithVTarget" -> min V with target set (if target set is different from initial V0)
# "maxVWithVTarget" -> max V with target set (if target set is different from initial V0)
# }

# (optional)
# Please specify this mode if you would like to add another target set, which can be an obstacle set
# for solving a reach-avoid problem
# "ObstacleSetMode":
# {
# "minVWithObstacle" -> min with obstacle set,
# "maxVWithObstacle" -> max with obstacle set
# }
# """

# compMethods = { "TargetSetMode": "minVWithVTarget",
#                 "ObstacleSetMode": "maxVWithObstacle"}
# # HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
# result = HJSolver(my_car, g, [goal, obstacle], tau, compMethods, po2, saveAllTimeSteps=True )
