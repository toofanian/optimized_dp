import numpy as np
from odp.Grid import Grid
from odp.Plots import PlotOptions
from odp.Shapes import Lower_Half_Space, Upper_Half_Space, Intersection
from odp.Plots import plot_isosurface
import heterocl as hcl
from odp.solver import HJSolver, HJSolverClass


class ActiveCruiseControl:
    def __init__(self, friction_coeffs, target_velocity, mass, uMode="min", dMode="max"):
        self.friction_coefficients = friction_coeffs
        self.target_velocity = target_velocity
        self.mass = mass
        self.uMode = uMode
        self.control_upper_bounds = [5000.0]
        self.dMode = dMode

    def opt_ctrl(self, t, state, spat_deriv):
        opt_a = hcl.scalar(self.control_upper_bounds[0], "opt_a")
        in2 = hcl.scalar(0, "in2")
        in3 = hcl.scalar(0, "in3")

        with hcl.if_(spat_deriv[1] < 0):
            opt_a[0] = -opt_a

        return opt_a[0], in2[0], in3[0]

    def opt_dstb(self, t, state, spat_deriv):
        d1 = hcl.scalar(0, "d1")
        d2 = hcl.scalar(0, "d2")
        d3 = hcl.scalar(0, "d3")
        return d1[0], d2[0], d3[0]

    def dynamics(self, t, state, u_opt, d_opt):
        x1_dot = hcl.scalar(0, "x1_dot")
        x2_dot = hcl.scalar(0, "x2_dot")
        x3_dot = hcl.scalar(0, "x3_dot")

        x1_dot[0] = state[1]
        x2_dot[0] = -1 / self.mass * \
            (
                    self.friction_coefficients[0] +
                    self.friction_coefficients[1] *
                    state[1] +
                    self.friction_coefficients[2] *
                    state[1]*state[1]
            ) \
            + 1 / self.mass * u_opt[0]
        x3_dot[0] = self.target_velocity - state[1]
        return x1_dot[0], x2_dot[0], x3_dot[0]


g = Grid(np.array([0.0, -20.0, 20.0]), np.array([1e3, 20.0, 80.0]), 3, np.array([50, 50, 50]))
shape1 = Lower_Half_Space(g, dim=2, value=60)
shape2 = Upper_Half_Space(g, dim=2, value=40)
initial_value_f = -Intersection(shape1, shape2)
tau = [0, 0.1]
my_car = ActiveCruiseControl([0.1, 5.0, 0.25], 0.0, 1650, uMode="max", dMode="min")
po2 = PlotOptions(do_plot=False, plot_type="2d_plot", plotDims=[1, 2], slicesCut=[0])
compMethods = {"TargetSetMode": "minVWithV0"}

solver = HJSolverClass()

old_vals = initial_value_f
vals = [initial_value_f]
t_total = 0
import time
init_time = time.time()
for i in range(50):
    # TODO: Analyze how much extra time this takes
    # active_set = np.random.randint(0, 2, size=initial_value_f.shape)
    active_set = old_vals > -0.1
    new_vals = solver(my_car, g, old_vals, tau, compMethods, po2, saveAllTimeSteps=False, active_set=active_set)
    t_total += tau[1]
    vals += [new_vals]
    old_vals = new_vals

t_total = time.time() - init_time
print(t_total)
vals = np.array(vals)
vals = np.moveaxis(vals, 0, -1)
vals = np.flip(vals, axis=-1)
plot_isosurface(g, vals, po2)


# init_time2 = time.time()
# other_vals = HJSolver(my_car, g, initial_value_f, [0, 5], compMethods, po2, saveAllTimeSteps=True)
# t_total2 = time.time() - init_time2
# print(t_total2)
# plot_isosurface(g, other_vals, po2)
# assert np.isclose(vals[-1], other_vals).all()
