from typing import Callable

import heterocl as hcl

from odp.Grid import Grid
from odp.dynamics.abstract_dynamics import OdpDynamics


class Quad4DFixedPolicy(OdpDynamics):
    def __init__(self, policy, grid: Grid):
        self.policy = policy
        self.grid: Grid = grid

        self.gravity: float = 9.81
        self.mass: float = 2.5
        self.drag_coefficient_v: float = .25
        self.drag_coefficient_phi: float = .02255
        self.length_between_copters: float = 1.0
        self.moment_of_inertia: float = 1.0

        self.min_thrust: float = 0
        self.max_thrust: float = 18.39375

    def dynamics(self, t, state, u_opt, d_opt):
        x1_dot = hcl.scalar(0, 'x1_dot')
        x2_dot = hcl.scalar(0, 'x2_dot')
        x3_dot = hcl.scalar(0, 'x3_dot')
        x4_dot = hcl.scalar(0, 'x4_dot')

        x1_dot[0] = (state[1])
        x2_dot[0] = (-self.drag_coefficient_v/self.mass * state[1] - self.gravity) + ((hcl.cos(state[2]) / self.mass * u_opt[0]) + (hcl.cos(state[2]) / self.mass * u_opt[1]))
        x3_dot[0] = (state[3])
        x4_dot[0] = (-self.drag_coefficient_phi / self.moment_of_inertia * state[3]) + ((-self.length_between_copters / self.moment_of_inertia * u_opt[0]) + (self.length_between_copters / self.moment_of_inertia * u_opt[1]))
        return x1_dot[0], x2_dot[0], x3_dot[0], x4_dot[0]

    def opt_ctrl(self, t, state, spat_deriv):
        uOpt1 = hcl.scalar(0, "uOpt1")
        uOpt2 = hcl.scalar(0, "uOpt2")
        uOpt3 = hcl.scalar(0, "uOpt3")
        uOpt4 = hcl.scalar(0, "uOpt4")

        control = hcl.asarray(self.policy)[get_index_from_state_and_grid_hcl_4d(state, self.grid)]

        uOpt1[0] = control[0]
        uOpt2[0] = control[1]
        uOpt3[0] = control[2]
        uOpt4[0] = control[3]

    def opt_dstb(self, t, state, spat_deriv):
        d1 = hcl.scalar(0, "d1")
        d2 = hcl.scalar(0, "d2")
        d3 = hcl.scalar(0, "d3")
        d4 = hcl.scalar(0, "d4")

        return d1[0], d2[0], d3[0], d4[0]


def get_index_from_state_and_grid_hcl_4d(state, grid: Grid):
    grid_index_1 = hcl.scalar(0, 'grid_index_1')
    grid_index_2 = hcl.scalar(0, 'grid_index_1')
    grid_index_3 = hcl.scalar(0, 'grid_index_1')
    grid_index_4 = hcl.scalar(0, 'grid_index_1')

    # convert state to grid frame
    grid_index_1[0] = (state[0] + grid.dx[0] / 2 - grid.min[0]) // grid.dx[0]
    grid_index_2[1] = (state[1] + grid.dx[0] / 2 - grid.min[1]) // grid.dx[1]
    grid_index_3[2] = (state[2] + grid.dx[0] / 2 - grid.min[2]) // grid.dx[2]
    grid_index_4[3] = (state[3] + grid.dx[0] / 2 - grid.min[3]) // grid.dx[3]

    return grid_index_1[0], grid_index_2[0], grid_index_3[0], grid_index_4[0]
