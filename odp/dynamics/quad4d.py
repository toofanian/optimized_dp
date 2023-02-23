import heterocl as hcl

from odp.dynamics.abstract_dynamics import OdpDynamics


class Quad4D(OdpDynamics):
    def __init__(self):
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

        # x1_dot[0] = (state[1])
        # x2_dot[0] = (-self.drag_coefficient_v/self.mass * state[1] - self.gravity) + ((hcl.cos(state[2]) / self.mass * u_opt[0]) + (hcl.cos(state[2]) / self.mass * u_opt[1]))
        # x3_dot[0] = (state[3])
        # x4_dot[0] = (-self.drag_coefficient_phi / self.moment_of_inertia * state[3]) + ((-self.length_between_copters / self.moment_of_inertia * u_opt[0]) + (self.length_between_copters / self.moment_of_inertia * u_opt[1]))
        return x1_dot[0], x2_dot[0], x3_dot[0], x4_dot[0]

    def opt_ctrl(self, t, state, spat_deriv):
        uOpt1 = hcl.scalar(0, "uOpt1")
        uOpt2 = hcl.scalar(0, "uOpt2")
        uOpt3 = hcl.scalar(0, "uOpt3")
        uOpt4 = hcl.scalar(0, "uOpt4")
        #
        # SumUU = hcl.scalar(0, "SumUU")
        # SumUL = hcl.scalar(0, "SumUL")
        # SumLU = hcl.scalar(0, "SumLU")
        # SumLL = hcl.scalar(0, "SumLL")
        #
        # MaxDV = hcl.scalar(0, "MaxDV")
        #
        # SumUU[0] = spat_deriv[1]*((-self.drag_coefficient_v/self.mass * state[1] - self.gravity) + ((hcl.cos(state[2]) / self.mass * self.max_thrust) + (hcl.cos(state[2]) / self.mass * self.max_thrust))) + spat_deriv[3]*((-self.drag_coefficient_phi / self.moment_of_inertia * state[3]) + ((-self.length_between_copters / self.moment_of_inertia * self.max_thrust) + (self.length_between_copters / self.moment_of_inertia * self.max_thrust)))
        # SumUL[0] = spat_deriv[1]*((-self.drag_coefficient_v/self.mass * state[1] - self.gravity) + ((hcl.cos(state[2]) / self.mass * self.max_thrust) + (hcl.cos(state[2]) / self.mass * self.max_thrust))) + spat_deriv[3]*((-self.drag_coefficient_phi / self.moment_of_inertia * state[3]) + ((-self.length_between_copters / self.moment_of_inertia * self.min_thrust) + (self.length_between_copters / self.moment_of_inertia * self.min_thrust)))
        # SumLU[0] = spat_deriv[1]*((-self.drag_coefficient_v/self.mass * state[1] - self.gravity) + ((hcl.cos(state[2]) / self.mass * self.min_thrust) + (hcl.cos(state[2]) / self.mass * self.min_thrust))) + spat_deriv[3]*((-self.drag_coefficient_phi / self.moment_of_inertia * state[3]) + ((-self.length_between_copters / self.moment_of_inertia * self.max_thrust) + (self.length_between_copters / self.moment_of_inertia * self.max_thrust)))
        # SumLL[0] = spat_deriv[1]*((-self.drag_coefficient_v/self.mass * state[1] - self.gravity) + ((hcl.cos(state[2]) / self.mass * self.min_thrust) + (hcl.cos(state[2]) / self.mass * self.min_thrust))) + spat_deriv[3]*((-self.drag_coefficient_phi / self.moment_of_inertia * state[3]) + ((-self.length_between_copters / self.moment_of_inertia * self.min_thrust) + (self.length_between_copters / self.moment_of_inertia * self.min_thrust)))
        #
        # MaxDV[0] = SumUU[0]
        # uOpt1[0] = self.max_thrust
        # uOpt2[0] = self.max_thrust
        #
        # with hcl.if_(SumUL[0] > MaxDV[0]):
        #     uOpt1[0] = self.max_thrust
        #     uOpt2[0] = self.min_thrust
        #     MaxDV[0] = SumUL[0]
        #
        # with hcl.elif_(SumLU[0] > MaxDV[0]):
        #     uOpt1[0] = self.min_thrust
        #     uOpt2[0] = self.max_thrust
        #     MaxDV[0] = SumLU[0]
        #
        # with hcl.if_(SumLL[0] > MaxDV[0]):
        #     uOpt1[0] = self.min_thrust
        #     uOpt2[0] = self.min_thrust

        return uOpt1[0], uOpt2[0], uOpt3[0], uOpt4[0]

    def opt_dstb(self, t, state, spat_deriv):
        d1 = hcl.scalar(0, "d1")
        d2 = hcl.scalar(0, "d2")
        d3 = hcl.scalar(0, "d3")
        d4 = hcl.scalar(0, "d4")

        return d1[0], d2[0], d3[0], d4[0]

