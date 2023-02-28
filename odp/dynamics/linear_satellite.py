import heterocl as hcl

from odp.dynamics.abstract_dynamics import OdpDynamics


class LinearSatelliteD(OdpDynamics):
    def __init__(self, a, ux_target, uy_target, uz_target):
        self.mu = 3.986e14
        self.a = a
        self.ux_target = ux_target
        self.uy_target = uy_target
        self.uz_target = uz_target
        self.max_control = (1, 1, 1)
        self.min_control = (-1, -1, -1)

    def dynamics(self, t, state, u_opt, d_opt):
        x1_dot = hcl.scalar(0, 'x1_dot')
        x2_dot = hcl.scalar(0, 'x2_dot')
        x3_dot = hcl.scalar(0, 'x3_dot')
        x4_dot = hcl.scalar(0, 'x4_dot')
        x5_dot = hcl.scalar(0, 'x5_dot')
        x6_dot = hcl.scalar(0, 'x6_dot')

        # f(x)
        x1_dot[0] = state[4]
        x2_dot[0] = state[5]
        x3_dot[0] = state[6]
        x4_dot[0] = 3 * hcl.sqrt(self.mu / (self.a * self.a * self.a)) * hcl.sqrt(self.mu / (self.a * self.a * self.a)) * state[0] + 2 * hcl.sqrt(self.mu / (self.a * self.a * self.a)) * state[4] + self.ux_target
        x5_dot[0] = -2 * hcl.sqrt(self.mu / (self.a * self.a * self.a)) * state[3] + self.uy_target
        x6_dot[0] = -(hcl.sqrt(self.mu / (self.a * self.a * self.a)) * hcl.sqrt(self.mu / (self.a * self.a * self.a))) * state[2] + self.uz_target

        # ... + G_u(x) @ u
        # x1_dot[0] = x1_dot[0]
        # x1_dot[0] = x1_dot[0]
        # x1_dot[0] = x1_dot[0]
        x1_dot[0] = x1_dot[0] + u_opt[0]
        x1_dot[0] = x1_dot[0] + u_opt[1]
        x1_dot[0] = x1_dot[0] + u_opt[2]

        # ... + G_d(x) @ d
        # x1_dot[0] = x1_dot[0]
        # x1_dot[0] = x1_dot[0]
        # x1_dot[0] = x1_dot[0]
        # x1_dot[0] = x1_dot[0]
        # x1_dot[0] = x1_dot[0]
        # x1_dot[0] = x1_dot[0]

        return x1_dot[0], x2_dot[0], x3_dot[0], x4_dot[0], x5_dot[0], x6_dot[0]

    def opt_ctrl(self, t, state, spat_deriv):
        uOpt1 = hcl.scalar(0, "uOpt1")
        uOpt2 = hcl.scalar(0, "uOpt2")
        uOpt3 = hcl.scalar(0, "uOpt3")
        uOpt4 = hcl.scalar(0, "uOpt4")
        uOpt5 = hcl.scalar(0, "uOpt4")
        uOpt6 = hcl.scalar(0, "uOpt4")

        SumUUU = hcl.scalar(0, "SumUUU")
        SumUUL = hcl.scalar(0, "SumUUL")
        SumULU = hcl.scalar(0, "SumULU")
        SumLUU = hcl.scalar(0, "SumLUU")
        SumULL = hcl.scalar(0, "SumULL")
        SumLUL = hcl.scalar(0, "SumLUL")
        SumLLU = hcl.scalar(0, "SumLLU")
        SumLLL = hcl.scalar(0, "SumLLL")

        MaxDV = hcl.scalar(0, "MaxDV")

        SumUUU = spat_deriv[3] * self.max_control[0] + spat_deriv[4] * self.max_control[1] + spat_deriv[5] * self.max_control[2]
        SumUUL = spat_deriv[3] * self.max_control[0] + spat_deriv[4] * self.max_control[1] + spat_deriv[5] * self.min_control[2]
        SumULU = spat_deriv[3] * self.max_control[0] + spat_deriv[4] * self.min_control[1] + spat_deriv[5] * self.max_control[2]
        SumLUU = spat_deriv[3] * self.min_control[0] + spat_deriv[4] * self.max_control[1] + spat_deriv[5] * self.max_control[2]
        SumULL = spat_deriv[3] * self.max_control[0] + spat_deriv[4] * self.min_control[1] + spat_deriv[5] * self.min_control[2]
        SumLUL = spat_deriv[3] * self.min_control[0] + spat_deriv[4] * self.max_control[1] + spat_deriv[5] * self.min_control[2]
        SumLLU = spat_deriv[3] * self.min_control[0] + spat_deriv[4] * self.min_control[1] + spat_deriv[5] * self.max_control[2]
        SumLLL = spat_deriv[3] * self.min_control[0] + spat_deriv[4] * self.min_control[1] + spat_deriv[5] * self.min_control[2]

        MaxDV[0] = SumUUU[0]
        uOpt1[0] = self.max_control[0]
        uOpt2[0] = self.max_control[1]
        uOpt3[0] = self.max_control[2]

        with hcl.if_(SumUUL[0] > MaxDV[0]):
            uOpt1[0] = self.max_control[0]
            uOpt2[0] = self.max_control[1]
            uOpt2[0] = self.min_control[2]
            MaxDV[0] = SumUUL[0]

        with hcl.elif_(SumULU[0] > MaxDV[0]):
            uOpt1[0] = self.max_control[0]
            uOpt2[0] = self.min_control[1]
            uOpt2[0] = self.max_control[2]
            MaxDV[0] = SumULU[0]

        with hcl.elif_(SumLUU[0] > MaxDV[0]):
            uOpt1[0] = self.min_control[0]
            uOpt2[0] = self.max_control[1]
            uOpt2[0] = self.max_control[2]
            MaxDV[0] = SumLUU[0]

        with hcl.elif_(SumULL[0] > MaxDV[0]):
            uOpt1[0] = self.max_control[0]
            uOpt2[0] = self.min_control[1]
            uOpt2[0] = self.min_control[2]
            MaxDV[0] = SumULL[0]

        with hcl.elif_(SumLUL[0] > MaxDV[0]):
            uOpt1[0] = self.min_control[0]
            uOpt2[0] = self.max_control[1]
            uOpt2[0] = self.min_control[2]
            MaxDV[0] = SumLUL[0]

        with hcl.elif_(SumLLU[0] > MaxDV[0]):
            uOpt1[0] = self.min_control[0]
            uOpt2[0] = self.min_control[1]
            uOpt2[0] = self.max_control[2]
            MaxDV[0] = SumLLU[0]

        with hcl.elif_(SumLLL[0] > MaxDV[0]):
            uOpt1[0] = self.min_control[0]
            uOpt2[0] = self.min_control[1]
            uOpt2[0] = self.min_control[2]
            MaxDV[0] = SumLLL[0]

        return uOpt1[0], uOpt2[0], uOpt3[0], uOpt4[0], uOpt5[0], uOpt6[0]

    def opt_dstb(self, t, state, spat_deriv):
        d1 = hcl.scalar(0, "d1")
        d2 = hcl.scalar(0, "d2")
        d3 = hcl.scalar(0, "d3")
        d4 = hcl.scalar(0, "d4")
        d5 = hcl.scalar(0, "d5")
        d6 = hcl.scalar(0, "d6")

        return d1[0], d2[0], d3[0], d4[0], d5[0], d6[0]

