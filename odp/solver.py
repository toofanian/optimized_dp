import heterocl as hcl
import numpy as np
import time

from odp.Plots import plot_isosurface

# Backward reachable set computation library
from odp.computeGraphs import graph_3D, graph_4D, graph_5D, graph_6D
from odp.TimeToReach import TTR_3D, TTR_4D, TTR_5D

# Value Iteration library
from odp.valueIteration import value_iteration_3D, value_iteration_4D, value_iteration_5D, value_iteration_6D


def solveValueIteration(MDP_obj):
    print("Welcome to optimized_dp \n")
    # Initialize the HCL environment
    hcl.init()
    hcl.config.init_dtype = hcl.Float(32)

    ########################################## INITIALIZE ##########################################

    # Convert the python array to hcl type array
    V_opt = hcl.asarray(np.zeros(MDP_obj._ptsEachDim))
    intermeds = hcl.asarray(np.ones(MDP_obj._actions.shape[0]))
    trans = hcl.asarray(MDP_obj._trans)
    gamma = hcl.asarray(MDP_obj._gamma)
    epsilon = hcl.asarray(MDP_obj._epsilon)
    count = hcl.asarray(np.zeros(1))
    maxIters = hcl.asarray(MDP_obj._maxIters)
    actions = hcl.asarray(MDP_obj._actions)
    bounds = hcl.asarray(MDP_obj._bounds)
    goal = hcl.asarray(MDP_obj._goal)
    ptsEachDim = hcl.asarray(MDP_obj._ptsEachDim)
    sVals = hcl.asarray(np.zeros([MDP_obj._bounds.shape[0]]))
    iVals = hcl.asarray(np.zeros([MDP_obj._bounds.shape[0]]))
    interpV = hcl.asarray(np.zeros([1]))
    useNN = hcl.asarray(MDP_obj._useNN)

    print(MDP_obj._bounds.shape[0])
    print(np.zeros([MDP_obj._bounds.shape[0]]))
    if MDP_obj._bounds.shape[0] == 3:
        fillVal = hcl.asarray(MDP_obj._fillVal)
        f = value_iteration_3D(MDP_obj)
    if MDP_obj._bounds.shape[0] == 4:
        f = value_iteration_4D(MDP_obj)
    if MDP_obj._bounds.shape[0] == 5:
        f = value_iteration_5D(MDP_obj)
    if MDP_obj._bounds.shape[0] == 6:
        f = value_iteration_6D(MDP_obj)

    # Build the graph and use the executable
    # Now use the executable
    t_s = time.time()
    if MDP_obj._bounds.shape[0] == 3:
        f(
            V_opt,
            actions,
            intermeds,
            trans,
            interpV,
            gamma,
            epsilon,
            iVals,
            sVals,
            bounds,
            goal,
            ptsEachDim,
            count,
            maxIters,
            useNN,
            fillVal,
        )
    else:
        f(
            V_opt,
            actions,
            intermeds,
            trans,
            interpV,
            gamma,
            epsilon,
            iVals,
            sVals,
            bounds,
            goal,
            ptsEachDim,
            count,
            maxIters,
            useNN,
        )
    t_e = time.time()

    V = V_opt.asnumpy()
    c = count.asnumpy()
    print("Finished in ", int(c[0]), " iterations")
    print("Took        ", t_e - t_s, " seconds")

    # # Write results to file
    # if (MDP_obj.dir_path):
    #     dir_path = MDP_obj.dir_path
    # else:
    #     dir_path = "./hcl_value_matrix_test/"
    #
    # if (MDP_obj.file_name):
    #     file_name = MDP_obj.file_name
    # else:
    #     file_name = "hcl_value_iteration_" + str(int(c[0])) + "_iterations_by" + (
    #         "_Interpolation" if MDP_obj._useNN[0] == 0 else "_NN")
    # MDP_obj.writeResults(V, dir_path, file_name, just_values=True)
    return V


class HJSolverClass:
    initialized = False

    def initialize(
        self,
        dynamics_obj,
        grid,
        init_value,
        tau,
        compMethod,
        plot_option,
        saveAllTimeSteps=False,
        accuracy="low",
        untilConvergent=False,
        epsilon=2e-3,
        verbose=True,
        int_scheme="first",
        global_minimizing=False,
    ):
        if verbose:
            print("Welcome to optimized_dp \n")
        # if type(multiple_value) == list:
        #     # We have both goal and obstacle set
        #     self.target = multiple_value[0] # Target set
        #     self.constraint = multiple_value[1] # Obstacle set
        # else:
        #     self.target = multiple_value
        #     self.constraint = None
        self.target = init_value
        hcl.init()
        hcl.config.init_dtype = hcl.Float(32)

        if verbose:
            print("Initializing\n")

        # Array for each state values
        list_x1 = np.reshape(grid.vs[0], grid.pts_each_dim[0])
        list_x2 = np.reshape(grid.vs[1], grid.pts_each_dim[1])
        list_x3 = np.reshape(grid.vs[2], grid.pts_each_dim[2])
        if grid.dims >= 4:
            list_x4 = np.reshape(grid.vs[3], grid.pts_each_dim[3])
        if grid.dims >= 5:
            list_x5 = np.reshape(grid.vs[4], grid.pts_each_dim[4])
        if grid.dims >= 6:
            list_x6 = np.reshape(grid.vs[5], grid.pts_each_dim[5])
        # Convert state arrays to hcl array type
        self.list_x1 = hcl.asarray(list_x1)
        self.list_x2 = hcl.asarray(list_x2)
        self.list_x3 = hcl.asarray(list_x3)
        if grid.dims >= 4:
            self.list_x4 = hcl.asarray(list_x4)
        if grid.dims >= 5:
            self.list_x5 = hcl.asarray(list_x5)
        if grid.dims >= 6:
            self.list_x6 = hcl.asarray(list_x6)
        # Get executable, obstacle check intial value function
        print(f"Integration scheme of {int_scheme} order")
        print("Global minimizing: ", global_minimizing)
        if grid.dims == 3:
            self.solve_pde = graph_3D(
                dynamics_obj,
                grid,
                compMethod["TargetSetMode"],
                accuracy,
                verbose=verbose,
                int_scheme=int_scheme,
                global_minimizing=global_minimizing,
            )

        if grid.dims == 4:
            self.solve_pde = graph_4D(
                dynamics_obj,
                grid,
                compMethod["TargetSetMode"],
                accuracy,
                verbose=verbose,
                int_scheme=int_scheme,
                global_minimizing=global_minimizing,
            )

        if grid.dims == 5:
            self.solve_pde = graph_5D(dynamics_obj, grid, compMethod["TargetSetMode"], accuracy, verbose=verbose)

        if grid.dims == 6:
            self.solve_pde = graph_6D(
                dynamics_obj,
                grid,
                compMethod["TargetSetMode"],
                accuracy,
                verbose=verbose,
                global_minimizing=global_minimizing,
            )
        self.l0 = hcl.asarray(init_value)
        self.initialized = True
        self.execution_time = 0

    def __call__(
        self,
        dynamics_obj,
        grid,
        init_value,
        tau,
        compMethod,
        plot_option,
        saveAllTimeSteps=False,
        accuracy="low",
        untilConvergent=False,
        epsilon=2e-3,
        active_set=None,
        verbose=True,
        int_scheme="third",
        global_minimizing=False,
    ):
        if not self.initialized:
            self.initialize(
                dynamics_obj,
                grid,
                init_value,
                tau,
                compMethod,
                plot_option,
                saveAllTimeSteps,
                accuracy,
                untilConvergent,
                epsilon,
                verbose=verbose,
                int_scheme=int_scheme,
                global_minimizing=global_minimizing,
            )
        # Tensors input to our computation graph
        V_0 = hcl.asarray(init_value)
        V_1 = hcl.asarray(np.zeros(tuple(grid.pts_each_dim)))
        if active_set is None:
            active_set = np.ones(tuple(grid.pts_each_dim), dtype=float)
        dummy_flags = hcl.asarray(active_set, dtype=hcl.Float())
        # dummy_flags = hcl.asarray(np.random.randint(0, 2, size=tuple(grid.pts_each_dim)))
        # dummy_flags = hcl.asarray(np.zeros(tuple(grid.pts_each_dim)))
        # Variables used for timing
        iter = 0
        tNow = tau[0]
        if verbose:
            print("Started running\n")

        # Backward reachable set/tube will be computed over the specified time horizon
        # Or until convergent ( which ever happens first )
        for i in range(1, len(tau)):
            # tNow = tau[i-1]
            t_minh = hcl.asarray(np.array((tNow, tau[i])))

            while tNow <= tau[i] - 1e-4:
                prev_arr = V_0.asnumpy()
                # Start timing
                iter += 1
                start = time.time()

                global_minimizer = hcl.asarray(np.array([self.l0.asnumpy().min()]))
                # Run the execution and pass input into graph
                if grid.dims == 3:
                    self.solve_pde(
                        V_1,
                        V_0,
                        self.list_x1,
                        self.list_x2,
                        self.list_x3,
                        t_minh,
                        self.l0,
                        dummy_flags,
                        global_minimizer,
                    )
                if grid.dims == 4:
                    self.solve_pde(
                        V_1,
                        V_0,
                        self.list_x1,
                        self.list_x2,
                        self.list_x3,
                        self.list_x4,
                        t_minh,
                        self.l0,
                        dummy_flags,
                        global_minimizer,
                    )
                if grid.dims == 5:
                    self.solve_pde(
                        V_1,
                        V_0,
                        self.list_x1,
                        self.list_x2,
                        self.list_x3,
                        self.list_x4,
                        self.list_x5,
                        t_minh,
                        self.l0,
                        dummy_flags,
                        global_minimizer,
                    )
                if grid.dims == 6:
                    self.solve_pde(
                        V_1,
                        V_0,
                        self.list_x1,
                        self.list_x2,
                        self.list_x3,
                        self.list_x4,
                        self.list_x5,
                        self.list_x6,
                        t_minh,
                        self.l0,
                        dummy_flags,
                        global_minimizer,
                    )

                tNow = t_minh.asnumpy()[0]

                # Calculate computation time
                self.execution_time += time.time() - start

                # Some information printin
                if verbose:
                    print(t_minh)
                    print("Computational time to integrate (s): {:.5f}".format(time.time() - start))
                    print("Total kernel time (s): {:.5f}".format(self.execution_time))

                if untilConvergent is True:
                    # Compare difference between V_{t-1} and V_{t} and choose the max changes
                    diff = np.amax(np.abs(V_1.asnumpy() - prev_arr))
                    if verbose:
                        print("Max difference between V_old and V_new : {:.5f}".format(diff))
                    if diff < epsilon:
                        if verbose:
                            print("Result converged ! Exiting the compute loop. Have a good day.")
                        break
            else:  # if it didn't break because of convergent condition
                continue
            break  # only if convergent condition is achieved
        if verbose:
            print("Total kernel time (s): {:.5f}".format(self.execution_time))
            print("Finished solving\n")
        return V_1.asnumpy()


def TTRSolver(dynamics_obj, grid, init_value, epsilon, plot_option):
    print("Welcome to optimized_dp \n")
    ################# INITIALIZE DATA TO BE INPUT INTO EXECUTABLE ##########################

    print("Initializing\n")
    hcl.init()
    hcl.config.init_dtype = hcl.Float(32)

    # Convert initial distance value function to initial time-to-reach value function
    init_value[init_value < 0] = 0
    init_value[init_value > 0] = 1000
    V_0 = hcl.asarray(init_value)
    prev_val = np.zeros(init_value.shape)

    # Re-shape states vector
    list_x1 = np.reshape(grid.vs[0], grid.pts_each_dim[0])
    list_x2 = np.reshape(grid.vs[1], grid.pts_each_dim[1])
    list_x3 = np.reshape(grid.vs[2], grid.pts_each_dim[2])
    if grid.dims >= 4:
        list_x4 = np.reshape(grid.vs[3], grid.pts_each_dim[3])
    if grid.dims >= 5:
        list_x5 = np.reshape(grid.vs[4], grid.pts_each_dim[4])
    if grid.dims >= 6:
        list_x6 = np.reshape(grid.vs[5], grid.pts_each_dim[5])

    # Convert states vector to hcl array type
    list_x1 = hcl.asarray(list_x1)
    list_x2 = hcl.asarray(list_x2)
    list_x3 = hcl.asarray(list_x3)
    if grid.dims >= 4:
        list_x4 = hcl.asarray(list_x4)
    if grid.dims >= 5:
        list_x5 = hcl.asarray(list_x5)
    if grid.dims >= 6:
        list_x6 = hcl.asarray(list_x6)

    # Get executable

    if grid.dims == 3:
        solve_TTR = TTR_3D(dynamics_obj, grid)
    if grid.dims == 4:
        solve_TTR = TTR_4D(dynamics_obj, grid)
    if grid.dims == 5:
        solve_TTR = TTR_5D(dynamics_obj, grid)
    if grid.dims == 6:
        solve_TTR = TTR_6D(dynamics_obj, grid)
    print("Got Executable\n")

    # Print out code for different backend
    # print(solve_pde)

    ################ USE THE EXECUTABLE ############
    error = 10000
    count = 0
    start = time.time()
    while error > epsilon:
        print("Iteration: {} Error: {}".format(count, error))
        count += 1
        if grid.dims == 3:
            solve_TTR(V_0, list_x1, list_x2, list_x3)
        if grid.dims == 4:
            solve_TTR(V_0, list_x1, list_x2, list_x3, list_x4)
        if grid.dims == 5:
            solve_TTR(V_0, list_x1, list_x2, list_x3, list_x4, list_x5)
        if grid.dims == 6:
            solve_TTR(V_0, list_x1, list_x2, list_x3, list_x4, list_x5, list_x6)

        error = np.max(np.abs(prev_val - V_0.asnumpy()))
        prev_val = V_0.asnumpy()
    print("Total TTR computation time (s): {:.5f}".format(time.time() - start))
    print("Finished solving\n")

    ##################### PLOTTING #####################
    plot_isosurface(grid, V_0.asnumpy(), plot_option)
    return V_0.asnumpy()


def computeSpatDerivArray(grid, V, deriv_dim, accuracy="low"):
    # Return a tensor same size as V that contains spatial derivatives at every state in V
    hcl.init()
    hcl.config.init_dtype = hcl.Float(32)

    # Need to make sure that value array has the same size as grid
    assert list(V.shape) == list(grid.pts_each_dim)

    V_0 = hcl.asarray(V)
    spatial_deriv = hcl.asarray(np.zeros(tuple(grid.pts_each_dim)))

    # Get executable, obstacle check intial value function
    if grid.dims == 3:
        compute_SpatDeriv = graph_3D(None, grid, "None", accuracy, generate_SpatDeriv=True, deriv_dim=deriv_dim)
    if grid.dims == 4:
        compute_SpatDeriv = graph_4D(None, grid, "None", accuracy, generate_SpatDeriv=True, deriv_dim=deriv_dim)
    if grid.dims == 5:
        compute_SpatDeriv = graph_5D(None, grid, "None", accuracy, generate_SpatDeriv=True, deriv_dim=deriv_dim)

    compute_SpatDeriv(V_0, spatial_deriv)
    return spatial_deriv.asnumpy()
