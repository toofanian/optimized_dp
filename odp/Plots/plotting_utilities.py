import plotly.graph_objects as go
import numpy as np
import plotly.io as pio
pio.renderers.default = "browser"

# TODO: copy to server, use same script ()


def plot_2d(grid, V, plot_option):

    dims_plot = plot_option.dims_plot
    idx = [slice(None)] * grid.dims
    slice_idx = 0

    dims_list = list(range(grid.dims))
    for i in dims_list:
        if i not in dims_plot:
            idx[i] = plot_option.slices[slice_idx]
            slice_idx += 1

    # Take the two dims from the plot
    dim1, dim2 = dims_plot[0], dims_plot[1]

    my_V_start = np.swapaxes(V[..., -1], dim1, dim2)[tuple(idx)]
    my_V_end = np.swapaxes(V[..., 0], dim1, dim2)[tuple(idx)]
    complex_x = complex(0, grid.pts_each_dim[dim1])
    complex_y = complex(0, grid.pts_each_dim[dim2])
    mg_X, mg_Y = np.mgrid[grid.min[dim1] : grid.max[dim1] : complex_x, grid.min[dim2] : grid.max[dim2] : complex_y]

    print("In the 2d case")
    fig = go.Figure(
        data=[
            go.Contour(
                z=my_V_end.flatten(),
                x=mg_X.flatten(),
                y=mg_Y.flatten(),
                # contours_coloring="lines",
                # colorscale="Hot",
                # contours=dict(
                #     start=0.0,
                #     end=3.0,
                #     size=1.0,
                # ),
                # line_width=4,
            ),
            go.Contour(
                z=my_V_end.flatten(),
                x=mg_X.flatten(),
                y=mg_Y.flatten(),
                contours=dict(
                    start=0.0,
                    end=0.1,
                    size=1.0
                ),
                line_width=4
            ),
            go.Contour(
                z=my_V_start.flatten(),
                x=mg_X.flatten(),
                y=mg_Y.flatten(),
                contours_coloring="lines",
                colorscale="teal",
                contours=dict(
                    start=0.0,
                    end=0.1,
                    size=1.0,
                ),
                line_width=4,
            ),
        ]
    )
    fig.show()


def plot_isosurface(grid, V, plot_option):
    print("IN the 3rd case")
    dims_plot = plot_option.dims_plot
    idx = [slice(None)] * grid.dims
    slice_idx = 0

    dims_list = list(range(grid.dims))
    for i in dims_list:
        if i not in dims_plot:
            idx[i] = plot_option.slices[slice_idx]
            slice_idx += 1

    if len(dims_plot) != 3:
        plot_2d(grid, V, plot_option)
    else:
        dim1, dim2, dim3 = dims_plot[0], dims_plot[1], dims_plot[2]
        complex_x = complex(0, grid.pts_each_dim[dim1])
        complex_y = complex(0, grid.pts_each_dim[dim2])
        complex_z = complex(0, grid.pts_each_dim[dim3])
        mg_X, mg_Y, mg_Z = np.mgrid[
            grid.min[dim1] : grid.max[dim1] : complex_x,
            grid.min[dim2] : grid.max[dim2] : complex_y,
            grid.min[dim3] : grid.max[dim3] : complex_z,
        ]

        my_V = V[tuple(idx)]

        if (V > 0.0).all() or (V < 0.0).all():
            print("Implicit surface will not be shown since all values have the same sign ")
        print("Plotting beautiful plots. Please wait\n")
        fig = go.Figure(
            data=go.Isosurface(
                x=mg_X.flatten(),
                y=mg_Y.flatten(),
                z=mg_Z.flatten(),
                value=my_V.flatten(),
                colorscale="jet",
                isomin=plot_option.min_isosurface,
                surface_count=1,
                isomax=plot_option.max_isosurface,
                caps=dict(x_show=True, y_show=True),
            )
        )
        fig.show()
        print("Please check the plot on your browser.")

