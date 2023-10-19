import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.cm import ScalarMappable, get_cmap
from matplotlib.colors import Normalize

def map_colors(p3dc, norm, cmap='viridis'):

    # reconstruct the triangles from internal data
    x, y, z, _ = p3dc._vec
    slices = p3dc._segslices
    triangles = np.array([np.array((x[s],y[s],z[s])).T for s in slices])

    # usual stuff
    colors = get_cmap(cmap)(norm)
    breakpoint()

    # set the face colors of the Poly3DCollection
    p3dc.set_fc(colors)

    # if the caller wants a colorbar, they need this
    return ScalarMappable(cmap=cmap, norm=norm)


# Assuming vertices is a Nx3 array for N vertices and
# triangles is a Mx3 array for M triangles, and
# stresses is a M-length array of stress magnitudes.
vertices_df = pd.read_csv('vertices.csv', header=None, names=['x', 'y', 'z'])
vertices = vertices_df.values

triangles_df = pd.read_csv('triangles.csv', header=None, names=['x', 'y', 'z'], dtype=int)
triangles = triangles_df.values

stresses_df = pd.read_csv('stresses.csv', header=None, names=['x', 'y', 'z', 'norm'])
stress_magnitudes = stresses_df['norm'].values

# Scale to be between 0 and 1
stress_magnitudes_normed = stress_magnitudes / np.max(stress_magnitudes)

stress_magnitudes_log = np.log(stress_magnitudes)
# Now have negative values; scale again between 0 and 1.
stress_magnitudes_log_normed = (stress_magnitudes_log - np.min(stress_magnitudes_log)) / (np.max(stress_magnitudes_log) - np.min(stress_magnitudes_log))
# breakpoint()

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Plotting the mesh and color it based on stress magnitude
p3dc = ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2],
                triangles=triangles,
)

p3dc.set_fc(plt.cm.viridis(stress_magnitudes_normed))

plt.show()
