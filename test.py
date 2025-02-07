import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Step 1: Define a simple oval path
theta = np.linspace(0, 2 * np.pi, 100)  # Angle from 0 to 2π
radius = 5  # Outer radius of the track
inner_radius = 4  # Inner radius of the track

# Outer track points
x_outer = radius * np.cos(theta)
y_outer = radius * np.sin(theta)

# Inner track points
x_inner = inner_radius * np.cos(theta)
y_inner = inner_radius * np.sin(theta)

# Combine points into a single array
outer_track = np.array([x_outer, y_outer])
inner_track = np.array([x_inner, y_inner])

# Step 2: Function to create 3D track from 2D points
def create_3d_track(outer_track, inner_track, height=1.0):
    outer_faces = []
    inner_faces = []

    for i in range(len(outer_track[0]) - 1):
        # Define vertices for outer track
        outer_faces.append([
            [outer_track[0][i], outer_track[1][i], 0],
            [outer_track[0][i], outer_track[1][i], height],
            [outer_track[0][i + 1], outer_track[1][i + 1], height],
            [outer_track[0][i + 1], outer_track[1][i + 1], 0]
        ])

        # Define vertices for inner track
        inner_faces.append([
            [inner_track[0][i], inner_track[1][i], 0],
            [inner_track[0][i], inner_track[1][i], height],
            [inner_track[0][i + 1], inner_track[1][i + 1], height],
            [inner_track[0][i + 1], inner_track[1][i + 1], 0]
        ])

    return outer_faces, inner_faces

outer_faces, inner_faces = create_3d_track(outer_track, inner_track)

# Step 3: Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Add outer track faces
outer_poly = Poly3DCollection(outer_faces, color='blue', alpha=0.6)
ax.add_collection3d(outer_poly)

# Add inner track faces
inner_poly = Poly3DCollection(inner_faces, color='red', alpha=0.6)
ax.add_collection3d(inner_poly)

# Set limits
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_xlim([-6, 6])
ax.set_ylim([-6, 6])
ax.set_zlim([0, 2])

plt.title("3D Race Track Visualization")
plt.show()

