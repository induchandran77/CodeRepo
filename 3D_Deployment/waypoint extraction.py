import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_waypoints(length, breadth, x, y):
    # Initialize list to store waypoints
    waypoints = []
    # Calculate the number of columns and rows
    cols = math.ceil(breadth / y)
    rows = math.ceil(length / x)
    for c in range(cols):
        column_waypoints = []
        for r in range(rows):
            # Calculate the midpoint coordinates
            mid_x = (c * y) + (y / 2)
            mid_y = (r * x) + (x / 2)
            # Append the midpoint to the column list with z-coordinate starting at 0
            column_waypoints.append((mid_x, mid_y, 0))  # Starting z-coordinate at 0
        # Append the column waypoints to the main list
        waypoints.append(column_waypoints)
    return waypoints

# Define the parameters
length = 1000.0  # Length of the area
breadth = 1000.0  # Breadth of the area
altitude = 86.97 # Altitude of the waypoints
x = 2*altitude*math.tan((45/2)*math.pi/180) # Width of the camera footprint
y = x/(4/3)  # Height of the camera footprint
waypoints = calculate_waypoints(length, breadth, x, y)
total_waypoints = sum(len(column) for column in waypoints)
print(f"Total number of waypoints: {total_waypoints}")
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
for column in waypoints:
    for waypoint in column:
        ax.scatter(waypoint[0], waypoint[1], waypoint[2], color='b', marker='o')
ax.set_xlabel('Width W (m)')
ax.set_ylabel('Length L (m)')
ax.set_zlabel('Altitude (m)')
ax.set_zlim(0, None)
plt.title('Waypoints in 3D')
plt.show()
