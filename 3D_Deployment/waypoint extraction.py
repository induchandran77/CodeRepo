# # import math

# # def calculate_waypoints(length, breadth, x, y):
# #     # Initialize list to store waypoints
# #     waypoints = []

# #     #calculate the number of columns and rows
# #     cols = math.ceil(breadth / y)
# #     rows = math.ceil(length / x)

# #     #create waypoints for each block
# #     for r in range(rows):
# #         for c in range(cols):
# #             # Calculate the midpoint coordinates
# #             mid_x = (c * y) + (y / 2)
# #             mid_y = (r * x) + (x / 2)
# #             # Append the midpoint to the list of waypoints
# #             waypoints.append((mid_x, mid_y))

# #     return waypoints
# # #define the parameters
# # length = 500.0  # Length of the area
# # breadth = 500.0  # Breadth of the area
# # x = 83.67  # Width of the camera footprint
# # y = 62.7  # Height of the camera footprint

# # waypoints = calculate_waypoints(length, breadth, x, y)

# # # Print the waypoints
# # for waypoint in waypoints:
# #     print(waypoint)


# import math

# def calculate_waypoints(length, breadth, x, y):
#     # Initialize list to store waypoints
#     waypoints = []

#     # Calculate the number of columns and rows
#     cols = math.ceil(breadth / y)
#     rows = math.ceil(length / x)

#     # Create waypoints for each block
#     for c in range(cols):
#         column_waypoints = []
#         for r in range(rows):
#             # Calculate the midpoint coordinates
#             mid_x = (c * y) + (y / 2)
#             mid_y = (r * x) + (x / 2)
#             # Append the midpoint to the column list
#             column_waypoints.append((mid_x, mid_y))
#         # Append the column waypoints to the main list
#         waypoints.append(column_waypoints)

#     return waypoints

# # Define the parameters
# length = 1000.0  # Length of the area
# breadth = 1000.0  # Breadth of the area
# # x = 72.06317731365867  # Width of the camera footprint
# # y = 54.047382985244  # Height of the camera footprint
# x = 83.67  # Width of the camera footprint
# y = 62.7  # Height of the camera footprint

# waypoints = calculate_waypoints(length, breadth, x, y)

# # Print the waypoints
# print(waypoints)


import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_waypoints(length, breadth, x, y):
    # Initialize list to store waypoints
    waypoints = []

    # Calculate the number of columns and rows
    cols = math.ceil(breadth / y)
    rows = math.ceil(length / x)

    # Create waypoints for each block
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


# x = 72.06317731365867  # Width of the camera footprint
# y = 54.047382985244  # Height of the camera footprint
# altitude = 86.97  # Altitude of the waypoints
waypoints = calculate_waypoints(length, breadth, x, y)


total_waypoints = sum(len(column) for column in waypoints)
print(f"Total number of waypoints: {total_waypoints}")

# Plot the waypoints in 3D
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Flatten the waypoints list for plotting
for column in waypoints:
    for waypoint in column:
        ax.scatter(waypoint[0], waypoint[1], waypoint[2], color='b', marker='o')

# Set labels
ax.set_xlabel('Width W (m)')
ax.set_ylabel('Length L (m)')
ax.set_zlabel('Altitude (m)')

# Set z-axis limits to start from 0
ax.set_zlim(0, None)

plt.title('Waypoints in 3D')
plt.show()
