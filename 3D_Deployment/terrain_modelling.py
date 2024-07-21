import numpy as np
import plotly.offline as go_offline
import plotly.graph_objects as go

file = open("C:survey_data.csv")
lines = file.readlines()
n_line = len(lines)
x = []
y = []
z = []
for i in range(1, n_line):
    split_line = lines[i].split(",")
    xyz_t = []
    x.append(float(split_line[0].rstrip()))
    y.append(float(split_line[1].rstrip()))
    z.append(float(split_line[2].rstrip()))

# DISTANCE FUNCTION
def distance(x1, y1, x2, y2):
    d = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return d

# CREATING IDW FUNCTION
def idw_npoint(xz, yz, n_point, p):
    r = 10  # block radius iteration distance
    nf = 0
    while nf <= n_point:  # will stop when np reaching at least n_point
        x_block = []
        y_block = []
        z_block = []
        r += 10  # add 10 unit each iteration
        xr_min = xz - r
        xr_max = xz + r
        yr_min = yz - r
        yr_max = yz + r
        for i in range(len(x)):
            # condition to test if a point is within the block
            if ((x[i] >= xr_min and x[i] <= xr_max) and (y[i] >= yr_min and y[i] <= yr_max)):
                x_block.append(x[i])
                y_block.append(y[i])
                z_block.append(z[i])
        nf = len(x_block)  # calculate number of point in the block

    # calculate weight based on distance and p value
    w_list = []
    for j in range(len(x_block)):
        d = distance(xz, yz, x_block[j], y_block[j])
        if d > 0:
            w = 1 / (d ** p)
            w_list.append(w)
            z0 = 0
        else:
            w_list.append(0)  # if meet this condition, it means d<=0, weight is set to 0

    # check if there is 0 in weight list
    w_check = 0 in w_list
    if w_check == True:
        idx = w_list.index(0)  # find index for weight=0
        z_idw = z_block[idx]  # set the value to the current sample value
    else:
        wt = np.transpose(w_list)
        z_idw = np.dot(z_block, wt) / sum(w_list)  # idw calculation using dot product
    return z_idw

# POPULATE INTERPOLATION POINTS
n = 1000  # number of interpolation point for x and y axis
x_min = min(x)
x_max = max(x)
y_min = min(y)
y_max = max(y)
w = x_max - x_min  # width
h = y_max - y_min  # length
wn = w / n  # x interval
hn = h / n  # y interval

# list to store interpolation point and elevation
y_init = y_min
x_init = x_min
x_idw_list = []
y_idw_list = []
z_head = []
z_interpolated = []  # Store interpolated values
x_interpolated = []  # Store x coordinates for each interpolated z value
y_interpolated = []  # Store y coordinates for each interpolated z value

for i in range(n):
    xz = x_init + wn * i
    yz = y_init + hn * i
    y_idw_list.append(yz)
    x_idw_list.append(xz)
    z_idw_list = []
    z_interpolated_row = []  # Store interpolated values for this row
    x_interpolated_row = []  # Store x coordinates for this row
    y_interpolated_row = []  # Store y coordinates for this row
    for j in range(n):
        xz = x_init + wn * j
        z_idw = idw_npoint(xz, yz, 5, 1.5)  # min. point=5, p=1.5
        z_idw_list.append(z_idw)
        z_interpolated_row.append(z_idw)  # Append the interpolated value
        x_interpolated_row.append(xz)  # Append the x coordinate
        y_interpolated_row.append(yz)  # Append the y coordinate
    z_head.append(z_idw_list)
    z_interpolated.append(z_interpolated_row)
    x_interpolated.append(x_interpolated_row)
    y_interpolated.append(y_interpolated_row)

print("x_interpolated",x_interpolated)
print("y_interpolated",y_interpolated)
print("z_interpolated",z_interpolated)

data_dict = {}

# Iterate through the lists and create the dictionary
for i in range(len(x_interpolated)):
    for j in range(len(x_interpolated[i])):
        key = (x_interpolated[i][j], y_interpolated[i][j])
        value = z_interpolated[i][j]
        data_dict[key] = value

def alt_adjust(key):
    value = data_dict[key]
    if value:
        # Print the value
        #print("Value associated with key {}: {}".format(key, value))
        return value
    else:
        print("Value not found")

point_markers = [
    {"name": "Point A", "waypoints": [(x[0], y[0]), (x[10], y[10]), (x[20], y[20])]},
    {"name": "Point B", "waypoints": [(x[30], y[30]), (x[40], y[40]), (x[50], y[50])]}
    # Add more point markers with their respective waypoints as needed
]

# Calculate altitude offset and movement speed
altitude_offset = 10  # Adjust this value as needed
movement_speed = 0.1  # Adjust this value as needed

# Function to calculate altitude of point marker based on terrain elevation
def calculate_altitude(x, y):
    z_idw = idw_npoint(x, y, 5, 1.5)  # Interpolate terrain elevation at given (x, y)
    return z_idw + altitude_offset  # Add altitude offset

# Function to update position of point marker along its waypoints
def update_point_marker_position(marker, point_marker_positions, current_position_index):
    current_position = point_marker_positions[marker["name"]]
    if current_position_index < len(marker["waypoints"]):
        next_waypoint = marker["waypoints"][current_position_index]
        if current_position != next_waypoint:
            # Move towards next waypoint
            dx = next_waypoint[0] - current_position[0]
            dy = next_waypoint[1] - current_position[1]
            distance_to_waypoint = np.sqrt(dx**2 + dy**2)
            if distance_to_waypoint > movement_speed:
                # Move towards next waypoint
                angle = np.arctan2(dy, dx)
                new_x = current_position[0] + movement_speed * np.cos(angle)
                new_y = current_position[1] + movement_speed * np.sin(angle)
                new_z = calculate_altitude(new_x, new_y)
                point_marker_positions[marker["name"]] = (new_x, new_y, new_z)
            else:
                # Arrived at waypoint, move to next waypoint
                current_position_index += 1
    return current_position_index

def terrain_plot(list_of_coord):

    # Combine waypoints for all points
    all_waypoints = list_of_coord

    # Create animation frames
    frames = []
    frame_count = len(all_waypoints[0]) 
    print("Flag1")
    for i in range(frame_count):
        frame_data = []
        
        # Iterate over each point
        for j, waypoints in enumerate(all_waypoints):
            waypoint = waypoints[i]
            
            # Move flying point along x-axis
            flying_point_data = go.Scatter3d(x=[waypoint[0]], y=[waypoint[1]], z=[waypoint[2]],
                                            mode='markers', marker=dict(color=f'rgb({ 50}, {50}, {50})', size=7))
            
            frame_data.append(flying_point_data)
        
        # Add terrain (assuming it's the same for all points)
        terrain_data = go.Surface(z=z_head, x=x_idw_list, y=y_idw_list)
        frame_data.append(terrain_data)
        
        frames.append(go.Frame(data=frame_data))
        print("Flag2")
        # Create figure
    fig = go.Figure()

    # Add terrain
    terrain = go.Surface(z=z_head, x=x_idw_list, y=y_idw_list)
    fig.add_trace(terrain)

    # Add initial flying points traces to figure
    initial_flying_points = []
    for j, waypoints in enumerate(all_waypoints):
        initial_waypoint = waypoints[0]
        initial_flying_point = go.Scatter3d(x=[initial_waypoint[0]], y=[initial_waypoint[1]], z=[waypoint[2]],
                                            mode='markers', marker=dict(color=f'rgb({50}, {50}, {50})', size=7))
        initial_flying_points.append(initial_flying_point)

    fig.add_traces(initial_flying_points)
    print("Flag3")
    # Update layout
    fig.update_layout(scene=dict(aspectratio=dict(x=2, y=2, z=0.5), xaxis=dict(range=[x_min, x_max]), yaxis=dict(range=[y_min, y_max])),
                    updatemenus=[dict(type='buttons',
                                        showactive=False,
                                        buttons=[dict(label='Play',
                                                    method='animate',
                                                    args=[None, dict(frame=dict(duration=400, redraw=True), fromcurrent=True)])])])

    # Add animation frames to figure
    fig.frames = frames
    print("Flag4")
    # Save plot as HTML file
    go_offline.plot(fig, filename='C:3d_terrain_with_waypoints.html', validate=True, auto_open=False)
