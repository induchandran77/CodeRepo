#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

TOLERANCE = 0.5  # Increased tolerance value
LANDING_ALTITUDE_THRESHOLD = 0.1
DRONE_COUNT = 8
states = [State() for _ in range(DRONE_COUNT)]
poses = [PoseStamped() for _ in range(DRONE_COUNT)]
initial_positions = [PoseStamped() for _ in range(DRONE_COUNT)]
target_pose_index = 0
waypoints = [
    [0.0, 0.0, 10.0], [0.0, 10.0, 10.0], [0.0, 20.0, 10.0], [0.0, 30.0, 10.0],
    [0.0, 40.0, 10.0], [0.0, 50.0, 10.0], [0.0, 60.0, 10.0], [0.0, 70.0, 10.0], [0.0, 80.0, 10.0], [0.0, 0.0, 10.0]
]
offsets = [i * 2.0 for i in range(DRONE_COUNT)]
fault_injected = False
falling = False
faulty_drone_index = 2  # Index of the drone to be made faulty
maintain_line_formation = True  # Flag to maintain line formation
returning_to_initial_positions = False  # Flag to indicate return to initial positions
drones_landed = [False] * DRONE_COUNT  # Track if each drone has landed

def state_cb(index):
    def callback(msg):
        global states
        states[index] = msg
    return callback

def pose_cb(index):
    def callback(msg):
        global poses
        poses[index] = msg
    return callback

def set_target_pose(target, waypoint, offset, maintain_line_formation):
    if maintain_line_formation:
        target.pose.position.x = waypoint[0] + offset
    else:
        target.pose.position.x = waypoint[0]
    target.pose.position.y = waypoint[1]
    target.pose.position.z = waypoint[2]

def drone_reached_target(index, waypoint):
    x_diff = abs(poses[index].pose.position.x - waypoint[0] - (offsets[index] if maintain_line_formation else 0))
    y_diff = abs(poses[index].pose.position.y - waypoint[1])
    z_diff = abs(poses[index].pose.position.z - waypoint[2])

    return x_diff < TOLERANCE and y_diff < TOLERANCE and z_diff < TOLERANCE

def trigger_landing_and_disarm(index):
    rospy.loginfo(f"Setting drone {index} to AUTO.LAND mode...")
    set_mode_clients[index].call(SetModeRequest(custom_mode="AUTO.LAND"))

    rospy.loginfo(f"Waiting for drone {index} to land...")
    while poses[index].pose.position.z > LANDING_ALTITUDE_THRESHOLD:
        rospy.sleep(1)

    rospy.loginfo(f"Disarming drone {index}...")
    arming_clients[index].call(CommandBoolRequest(value=False))
    rospy.loginfo(f"Drone {index} disarmed.")
    drones_landed[index] = True

def inject_fault():
    global fault_injected, falling
    if not fault_injected:
        rospy.logwarn(f"Injecting fault in drone {faulty_drone_index}")
        fault_injected = True
        falling = True
        # Disarm the drone to simulate fault
        arming_clients[faulty_drone_index].call(CommandBoolRequest(value=False))

if __name__ == "__main__":
    rospy.init_node("offb_node")

    state_subs = []
    pose_subs = []
    local_pos_pubs = []
    arming_clients = []
    set_mode_clients = []

    for i in range(DRONE_COUNT):
        ns = f"/uav{i}"
        state_subs.append(rospy.Subscriber(f"{ns}/mavros/state", State, callback=state_cb(i)))
        pose_subs.append(rospy.Subscriber(f"{ns}/mavros/local_position/pose", PoseStamped, callback=pose_cb(i)))
        local_pos_pubs.append(rospy.Publisher(f"{ns}/mavros/setpoint_position/local", PoseStamped, queue_size=10))
        rospy.wait_for_service(f"{ns}/mavros/cmd/arming")
        arming_clients.append(rospy.ServiceProxy(f"{ns}/mavros/cmd/arming", CommandBool))
        rospy.wait_for_service(f"{ns}/mavros/set_mode")
        set_mode_clients.append(rospy.ServiceProxy(f"{ns}/mavros/set_mode", SetMode))

    rate = rospy.Rate(20)

    rospy.loginfo("Waiting for all drones to connect...")
    while not rospy.is_shutdown() and not all(state.connected for state in states):
        rate.sleep()

    for i in range(DRONE_COUNT):
        poses[i].header.frame_id = 'base_link'
        poses[i].header.stamp = rospy.Time.now()
        poses[i].pose.position.x = offsets[i]
        poses[i].pose.position.y = 0.0
        poses[i].pose.position.z = 10.0
        initial_positions[i].pose.position.x = poses[i].pose.position.x
        initial_positions[i].pose.position.y = poses[i].pose.position.y
        initial_positions[i].pose.position.z = poses[i].pose.position.z
        rospy.loginfo(f"Drone {i} initial pose: {poses[i].pose}")

    rospy.loginfo("Sending initial setpoints...")
    for _ in range(100):
        if rospy.is_shutdown():
            break
        for pub, pose in zip(local_pos_pubs, poses):
            pub.publish(pose)
        rate.sleep()

    rospy.loginfo("Setting OFFBOARD mode and arming drones...")
    offb_set_modes = [SetModeRequest(custom_mode='OFFBOARD') for _ in range(DRONE_COUNT)]
    arm_cmds = [CommandBoolRequest(value=True) for _ in range(DRONE_COUNT)]
    last_req = rospy.Time.now()

    all_offboard_enabled = False
    all_armed = False

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        if not all_offboard_enabled:
            all_offboard_enabled = all(set_mode_clients[i].call(offb_set_modes[i]).mode_sent for i in range(DRONE_COUNT))
            if all_offboard_enabled:
                rospy.loginfo("OFFBOARD mode enabled for all UAVs")

        if all_offboard_enabled and not all_armed:
            all_armed = all(arming_clients[i].call(arm_cmds[i]).success for i in range(DRONE_COUNT))
            if all_armed:
                rospy.loginfo("All UAVs armed")

        if returning_to_initial_positions:
            for i in range(DRONE_COUNT):
                if not drones_landed[i] and i != faulty_drone_index:
                    if drone_reached_target(i, [initial_positions[i].pose.position.x, initial_positions[i].pose.position.y, 0.1]):
                        rospy.loginfo(f"Drone {i} reached its initial position, starting landing sequence.")
                        trigger_landing_and_disarm(i)
                    else:
                        set_target_pose(poses[i], [initial_positions[i].pose.position.x, initial_positions[i].pose.position.y, 0.1], 0, maintain_line_formation)
                        poses[i].header.stamp = rospy.Time.now()
                        local_pos_pubs[i].publish(poses[i])
                        rospy.loginfo(f"Drone {i} returning to initial position: {poses[i].pose}")

            if all(drones_landed):
                rospy.loginfo("All drones have landed.")
                break
        else:
            if all(drones_landed[i] or drone_reached_target(i, waypoints[target_pose_index]) for i in range(DRONE_COUNT) if i != faulty_drone_index):
                for i in range(DRONE_COUNT):
                    if not drones_landed[i] and i != faulty_drone_index:
                        set_target_pose(poses[i], waypoints[target_pose_index], offsets[i], maintain_line_formation)
                        poses[i].header.stamp = rospy.Time.now()
                        local_pos_pubs[i].publish(poses[i])
                        rospy.loginfo(f"Drone {i} target pose: {poses[i].pose}")
                if all(drone_reached_target(i, waypoints[target_pose_index]) for i in range(DRONE_COUNT) if i != faulty_drone_index):
                    rospy.loginfo(f"All drones reached waypoint {target_pose_index}")
                    if target_pose_index == 2 and not fault_injected:  # Change the condition to waypoint 2
                        inject_fault()
                    if target_pose_index < len(waypoints) - 1:
                        target_pose_index += 1
                        rospy.loginfo(f"Moving to waypoint {target_pose_index}")
                    else:
                        # All waypoints have been covered, stop maintaining line formation
                        rospy.loginfo("All waypoints covered, returning to initial positions")
                        maintain_line_formation = False
                        returning_to_initial_positions = True
                        target_pose_index = 0  # Reset target index to use initial positions for landing

        for i in range(DRONE_COUNT):
            if not drones_landed[i]:
                if fault_injected and i == faulty_drone_index:
                    # Simulate falling by decreasing altitude without control
                    if poses[faulty_drone_index].pose.position.z > 0:
                        poses[faulty_drone_index].pose.position.z -= 1.0  # Faster fall
                    else:
                        poses[faulty_drone_index].pose.position.z = 0  # Ensure it doesn't go below ground level

                    # Publish the updated pose for the faulty drone
                    poses[faulty_drone_index].header.stamp = current_time
                    local_pos_pubs[faulty_drone_index].publish(poses[faulty_drone_index])
                    continue

                set_target_pose(poses[i], waypoints[target_pose_index], offsets[i], maintain_line_formation)
                poses[i].header.stamp = current_time
                local_pos_pubs[i].publish(poses[i])
                rospy.loginfo(f"Drone {i} target pose: {poses[i].pose}")

        rate.sleep()
