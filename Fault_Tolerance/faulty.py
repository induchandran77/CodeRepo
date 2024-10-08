#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf.transformations as tf_trans

TOLERANCE = 0.2  # Adjusted tolerance value
LANDING_ALTITUDE_THRESHOLD = 0.1
DRONE_COUNT = 8
states = [State() for _ in range(DRONE_COUNT)]
poses = [PoseStamped() for _ in range(DRONE_COUNT)]
target_pose_index = 0
waypoints = [
    [0.0, 0.0, 10.0], [0.0, 10.0, 10.0], [0.0, 20.0, 10.0], [0.0, 30.0, 10.0], 
    [0.0, 40.0, 10.0], [0.0, 50.0, 10.0], [0.0, 60.0, 10.0], [0.0, 70.0, 10.0], [0.0, 80.0, 10.0], [0.0, 0.0, 10.0]
]
offsets = [i * 2.0 for i in range(DRONE_COUNT)]
fault_injected = False
falling = False
faulty_drone_index = 2  # Index of the drone to be made faulty

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

def set_target_pose(target, waypoint, offset):
    target.pose.position.x = waypoint[0] + offset
    target.pose.position.y = waypoint[1]
    target.pose.position.z = waypoint[2]

def all_drones_reached_target():
    for i in range(DRONE_COUNT):
        if fault_injected and i == faulty_drone_index:  # Ignore the faulty drone after fault is injected
            continue

        x_diff = abs(poses[i].pose.position.x - waypoints[target_pose_index][0] - offsets[i])
        y_diff = abs(poses[i].pose.position.y - waypoints[target_pose_index][1])
        z_diff = abs(poses[i].pose.position.z - waypoints[target_pose_index][2])

        if not (x_diff < TOLERANCE and y_diff < TOLERANCE and z_diff < TOLERANCE):
            rospy.loginfo(f"Drone {i} has not reached the target: x_diff={x_diff}, y_diff={y_diff}, z_diff={z_diff}")
            return False
    return True

def all_drones_landed():
    for i in range(DRONE_COUNT):
        if poses[i].pose.position.z > LANDING_ALTITUDE_THRESHOLD:
            return False
    return True

def trigger_landing_and_disarm():
    rospy.loginfo("Setting drones to AUTO.LAND mode...")
    for i in range(DRONE_COUNT):
        set_mode_clients[i].call(SetModeRequest(custom_mode="AUTO.LAND"))
    rospy.sleep(10)  # Wait for landing
    rospy.loginfo("Checking if drones have landed...")
    while not all_drones_landed():
        rospy.sleep(1)
    rospy.loginfo("Disarming drones...")
    for i in range(DRONE_COUNT):
        arming_clients[i].call(CommandBoolRequest(value=False))
    rospy.loginfo("Drones disarmed.")

def inject_fault():
    global fault_injected, falling
    if not fault_injected:
        rospy.logwarn(f"Injecting fault in drone {faulty_drone_index}")
        upside_down_orientation = tf_trans.quaternion_from_euler(0, 3.14159, 0)  # 180 degrees around Y-axis
        poses[faulty_drone_index].pose.orientation = Quaternion(*upside_down_orientation)
        # Disarm the drone to simulate fault and set falling flag
        arming_clients[faulty_drone_index].call(CommandBoolRequest(value=False))
        # Initiate landing for the faulty drone
        set_mode_clients[faulty_drone_index].call(SetModeRequest(custom_mode="AUTO.LAND"))
        fault_injected = True
        falling = True

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

        if all_drones_reached_target():
            rospy.loginfo(f"All drones reached waypoint {target_pose_index}")
            if target_pose_index == 2 and not fault_injected:
                inject_fault()
            if target_pose_index < len(waypoints) - 1:
                target_pose_index += 1
                rospy.loginfo(f"Moving to waypoint {target_pose_index}")
            else:
                # All waypoints have been covered, initiate landing and disarming
                rospy.loginfo("All waypoints covered, initiating landing sequence")
                trigger_landing_and_disarm()
                break  # Exit the main loop after landing

        for i in range(DRONE_COUNT):
            if fault_injected and i == faulty_drone_index:
                if falling and poses[faulty_drone_index].pose.position.z > 0:
                    poses[faulty_drone_index].pose.position.z -= 0.1  # Simulate falling by decreasing altitude
                continue
            set_target_pose(poses[i], waypoints[target_pose_index], offsets[i])
            poses[i].header.stamp = current_time
            local_pos_pubs[i].publish(poses[i])
            rospy.loginfo(f"Drone {i} target pose: {poses[i].pose}")

        rate.sleep()
