#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

TOLERANCE = 0.1
LANDING_ALTITUDE_THRESHOLD = 0.1

# Update the drone count to 10
DRONE_COUNT = 8

# State and pose for each drone
states = [State() for _ in range(DRONE_COUNT)]
poses = [PoseStamped() for _ in range(DRONE_COUNT)]
target_pose_index = 0
waypoints = [
    [0.0,0.0,60],[77.02369149262199, 36.031588656829335, 60], [131.071074477866, 36.031588656829335, 60], [185.11845746311002, 36.031588656829335, 60], [239.165840448354, 36.031588656829335, 60], 
    [293.213223433598, 36.031588656829335, 60], [347.26060641884203, 36.031588656829335, 60], [401.307989404086, 36.031588656829335, 60], [455.35537238933, 36.031588656829335, 60],[0.0,0.0,60.0]
]
# Offsets for the follower drones
offsets = [i * -72.06317731365867 for i in range(DRONE_COUNT)]

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
    target.pose.position.x = waypoint[0] 
    target.pose.position.y = waypoint[1] + offset
    target.pose.position.z = waypoint[2]

def all_drones_reached_target():
    for i in range(DRONE_COUNT):
        if not (abs(poses[i].pose.position.x - waypoints[target_pose_index][0]) < TOLERANCE and
                abs(poses[i].pose.position.y - waypoints[target_pose_index][1] - offsets[i]) < TOLERANCE and
                abs(poses[i].pose.position.z - waypoints[target_pose_index][2]) < TOLERANCE):
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

if __name__ == "__main__":
    rospy.init_node("offb_node")

    state_subs = []
    pose_subs = []
    local_pos_pubs = []
    arming_clients = []
    set_mode_clients = []

    # Setup for each drone
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

    while not rospy.is_shutdown() and not all(state.connected for state in states):
        rate.sleep()

    # Initialize poses
    for i in range(DRONE_COUNT):
        poses[i].header.frame_id = 'base_link'
        poses[i].header.stamp = rospy.Time.now()
        poses[i].pose.position.x = 0.0
        poses[i].pose.position.y = 0.0
        poses[i].pose.position.z = 10.0

    # Send initial setpoints
    for _ in range(100):
        if rospy.is_shutdown():
            break
        for pub, pose in zip(local_pos_pubs, poses):
            pub.publish(pose)
        rate.sleep()

    offb_set_modes = [SetModeRequest(custom_mode='OFFBOARD') for _ in range(DRONE_COUNT)]
    arm_cmds = [CommandBoolRequest(value=True) for _ in range(DRONE_COUNT)]
    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Set mode and arm drones
        for i in range(DRONE_COUNT):
            if states[i].mode != "OFFBOARD" and (current_time - last_req) > rospy.Duration(5.0):
                if set_mode_clients[i].call(offb_set_modes[i]).mode_sent:
                    rospy.loginfo(f"OFFBOARD enabled for uav{i}")
                last_req = current_time
            elif not states[i].armed and (current_time - last_req) > rospy.Duration(5.0):
                if arming_clients[i].call(arm_cmds[i]).success:
                    rospy.loginfo(f"uav{i} armed")
                last_req = current_time

        # Set target poses
        if all_drones_reached_target():
            if target_pose_index < len(waypoints) - 1:
                target_pose_index += 1
            else:
                trigger_landing_and_disarm()
                break

        # Publish target poses
        for i in range(DRONE_COUNT):
            set_target_pose(poses[i], waypoints[target_pose_index], offsets[i])
            poses[i].header.stamp = current_time
            local_pos_pubs[i].publish(poses[i])

        rate.sleep()
