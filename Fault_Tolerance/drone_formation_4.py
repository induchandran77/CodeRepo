#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

TOLERANCE = 0.1

current_state_leader = State()
def state_cb_leader(msg):
    global current_state_leader
    current_state_leader = msg

current_pose_leader = PoseStamped()
def pose_cb_leader(msg):
    global current_pose_leader
    current_pose_leader = msg

current_state_follower1 = State()
def state_cb_follower1(msg):
    global current_state_follower1
    current_state_follower1 = msg

current_pose_follower1 = PoseStamped()
def pose_cb_follower1(msg):
    global current_pose_follower1
    current_pose_follower1 = msg

current_state_follower2 = State()
def state_cb_follower2(msg):
    global current_state_follower2
    current_state_follower2 = msg

current_pose_follower2 = PoseStamped()
def pose_cb_follower2(msg):
    global current_pose_follower2
    current_pose_follower2 = msg

current_state_follower3 = State()
def state_cb_follower3(msg):
    global current_state_follower3
    current_state_follower3 = msg

current_pose_follower3 = PoseStamped()
def pose_cb_follower3(msg):
    global current_pose_follower3
    current_pose_follower3 = msg

poses = [
    [0.0,0.0,10.0],[-18.301886792452827,25.754716981132074,10.0], [28.040540540540544,46.351351351351354,10.0], 
    [31.081081081081084,42.70270270270271,10.0], [-16.603773584905657,21.509433962264147,10.0], 
    [-14.905660377358489,17.264150943396224,10.0],[34.12162162162163,39.05405405405405,10.0], 
    [37.16216216216217,35.4054054054054,10.0], [-13.207547169811319,13.0188679245283,10.0], 
    [-11.509433962264149,8.773584905660375,10.0],[0.0,0.0,10.0],[0.0,0.0,0.0]
]

target_poses_leader = []

for i in range(12):
    pose = PoseStamped()
    pose.pose.position.x = poses[i][0]
    pose.pose.position.y = poses[i][1]
    pose.pose.position.z = poses[i][2]
    target_poses_leader.append(pose)

target_pose_index = 0
landing_triggered = False

def trigger_landing():
    global landing_triggered
    landing_triggered = True

def get_target_pose_leader():
    global current_pose_leader, target_poses_leader, target_pose_index
    if((abs(current_pose_leader.pose.position.x - target_poses_leader[target_pose_index].pose.position.x) < TOLERANCE) and \
        (abs(current_pose_leader.pose.position.y - target_poses_leader[target_pose_index].pose.position.y) < TOLERANCE) and \
        (abs(current_pose_leader.pose.position.z - target_poses_leader[target_pose_index].pose.position.z) < TOLERANCE)):
        if(target_pose_index < len(target_poses_leader) - 1):
            target_pose_index += 1
            return target_poses_leader[target_pose_index]
        else:
            trigger_landing()
            rospy.loginfo('Landing...')
            return target_poses_leader[-1]
    else:
        return target_poses_leader[target_pose_index]

def get_target_pose_follower(offset_x, offset_y=0.0, offset_z=0.0):
    global current_pose_leader
    follower_pose = PoseStamped()
    follower_pose.pose.position.x = current_pose_leader.pose.position.x + offset_x
    follower_pose.pose.position.y = current_pose_leader.pose.position.y + offset_y
    follower_pose.pose.position.z = current_pose_leader.pose.position.z + offset_z
    return follower_pose

if __name__ == "__main__":
    rospy.init_node("offb_node_0_py")

    # Leader drone
    state_sub_leader = rospy.Subscriber("/uav0/mavros/state", State, callback=state_cb_leader)
    pose_sub_leader = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, callback=pose_cb_leader)
    local_pos_pub_leader = rospy.Publisher("/uav0/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Follower drone 1
    state_sub_follower1 = rospy.Subscriber("/uav1/mavros/state", State, callback=state_cb_follower1)
    pose_sub_follower1 = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, callback=pose_cb_follower1)
    local_pos_pub_follower1 = rospy.Publisher("/uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # Follower drone 2
    state_sub_follower2 = rospy.Subscriber("/uav2/mavros/state", State, callback=state_cb_follower2)
    pose_sub_follower2 = rospy.Subscriber("/uav2/mavros/local_position/pose", PoseStamped, callback=pose_cb_follower2)
    local_pos_pub_follower2 = rospy.Publisher("/uav2/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Follower drone 3
    state_sub_follower3 = rospy.Subscriber("/uav3/mavros/state", State, callback=state_cb_follower3)
    pose_sub_follower3 = rospy.Subscriber("/uav3/mavros/local_position/pose", PoseStamped, callback=pose_cb_follower3)
    local_pos_pub_follower3 = rospy.Publisher("/uav3/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # Wait for services
    rospy.wait_for_service("/uav0/mavros/cmd/arming")
    arming_client_leader = rospy.ServiceProxy("/uav0/mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/uav0/mavros/set_mode")
    set_mode_client_leader = rospy.ServiceProxy("/uav0/mavros/set_mode", SetMode)

    rospy.wait_for_service("/uav1/mavros/cmd/arming")
    arming_client_follower1 = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/uav1/mavros/set_mode")
    set_mode_client_follower1 = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)

    rospy.wait_for_service("/uav2/mavros/cmd/arming")
    arming_client_follower2 = rospy.ServiceProxy("/uav2/mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/uav2/mavros/set_mode")
    set_mode_client_follower2 = rospy.ServiceProxy("/uav2/mavros/set_mode", SetMode)

    rospy.wait_for_service("/uav3/mavros/cmd/arming")
    arming_client_follower3 = rospy.ServiceProxy("/uav3/mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/uav3/mavros/set_mode")
    set_mode_client_follower3 = rospy.ServiceProxy("/uav3/mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state_leader.connected and not current_state_follower1.connected and not current_state_follower2.connected and not current_state_follower3.connected):
        rate.sleep()

    pose_leader = PoseStamped()
    pose_leader.header.frame_id = 'base_link'
    pose_leader.header.stamp = rospy.Time.now()
    pose_leader.pose.position.x = 0.0
    pose_leader.pose.position.y = 0.0
    pose_leader.pose.position.z = 10.0

    for i in range(100):
        if(rospy.is_shutdown()):
            break
        local_pos_pub_leader.publish(pose_leader)
        rate.sleep()

    offb_set_mode_leader = SetModeRequest()
    offb_set_mode_leader.custom_mode = 'OFFBOARD'
    arm_cmd_leader = CommandBoolRequest()
    arm_cmd_leader.value = True

    offb_set_mode_follower1 = SetModeRequest()
    offb_set_mode_follower1.custom_mode = 'OFFBOARD'
    arm_cmd_follower1 = CommandBoolRequest()
    arm_cmd_follower1.value = True

    offb_set_mode_follower2 = SetModeRequest()
    offb_set_mode_follower2.custom_mode = 'OFFBOARD'
    arm_cmd_follower2 = CommandBoolRequest()
    arm_cmd_follower2.value = True

    offb_set_mode_follower3 = SetModeRequest()
    offb_set_mode_follower3.custom_mode = 'OFFBOARD'
    arm_cmd_follower3 = CommandBoolRequest()
    arm_cmd_follower3.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        current_time = rospy.Time.now()
        
        if current_state_leader.mode != "OFFBOARD" and (current_time - last_req) > rospy.Duration(5.0):
            if set_mode_client_leader.call(offb_set_mode_leader).mode_sent:
                rospy.loginfo("OFFBOARD enabled for leader")
            last_req = current_time
        else:
            if not current_state_leader.armed and (current_time - last_req) > rospy.Duration(5.0):
                if arming_client_leader.call(arm_cmd_leader).success:
                    rospy.loginfo("Leader armed")
                last_req = current_time

        if current_state_follower1.mode != "OFFBOARD" and (current_time - last_req) > rospy.Duration(5.0):
            if set_mode_client_follower1.call(offb_set_mode_follower1).mode_sent:
                rospy.loginfo("OFFBOARD enabled for follower 1")
            last_req = current_time
        else:
            if not current_state_follower1.armed and (current_time - last_req) > rospy.Duration(5.0):
                if arming_client_follower1.call(arm_cmd_follower1).success:
                    rospy.loginfo("Follower 1 armed")
                last_req = current_time

        if current_state_follower2.mode != "OFFBOARD" and (current_time - last_req) > rospy.Duration(5.0):
            if set_mode_client_follower2.call(offb_set_mode_follower2).mode_sent:
                rospy.loginfo("OFFBOARD enabled for follower 2")
            last_req = current_time
        else:
            if not current_state_follower2.armed and (current_time - last_req) > rospy.Duration(5.0):
                if arming_client_follower2.call(arm_cmd_follower2).success:
                    rospy.loginfo("Follower 2 armed")
                last_req = current_time

        if current_state_follower3.mode != "OFFBOARD" and (current_time - last_req) > rospy.Duration(5.0):
            if set_mode_client_follower3.call(offb_set_mode_follower3).mode_sent:
                rospy.loginfo("OFFBOARD enabled for follower 3")
            last_req = current_time
        else:
            if not current_state_follower3.armed and (current_time - last_req) > rospy.Duration(5.0):
                if arming_client_follower3.call(arm_cmd_follower3).success:
                    rospy.loginfo("Follower 3 armed")
                last_req = current_time

        pose_leader = get_target_pose_leader()
        pose_leader.header.stamp = current_time
        local_pos_pub_leader.publish(pose_leader)

        pose_follower1 = get_target_pose_follower(offset_x=-2.0)
        pose_follower1.header.stamp = current_time
        local_pos_pub_follower1.publish(pose_follower1)

        pose_follower2 = get_target_pose_follower(offset_x=-4.0)
        pose_follower2.header.stamp = current_time
        local_pos_pub_follower2.publish(pose_follower2)

        pose_follower3 = get_target_pose_follower(offset_x=-6.0)
        pose_follower3.header.stamp = current_time
        local_pos_pub_follower3.publish(pose_follower3)

        if landing_triggered:
            rospy.loginfo("Setting drones to AUTO.LAND mode...")
            set_mode_client_leader.call(SetModeRequest(custom_mode="AUTO.LAND"))
            set_mode_client_follower1.call(SetModeRequest(custom_mode="AUTO.LAND"))
            set_mode_client_follower2.call(SetModeRequest(custom_mode="AUTO.LAND"))
            set_mode_client_follower3.call(SetModeRequest(custom_mode="AUTO.LAND"))
            rospy.sleep(10)  # Wait for landing
            rospy.loginfo("Disarming drones...")
            arming_client_leader.call(CommandBoolRequest(value=False))
            arming_client_follower1.call(CommandBoolRequest(value=False))
            arming_client_follower2.call(CommandBoolRequest(value=False))
            arming_client_follower3.call(CommandBoolRequest(value=False))
            break

        rate.sleep()
