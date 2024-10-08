#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

TOLERANCE = 0.1

current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

current_pose = PoseStamped()
target_pose_index = 0
def pose_cb(msg):
    global current_pose
    current_pose = msg

poses =[
    [0.0,0.0,10.0],[-18.301886792452827,25.754716981132074,10.0], [28.040540540540544,46.351351351351354,10.0], 
    [31.081081081081084,42.70270270270271,10.0], [-16.603773584905657,21.509433962264147,10.0], 
    [-14.905660377358489,17.264150943396224,10.0],[34.12162162162163,39.05405405405405,10.0], 
    [37.16216216216217,35.4054054054054,10.0], [-13.207547169811319,13.0188679245283,10.0], 
    [-11.509433962264149,8.773584905660375,10.0],[0.0,0.0,10.0],[0.0,0.0,0.0]
]

target_poses = []

for i in range(12):
    pose = PoseStamped()
    pose.pose.position.x = poses[i][0]
    pose.pose.position.y = poses[i][1]
    pose.pose.position.z = poses[i][2]
    target_poses.append(pose)

def trigger_landing():
    pass


def get_target_pose():
    global current_pose, target_poses, target_pose_index
    print("Target index: ", target_pose_index, "X error: ", (abs(current_pose.pose.position.x - target_poses[target_pose_index].pose.position.x)), "Y error: ", (abs(current_pose.pose.position.y - target_poses[target_pose_index].pose.position.y)), "Z error: ", (abs(current_pose.pose.position.z - target_poses[target_pose_index].pose.position.z)))
    print(abs(current_pose.pose.position.x - target_poses[target_pose_index].pose.position.x) < TOLERANCE)
    print(abs(current_pose.pose.position.y - target_poses[target_pose_index].pose.position.y) < TOLERANCE)
    print(abs(current_pose.pose.position.z - target_poses[target_pose_index].pose.position.z) < TOLERANCE)
    print(len(target_poses))

    if((abs(current_pose.pose.position.x - target_poses[target_pose_index].pose.position.x) < TOLERANCE) and \
        (abs(current_pose.pose.position.y - target_poses[target_pose_index].pose.position.y) < TOLERANCE) and \
        (abs(current_pose.pose.position.z - target_poses[target_pose_index].pose.position.z) < TOLERANCE)):
        if(target_pose_index < len(target_poses) - 1):
            target_pose_index += 1
            return target_poses[target_pose_index]
        else:
            print("Landing triggered.")
            trigger_landing()
            rospy.loginfo('Landing...')
            return target_poses[-1]
    else:
        return target_poses[target_pose_index]


if __name__ == "__main__":
    rospy.init_node("offb_node_0_py")

    state_sub = rospy.Subscriber("/uav0/mavros/state", State, callback = state_cb)
    
    pose_sub = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, callback = pose_cb)

    local_pos_pub = rospy.Publisher("/uav0/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/uav0/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/uav0/mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/uav0/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/uav0/mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()

    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 10.0

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        pose = get_target_pose()
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)

        rate.sleep()
