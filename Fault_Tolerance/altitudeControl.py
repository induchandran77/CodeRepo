import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

class Ground:
    def __init__(self):
        
        # Set the altitude value to be sent to the drone
        self.altitude = 60.0
        self.velocity = 17.0

        # Set the hotspot location to be sent to the drone
        self.hotspot = PoseStamped()
        self.hotspot.pose.position.x = 10.0
        self.hotspot.pose.position.y = 10.0
        self.hotspot.pose.position.z = 70.0
        
        # Set the step size for altitude updates
        self.step_size = 2.0
        
        # Create a publisher for sending the altitude value to the drone
        self.altitude_pub = rospy.Publisher('Altitude', Float32, queue_size=10)
        self.velocity_pub = rospy.Publisher("Velocity", Float32, queue_size=10)
        # Publish the initial altitude value
        self.altitude_pub.publish(self.altitude)
        self.velocity_pub.publish(self.velocity)
        # Create a subscriber for receiving the value from the drone
        self.value_sub = rospy.Subscriber('ClarityValue', Float32, self.handle_value)
        
        self.hotspot_pub = rospy.Publisher("NewPosition",PoseStamped,queue_size=10)

        # Create a subscriber for receiving the acknowledgement message from the drone
        self.ack_sub = rospy.Subscriber('ACK0', PoseStamped, self.handle_acknowledgement)
        
    def handle_acknowledgement(self, msg):
        # When the acknowledgement message is received, send the hotspot location to the drone
        rospy.loginfo("Received acknowledgement from drone0. Sending hotspot location.")
        self.hotspot_pub.publish(self.hotspot)
        
    def handle_value(self, msg):
        if msg.data >= 0.8:
            rospy.loginfo("Received value from drone0: %f. Ignoring.", msg.data)
        else:
            rospy.loginfo("Received value from drone0: %f. Updating altitude.", msg.data)

            # Update the altitude by subtracting the step size
            self.hotspot.pose.position.z  -= self.step_size
            # Send the updated altitude value to the drone
            self.hotspot_pub.publish(self.hotspot)

if __name__ == '__main__':
    rospy.init_node('ground_node')
    try:
        g = Ground()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()