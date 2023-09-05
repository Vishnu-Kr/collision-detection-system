#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(100)  # 10 Hz

    def scan_callback(self, data):
        # Extract laser scan data
        ranges = data.ranges
        l=len(ranges)

        # Calculate distances in four directions
        # forward_distance = min(ranges[0:45] + ranges[-45:])
        # backward_distance = min(ranges[135:225])
        # left_distance = min(ranges[225:315])
        # right_distance = min(ranges[45:135])

        forward_distance = ranges[360]
        backward_left_distance = ranges[719]
        backward_right_distance = ranges[0]
        left_distance = ranges[600]
        right_distance = ranges[120]
        # Print the distances
        # rospy.loginfo(" ")
        # rospy.loginfo("======= No Obstacle Detected ======= ")
        # rospy.loginfo(" ")
        # rospy.loginfo("Forward Distance       : %.2f m" % forward_distance)
        # # rospy.loginfo("Backward Distance: %.2f m" % backward_distance)
        # rospy.loginfo("Left Distance          : %.2f m" % left_distance)
        # rospy.loginfo("Right Distance         : %.2f m" % right_distance)
        # rospy.loginfo("Backward Left Distance : %.2f m" % backward_left_distance)
        # rospy.loginfo("Backward Right Distance: %.2f m" % backward_right_distance)


        # Check for collision in forward or backward directions
        if forward_distance < 2 or left_distance < 2 or right_distance < 2 or backward_right_distance < 2 or backward_left_distance < 2 :
            rospy.logwarn("===== Collision Warning! =====")
            self.stop_robot()

            # If the obstacle is very close, move in the opposite direction
            if forward_distance < 1.5 :
                rospy.logwarn("Collision Warning! Obstacle detected in front side")
                self.move_backwards()

            elif left_distance < 1 :
                rospy.logwarn("Collision Warning! Obstacle detected in left side")
                self.move_backwards()

            elif right_distance < 1:
                rospy.logwarn("Collision Warning! Obstacle detected in right side")
                self.move_backwards()

            elif backward_right_distance < 1.2:
                rospy.logwarn("Collision Warning! Obstacle detected in backward right side")
                self.move_forwards()
                    
            elif backward_left_distance < 1.2:
                rospy.logwarn("Collision Warning! Obstacle detected in backward left side")
                self.move_forwards()
        else:
            rospy.loginfo("======= No Obstacle Detected ======= ")
        # Optionally, you can add logic for left and right directions here

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def move_backwards(self):
        twist = Twist()
        twist.linear.x = -0.3  # Adjust the linear velocity as needed
        self.cmd_vel_pub.publish(twist)

    # def move_backwards(self):
    #     twist = Twist()
    #     twist.linear.x = -0.5  # Adjust the linear velocity as needed
    #     self.cmd_vel_pub.publish(twist)
    
    def move_forwards(self):
        twist = Twist()
        twist.linear.x = 0.5  # Adjust the linear velocity as needed
        self.cmd_vel_pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            # rospy.sleep(0.001)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
