#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveRobot(Node):
    # This is the constructor function for the MoveRobot class, which initializes some of the needed attributes
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.x = 0.0  
        self.y = 0.0
        self.upper_bound_exceeded = False
        self.lower_bound_exceeded = False
        self.turn_direction = 1

    # This is the callback function executed everytime the 7odom topic is updated. It stores the position of the robot ...
    # ... and triggers the move_robot function
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.move_robot()

    # This function should make the robot move in a "s" pattern like we did in the turtlesim exercises, while also making ...
    # ... sure that the robot stays within bounds
    def move_robot(self):
        vel_msg = Twist()

        # Ideally the logic would be to invert the turn direction when getting too close to the y boundaries in order ...
        # ... to not exceed them and go back and forth over all the grid. The logic is still wrong though
        if self.y > 0.9:
            self.upper_bound_exceeded = True
        elif self.y < -0.9:
            self.lower_bound_exceeded = True

        # Here the logic is exactly the same as in the turtlesim exercise (aside from the use of self.turn_direction)
        if self.x > 9.0:                
            vel_msg.linear.x = 1.0 
            vel_msg.angular.z = self.turn_direction * 3.0           
        elif self.x < -9.0:
            vel_msg.linear.x = 1.0 
            vel_msg.angular.z = self.turn_direction * -3.0          
        else:
            vel_msg.linear.x = 1.0 
            vel_msg.angular.z = 0.0
            if self.upper_bound_exceeded:
                self.turn_direction = -1
                self.upper_bound_exceeded = False
            elif self.lower_bound_exceeded:
                self.turn_direction = 1
                self.lower_bound_exceeded = False
 
        self.publisher_.publish(vel_msg)

# Main function that sets up the node and create an instance of the MoveRobot() class
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
