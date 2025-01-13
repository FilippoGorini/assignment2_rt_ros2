#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs_srvs.msg import Pos2D
from custom_msgs_srvs.srv import StartStop
from nav_msgs.msg import Odometry

class MoveRobot(Node):
    # This is the constructor function for the MoveRobot class, which initializes some of the needed attributes
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_position = self.create_publisher(Pos2D, '/position', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.serv_start_stop = self.create_service(StartStop, 'start_stop', self.start_stop_callback)
        self.x = 0.0                                                # Robot's x position
        self.y = 0.0                                                # Robot's y position
        self.upper_bound_exceeded = False                           # Flag that indicates the upper y bound has been exceeded
        self.lower_bound_exceeded = False                           # Flag that indicates the lower y bound has been exceeded
        self.turn_direction = 1
        self.move = True                                            # New flag which indicates if robot should move or stop


    def start_stop_callback(self, request, response):
        self.move = request.start                                    # Set flag to what the user requested  
        if request.start == True:
            response.message = "Robot started!" 
        elif request.start == False:
            response.message = "Robot stopped!"
        return response.message
                 
        

    # This is the callback function executed everytime the /odom topic is updated. It stores the position of the robot ...
    # ... and triggers the move_robot function, which manages the velocity commands to the robot
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.move_robot()
        self.publish_position()

    # new publish position function
    def publish_position(self):
        pos = Pos2D()
        pos.x = self.x
        pos.y = self.y
        self.pub_position.publish(pos)

    # This function should make the robot move in a "s" pattern like we did in the turtlesim exercises, while also making ...
    # ... sure that the robot stays within bounds
    def move_robot(self):
        vel_msg = Twist()

        if self.move == True:
            # The logic here is to invert the turn direction when getting too close to the y boundaries in order ...
            # ... to not exceed them and go back and forth over all the grid. Once the flag has been set to True, the turn direction ...
            # ... is reversed only once that the robot is moving in a straight line (for -9<x<9), to ensure that we don't change it ...
            # ... during the turn itself. This ensure that the robot keeps moving around the world without getting out of its boundaries
            if self.y > 9.0:
                self.upper_bound_exceeded = True
            elif self.y < -9.0:
                self.lower_bound_exceeded = True

            # Here the logic is almost the same as in the turtlesim exercise (aside from the use of self.turn_direction)
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
        elif self.move == False:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
 
        self.publisher_.publish(vel_msg)                            # Publish the velocity command to the /cmd_vel topic

# Main function that sets up the node and create an instance of the MoveRobot() class. This is the entry point for the setup.py file
def main(args=None):
    rclpy.init(args=args)                                           # Initialize ROS 2
    node = MoveRobot()                                              # Create an instance of the MoveRobot class
    rclpy.spin(node)                                                # Run the node
    node.destroy_node()                                             # Clean up
    rclpy.shutdown()                                                # Shutdown ROS 2 

if __name__ == '__main__':
    main()
