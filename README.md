# RT1 Assignment 2 (ROS2)
This repository contains a ROS 2 package implementing a simple node called `move_robot.py`, which is used to control a mobile robot's movement in the Gazebo simulation environment. 

## Running the Node
Ensure you have **ROS 2 (Foxy or compatible)** and that the package **robot_urdf** is available in your workspace. This package provides the Gazebo simulation environment in which the robot operates.
Then, clone this repository into your workspace's `src` folder:
```
cd ~/<your_workspace>/src
git clone https://github.com/FilippoGorini/assignment2_rt_ros2.git
```

Build the package:
```
cd ~/<your_workspace>
colcon build
```

Source ROS 2 and your workspace in the `.bashrc` file:
```
source /opt/ros/foxy/setup.bash
source ~/<your_workspace>/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
```

Once this is done, you can run the Gazebo simulation using the following command:
```
ros2 launch robot_urdf gazebo.launch.py
```

## Node `move_robot.py`
This node implements a ROS 2 node which interacts with a 3D simulation in Gazebo, making the robot move in a snake-like pattern.
To run it, after making sure that you have correctly built your workspace and that you have launched the Gazebo simulation, run this command:
```
ros2 run assignment2_rt_ros2 move_robot
```

### Main Features
- Subscribes to `/odom` to track the robot’s position (x, y).
- Publishes velocity commands to `/cmd_vel` to make the robot turn when it gets past the x axis boundaries, resulting in a snake-like pattern.
- Ensures that the y boundaries are also respected, by changing the robot turn direction.

### Code Structure
The `move_robot.py` node is implemented using a class-based approach, following ROS 2 best practices for node organization. 
The `main()` function manages the initialization of ROS 2, creates an instance of the `MoveRobot` class and keeps the node running.
The `MoveRobot` class implements the following methods:

- **`__init__()`**: This is the constructor of the class, which initializes the position attributes, a publisher and one subscriber. 
- **`odom_callback()`**: Callback function of the subscriber to `/odom`, which updates the robot position and calls the `move_robot` method
- **`move_robot()`**: Implements the logic to move the robot in an S-pattern by publishing Twist messages on the `/cmd_vel` topic:
    - Moves straight when -9 < x < 9.
    - Turns at the left and right boundaries (x = ±9).
    - Reverses turn direction when reaching y = ±9, ensuring the robot stays within the given world grid.


