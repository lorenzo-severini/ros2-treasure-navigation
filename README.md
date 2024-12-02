# ROS2 Project: Navigation and Treasure Hunt

This repository contains ROS2 packages for controlling a TurtleBot3 in a Gazebo simulation. It includes both navigation control services and a treasure hunt functionality.

## Installing the `.deb` Package

To install the custom `.deb` package for the treasure hunt functionality, follow these steps:

1. Download the package from the `deb_packages` directory in the repository:

   ```bash
   wget https://github.com/yourusername/your-repository/raw/main/deb_packages/tesoro_pkg_0.0.0-0jammy_amd64.deb
   ```

2. Install the package using `dpkg`:

   ```bash
   sudo dpkg --install tesoro_pkg_0.0.0-0jammy_amd64.deb
   ```

3. If there are any missing dependencies, run:

   ```bash
   sudo apt-get install -f
   ```

This will install the `tesoro_pkg` package and allow you to use the treasure hunt functionality in your ROS2 environment.

## Building the ROS2 Workspace

If you need to build the ROS2 packages locally, you can do so using `colcon`. First, clone the repository and set up your workspace:

```bash
# Create a ROS2 workspace and clone the repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/lorenzo-severini/ros2-treasure-navigation.git
cd ~/ros2_ws

# Build the workspace
colcon build
source install/setup.bash
```

## Running the ROS2 Nodes

### 1. Launching the navigation through waypoints

To execute the house navigation through waypoints from the workspace directory:

#### Terminal 1: Launch the TurtleBot3 Gazebo simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

#### Terminal 2: Launch the navigation stack
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/ros2_navigation_control/ros2_navigation_control/house_map.yaml
```

#### Terminal 3: Launch the navigation control
```bash
ros2 launch ros2_navigation_control navigation.launch.py
```

### 2. Launching the treasure hunt

To execute the treasure hunt functionality, follow the steps below to launch the corresponding nodes:

#### Terminal 1: Launch the TurtleBot3 Gazebo simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

#### Terminal 2: Launch the navigation stack
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/lorenzo/ros2_ws/src/ros2_navigation_control/ros2_navigation_control/house_map.yaml
```

#### Terminal 3: Run the treasure hunt node from the tesoro_pkg package
```bash
ros2 run tesoro_pkg tesoro_nodo
```

#### Terminal 4: Launch the treasure hunt to locate the treasure
```bash
ros2 run ros2_navigation_control treasure_hunt
```

## Conclusions
This project demonstrates the use of ROS2 for controlling the TurtleBot3 in a Gazebo simulation, providing both a navigation control system and a treasure hunt simulation. The integration of ROS2's navigation stack (Nav2) allows for the robot's autonomous navigation, and the treasure hunt functionality provides a fun and interactive exploration task for the robot.

## Authors
* Lorenzo Severini: Co-creator and developer of the ROS2 navigation and treasure hunt functionality.
* David Zipperstein: Co-creator and developer of the ROS2 navigation and treasure hunt functionality.

## Credits
* [euivmar](https://github.com/euivmar) for the development of the 'busquedaTesoro' package, which provides the treasure hunt functionality.

## License
This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
