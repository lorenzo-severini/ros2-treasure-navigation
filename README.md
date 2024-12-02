# ROS2 Project: Navigation and Treasure Hunt

This repository contains ROS2 packages for controlling a TurtleBot3 in a Gazebo simulation. It includes both navigation control services and a treasure hunt functionality.

---

## Mapping the House

**Objective**: Create a detailed map of a simulated house environment using Turtlebot3 in the **Gazebo** simulator.

### Process:
1. Launch the simulation with Turtlebot3 and the house model.
2. Use **ROS2**, **Nav2**, and **SLAM Toolbox** for automatic mapping.
3. Explore the entire environment to generate a complete map.
4. Save the generated map file.
5. Correct any errors in the map using an editing program like **GIMP**.

### Result:

![Mapped house](https://github.com/lorenzo-severini/ros2-treasure-navigation/blob/main/ros2_navigation_control/ros2_navigation_control/house_map.pgm)

Above there is the complete and corrected house map saved and edited for future use in robot navigation.



## Command Service and Test Client

**Objective**: Create a service in ROS2 to handle specific navigation commands (e.g., "Patrol" and "Go to the exit") and a client to test the service.

### Process:
1. **Service Server**:
   - Create a ROS2 service to handle navigation commands for the **Turtlebot3**, such as navigating between waypoints or to a predefined exit.
   - Implement a feedback logic to report if the command was completed successfully or encountered any issues.
2. **Service Client**:
   - Develop a client that sends commands to the server and receives responses, such as "Patrol" or "GoToExit".
   - Handle the responses to provide feedback to the user on whether the command was successful.

## Treasure Hunt Script

**Objective**: Implement a treasure hunt script that guides the robot towards the treasure based on distance data from the **/distanciaTesoro** topic.

### Process:
1. Use the **busquedaTesoro.deb** package to obtain distance data (X, Y, and Euclidean distance) between the robot and the treasure.
2. Create an algorithm that guides the robot towards the treasure, stopping if the treasure is found (distance <= 0.5 meters) or if the time exceeds 90 seconds.
3. Display progress on the search and notify whether the search was completed successfully or failed.

### Result:
A treasure hunt system that guides the robot to locate the hidden object in the shortest time possible.

---

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

---

## Conclusions

This project demonstrates the use of ROS2 for controlling the TurtleBot3 in a Gazebo simulation, providing both a navigation control system and a treasure hunt simulation. The integration of ROS2's navigation stack (Nav2) allows for the robot's autonomous navigation, and the treasure hunt functionality provides a fun and interactive exploration task for the robot.


## Authors

* [lorenzo-severini](https://github.com/lorenzo-severini): Co-creator and developer of the ROS2 navigation and treasure hunt functionality.
* [David-Z2812](https://github.com/david-z2812): Co-creator and developer of the ROS2 navigation and treasure hunt functionality.


## Credits

* [euivmar](https://github.com/euivmar) for the development of the 'busquedaTesoro' package, which provides the treasure hunt functionality.


## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
