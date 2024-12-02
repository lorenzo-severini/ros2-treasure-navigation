# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Launch file for the Proyecto ROS2 navigation service and client
# Authors: Lorenzo Severini, David Zipperstein

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generates the launch description for launching both the navigation service and client nodes, with a delay for the client node to ensure the service is ready."""
    
    # Returns the launch description containing service and client nodes
    return LaunchDescription([
        # Declare a launch argument to enable or disable simulation time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',  # Default to true for use with simulation (e.g., Gazebo)
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch the navigation service node
        Node(
            package='proyecto_ros2',  # ROS2 package containing the service node
            executable='service_navigation_plan',  # Executable name of the service node
            name='navigation_service',  # Node name
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],  # Use simulation time parameter
        ),

        # Launch the navigation client node after a delay to ensure service is ready
        TimerAction(
            period=5.0,  # Delay of 5 seconds before starting the client
            actions=[
                Node(
                    package='proyecto_ros2',  # ROS2 package containing the client node
                    executable='client_async_navigation',  # Executable name of the client node
                    name='navigation_client',  # Node name
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],  # Use simulation time parameter
                )
            ]
        )
    ])
