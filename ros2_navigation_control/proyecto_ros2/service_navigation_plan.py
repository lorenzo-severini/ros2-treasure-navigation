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

# Service for the Proyecto ROS2 navigation service
# Authors: Lorenzo Severini, David Zipperstein

from ros2_navigation_interfaces.srv import NavigationService
import rclpy
from enum import Enum
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf_transformations

class TaskResult(Enum):
    """Enumeration for the possible results of a task: UNKNOWN, SUCCEEDED, CANCELED, and FAILED."""
    
    UNKNOWN = 0     # Default result for unknown tasks
    SUCCEEDED = 1   # Task completed successfully
    CANCELED = 2    # Task was canceled
    FAILED = 3      # Task failed


class ServiceNavigationPlan(Node):
    """Handles service-based navigation tasks, subscribing to Odometry data and providing a service to execute different navigation plans (e.g., Patrol, GoToExit)."""
    
    def __init__(self):
        """Initializes the node, subscribes to odometry data, sets up the initial pose, and creates the navigation service."""        
        # Initialize the node and the navigator
        self.nav = BasicNavigator()
        super().__init__('service_navigation_plan')  # Name of the node
        
        # Subscribe to the odometry topic to get the robot's initial position
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    
    
    def odom_callback(self, msg):
        """Processes Odometry data, sets the robot's initial pose, and activates the navigation system."""        
        # Stop the subscription after receiving the initial position
        self.destroy_subscription(self.subscription)
        
        # Set up the initial pose using odometry data
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.initial_pose.pose.position = msg.pose.pose.position
        self.initial_pose.pose.orientation = msg.pose.pose.orientation
        
        self.get_logger().info(f'Initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}')
        
        # Set the robot's initial pose and activate the navigation system
        self.nav.setInitialPose(self.initial_pose)
        self.nav.waitUntilNav2Active()
        
        # Create the navigation service
        self.srv = self.create_service(NavigationService, 'navigation_plan', self.navigation_plan_callback)


    def navigation_plan_callback(self, request, response):
        """Handles service requests by executing the requested navigation plan (Patrol or GoToExit) and providing the result."""
        
        if request.nav_type == 'Patrol':
            response.result = self.Patrol()             # Execute the patrol routine
        elif request.nav_type == 'GoToExit':
            response.result = self.GoToExit()           # Execute the go-to-exit routine
        else:
            response.result = TaskResult.UNKNOWN.value  # Unknown navigation type
            
        return response


    def Patrol(self):
        """Executes a patrol routine, navigating to several predefined waypoints around the house, and returns the result of the navigation task."""

        waypoints = [
            create_pose_stamped(self.nav, 0.92, 0.35, 3.14),            # In front of entrance
            create_pose_stamped(self.nav, -4.27, 3.18, 2.63),           # Before entrance of left rooms, from below
            create_pose_stamped(self.nav, -6.34, -2.73, -1.56),         # End of the left rooms
            create_pose_stamped(self.nav, -1.46, 3.15, -0.31),          # Big left room, above the table
            create_pose_stamped(self.nav, 1.02, 2.6, -1.55),            # Tiny central room, accessed from right
            create_pose_stamped(self.nav, 6.29, 4.13, -0.94),           # Big right room, above the table
            create_pose_stamped(self.nav, 6.29, -4.38, -0.08),          # Most right room, bottom from left
            create_pose_stamped(self.nav, 6.93, -1.11, 2.13)            # Most right room, above table from right
        ]
        
        self.nav.followWaypoints(waypoints)  # Start following waypoints
        
        # Continuously check if the navigation task is complete
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            print(feedback)
        
        return self.nav.getResult().value

    def GoToExit(self):
        """Executes a routine to navigate directly to the exit and returns the result of the navigation task."""
        
        exit_goal = create_pose_stamped(self.nav, 1.16, -1.38, -1.56)
        self.nav.goToPose(exit_goal)  # Navigate to the exit
        
        # Check task completion status
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            print(feedback)
            
        return self.nav.getResult().value

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    """Creates a PoseStamped message with the given position and orientation, to be used as a navigation goal."""
    
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose


def main(args=None):
    """Initializes the ROS2 node and starts the service navigation plan, allowing navigation tasks to be requested."""
    
    rclpy.init(args=args)
    
    service_navigation_plan = ServiceNavigationPlan()
    rclpy.spin(service_navigation_plan) 
    
    rclpy.shutdown()  

if __name__ == '__main__':
    main()