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

# Treasure Hunt Navigation for the Proyecto ROS2
# Authors: Lorenzo Severini, David Zipperstein

import rclpy

from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg._bool import Bool
import tf_transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from time import sleep

class TreasureHunt(Node):
    """Node for navigating a robot to a treasure using Nav2 and odometry data."""
    
    def __init__(self, distance_subscriber):
        """Initializes the TreasureHunt node with necessary publishers and subscribers, and starts the treasure hunt by publishing a message."""
        
        # Initialize the node with the name 'treasure_hunt'
        super().__init__('treasure_hunt')
        self.nav = BasicNavigator()  # Nav2 navigation object
        self.publisher_ = self.create_publisher(Bool, '/busquedaTesoro', 10)  # Publisher for starting treasure hunt
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # Subscribe to odometry
        
        self.distance_subscriber = distance_subscriber  # Reference to the DistanceSubscriber instance
        
        # Initialize distances to large values to represent an unknown state
        self.dist_x = float("inf")
        self.dist_y = float("inf")
        self.dist_euclid = float("inf")
        
        msg = Bool()
        msg.data = True  # Indicate that the treasure hunt is active
        self.publisher_.publish(msg)
        self.get_logger().info(f'Activated search with value "{msg.data}"')

    def odom_callback(self, msg):
        """Callback for handling odometry messages and initiating navigation."""
        
        self.destroy_subscription(self.subscription)  # Unsubscribe after receiving the first odometry message
        self.initial_pose = pose_from_msg(msg, self.nav)  # Convert odometry to initial pose
        self.get_logger().info(f'Initial pose: x={self.initial_pose.pose.position.x}, y={self.initial_pose.pose.position.y}')
        
        self.nav.setInitialPose(self.initial_pose)  # Set the initial pose for Nav2
        self.nav.waitUntilNav2Active()  # Wait until Nav2 is active
        
        self.init_x = self.initial_pose.pose.position.x
        self.init_y = self.initial_pose.pose.position.y
        
        # Wait until valid distance data is received
        while self.dist_x == float("inf") or self.dist_y == float("inf") or self.dist_euclid == float("inf"):
            self.dist_x, self.dist_y, self.dist_euclid = self.distance_subscriber.update_distance()
        
        goal_x = self.init_x + self.dist_x
        goal_y = self.init_y + self.dist_y
        
        treasure_goal_pose = create_pose_stamped(self.nav, goal_x, goal_y, 0.0)  # Create the goal pose
        self.nav.goToPose(treasure_goal_pose)  # Send the robot to the goal
        self.get_logger().info(f'Starting navigation to goal with coordinates: x={goal_x}, y={goal_y}')

        self.timer_counter = 90  # Timer for the search duration
        
        while not self.dist_euclid <= 0.5 or self.nav.isTaskComplete():
            sleep(1)    # Permits to check the timer and decrease it every second
            if self.timer_counter > 0:
                self.get_logger().info(f'Remaining time: {self.timer_counter} seconds.')
                self.timer_counter -= 1
                self.dist_x, self.dist_y, self.dist_euclid = self.distance_subscriber.update_distance()
            else:
                self.get_logger().info('Treasure search out of time: 90 seconds have passed!')
                msg = Bool()
                msg.data = False  # Indicate search failure
                self.publisher_.publish(msg)
                break
        
        if not self.nav.isTaskComplete():
            self.nav.cancelTask()  # Cancel navigation if not completed
        
        if self.dist_euclid <= 0.5:
            self.get_logger().info('The treasure has been found!')
            self.distance_subscriber.treasure_found = True
            self.destroy_node()  # Shutdown the node if treasure is found
            
class DistanceSubscriber(Node):
    """Node that subscribes to distance updates indicating the position of the treasure."""
    
    def __init__(self):
        super().__init__('distance_subscriber')
        self.subscription = self.create_subscription(Vector3, '/distanciaTesoro', self.listener_callback, 10)  # Subscribe to distance topic
        self.subscription  # Keep a reference to the subscription
        
        self.treasure_found = False  # Flag to track if the treasure has been found

    def listener_callback(self, msg):
        """Callback function that processes distance messages."""
        
        if not self.treasure_found:
            # Log the received distances
            self.get_logger().info(f'Distances: x={msg.x}, y={msg.y} z={msg.z}')
            self.dist_x = msg.x
            self.dist_y = msg.y
            self.dist_euclid = msg.z  # Euclidean distance to the treasure
        else:
            self.destroy_subscription(self.subscription)  # Unsubscribe once the treasure is found
            self.destroy_node()  # Destroy the node after the treasure is found
            
    def update_distance(self):
        """Return the current distance values to the TreasureHunt node."""
        return self.dist_x, self.dist_y, self.dist_euclid
        
        
def pose_from_msg(msg, nav):
    """Converts odometry message to PoseStamped message suitable for Nav2."""
    
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg() 
    initial_pose.pose.position = msg.pose.pose.position
    initial_pose.pose.orientation = msg.pose.pose.orientation
    return initial_pose  # Return the PoseStamped message
    
def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    """Creates a PoseStamped message with specified position and orientation."""
    
    # Convert the desired orientation from Euler angles to quaternion
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x  # Set the x position of the goal
    goal_pose.pose.position.y = position_y  # Set the y position of the goal
    goal_pose.pose.position.z = 0.0         # Set the z position to 0 (on the ground)
    goal_pose.pose.orientation.x = q_x      
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose  # Return the created goal pose

def main(args=None):
    """Main function to initialize ROS2 nodes and start the treasure hunt process."""
    
    rclpy.init(args=args)  
    
    # Create instances of DistanceSubscriber and TreasureHunt nodes
    distance_subscriber = DistanceSubscriber()
    treasure_hunt = TreasureHunt(distance_subscriber)
    
    # Create a multi-threaded executor to manage the nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(treasure_hunt)        # Add the TreasureHunt node to the executor
    executor.add_node(distance_subscriber)  # Add the DistanceSubscriber node to the executor

    try:
        executor.spin()  # Start the executor to process the nodes
    finally:
        treasure_hunt.destroy_node()        # Ensure the TreasureHunt node is destroyed after completion
        distance_subscriber.destroy_node()  # Ensure the DistanceSubscriber node is destroyed
        rclpy.shutdown()

if __name__ == '__main__':
    main()
