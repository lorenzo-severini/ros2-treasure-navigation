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

# Client for the Proyecto ROS2 navigation service
# Authors: Lorenzo Severini, David Zipperstein

import rclpy
from rclpy.node import Node
from enum import Enum
from ros2_navigation_interfaces.srv import NavigationService

# Enumeration to represent the possible results of navigation tasks
class TaskResult(Enum):
    """Enumeration for the possible results of navigation tasks: UNKNOWN, SUCCEEDED, CANCELED, and FAILED."""
    
    UNKNOWN = 0     # Task result is unknown or invalid
    SUCCEEDED = 1   # Task completed successfully
    CANCELED = 2    # Task was canceled
    FAILED = 3      # Task failed to complete

class ClientAsyncNavigation(Node):
    """Client node that handles asynchronous communication with the navigation service, sending requests and processing responses."""
    
    def __init__(self):
        """Initializes the client node, creates a client for the navigation service, and waits until the service is available."""
        
        super().__init__('client_async_navigation')
        
        # Create a client to interact with the 'navigation_plan' service
        self.cli = self.create_client(NavigationService, 'navigation_plan')
        
        # Wait until the service becomes available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.req = NavigationService.Request()  # Create a request object for the service


    def send_request(self, nav_type):
        """Sends a navigation request to the service asynchronously and waits for the response."""
        
        self.req.nav_type = nav_type                            # Set the requested navigation type
        self.future = self.cli.call_async(self.req)             # Send the request asynchronously
        rclpy.spin_until_future_complete(self, self.future)     # Wait for the response
        return self.future.result()                             # Return the result of the service call

    def response_result(self, response):
        """Processes and logs the response from the navigation service based on the result (SUCCEEDED, CANCELED, FAILED, UNKNOWN)."""
        
        self.get_logger().info(f'Navigation result: {response.result}')
        
        # Check the result and log the appropriate message
        if response.result == TaskResult.UNKNOWN.value:
            self.get_logger().info('Navigation result unknown.')
        elif response.result == TaskResult.SUCCEEDED.value:
            self.get_logger().info('Navigation succeeded.')
        elif response.result == TaskResult.CANCELED.value:
            self.get_logger().info('Navigation canceled.')
        else:
            self.get_logger().info('Navigation failed.')

def main():
    """Initializes the ROS2 client node, sends navigation requests ('Patrol' and 'GoToExit'), and logs the responses."""

    rclpy.init()

    client_async = ClientAsyncNavigation()

    # Send a 'Patrol' navigation request and process the response
    response = client_async.send_request('Patrol')
    client_async.response_result(response)

    # Send a 'GoToExit' navigation request and process the response
    response = client_async.send_request('GoToExit')
    client_async.response_result(response)

    client_async.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
